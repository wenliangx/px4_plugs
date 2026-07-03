#!/usr/bin/env python3
"""
PX4 Link Monitor - ROS 1 Node

Monitors MAVLink link health in real-time and logs anomalous events for
post-mortem analysis.  Detects:

  1. TX queue overflow  (via /diagnostics)
  2. Excessive topic publish rates (configurable topic list)
  3. FCU connection state changes (heartbeat loss / recovery)
  4. GCS bridge state changes

All events are written to a timestamped log file under <log_dir> and
also published on the ~status latched topic.
"""

import os
import re
import time
import threading
from datetime import datetime

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ts() -> str:
    """ISO-8601 timestamp with microseconds, suitable for filenames and logs."""
    return datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")


def _fmt_dt(dt: datetime) -> str:
    return dt.strftime("%Y-%m-%d %H:%M:%S")


# ---------------------------------------------------------------------------
# TopicRateTracker
# ---------------------------------------------------------------------------

class TopicRateTracker:
    """Counts messages on a single topic and reports average rate over a window."""

    def __init__(self, topic: str, window: float):
        self.topic = topic
        self.window = window
        self._count = 0
        self._start = time.time()
        self._lock = threading.Lock()

    def tick(self):
        with self._lock:
            self._count += 1

    def rate(self) -> float:
        """Average Hz since last reset (or since creation)."""
        with self._lock:
            elapsed = time.time() - self._start
            if elapsed <= 0.0:
                return 0.0
            return self._count / elapsed

    def snapshot_and_reset(self) -> tuple:
        """Return (topic, rate_hz) and reset the counter."""
        with self._lock:
            elapsed = time.time() - self._start
            rate = self._count / elapsed if elapsed > 0 else 0.0
            self._count = 0
            self._start = time.time()
            return (self.topic, rate)


# ---------------------------------------------------------------------------
# PX4LinkMonitor
# ---------------------------------------------------------------------------

class PX4LinkMonitor:
    def __init__(self):
        self._load_params()
        self._init_log_file()
        self._lock = threading.Lock()

        # Per-topic rate trackers
        self._trackers = {}
        for t in self.watch_topics:
            self._trackers[t] = TopicRateTracker(t, self.rate_window)
            rospy.Subscriber(t, rospy.AnyMsg, self._make_tick_cb(t))

        # Diagnostic subscriber
        rospy.Subscriber("/diagnostics", DiagnosticArray, self._diag_cb)

        # Periodic rate reporter
        rospy.Timer(rospy.Duration(self.rate_window), self._rate_timer_cb)

        # Status topic (latched)
        self._status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)

        # Services
        rospy.Service("~status_report", Trigger, self._handle_status_report)

        # State
        self._fcu_connected = None
        self._gcs_connected = None
        self._overflow_count = 0
        self._last_overflow_time = 0.0
        self._overflow_message_ids = set()
        self._session_start = datetime.now()

        self._log_event("SESSION_START", "Link Monitor started")
        self._publish_status()
        rospy.loginfo("PX4 Link Monitor initialized (log: %s)", self._log_path)

    # ---- params -----------------------------------------------------------

    def _load_params(self):
        self.log_dir = self._resolve_dir(
            rospy.get_param("~log_dir", "~/.px4_monitor")
        )
        self.rate_window = rospy.get_param("~rate_window", 10.0)
        self.overflow_cooldown = rospy.get_param("~overflow_cooldown", 5.0)
        watch_raw = rospy.get_param(
            "~watch_topics",
            "/mavros/setpoint_raw/local,/mavros/setpoint_position/local",
        )
        self.watch_topics = [t.strip() for t in watch_raw.split(",") if t.strip()]

    @staticmethod
    def _resolve_dir(path: str) -> str:
        return os.path.abspath(os.path.expanduser(path))

    # ---- log file ---------------------------------------------------------

    def _init_log_file(self):
        os.makedirs(self.log_dir, exist_ok=True)
        date_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(self.log_dir, "link_monitor_%s.log" % date_str)
        # Symlink to a fixed name for convenience
        latest = os.path.join(self.log_dir, "link_monitor_latest.log")
        if os.path.islink(latest) or os.path.exists(latest):
            os.unlink(latest)
        os.symlink(self._log_path, latest)

    def _log_event(self, event_type: str, detail: str):
        """Append a structured line to the log file."""
        line = "[%s] %-20s %s" % (_ts(), event_type, detail)
        with self._lock:
            with open(self._log_path, "a") as f:
                f.write(line + "\n")
        rospy.loginfo("%s | %s", event_type, detail)

    # ---- subscribers ------------------------------------------------------

    def _make_tick_cb(self, topic: str):
        """Return a lightweight callback that just ticks the tracker."""
        tracker = self._trackers[topic]

        def _cb(msg):
            tracker.tick()

        return _cb

    def _diag_cb(self, msg: DiagnosticArray):
        for status in msg.status:
            self._process_diag_status(status)

    def _process_diag_status(self, status: DiagnosticStatus):
        name = status.name
        level = status.level
        # mavros diagnostic names: "mavros: FCU connection", "mavros: GCS bridge"
        is_fcu = name.startswith("mavros: FCU")
        is_gcs = name.startswith("mavros: GCS")

        # --- connection state changes ---
        if is_fcu:
            conn = level != DiagnosticStatus.STALE
            if self._fcu_connected is None:
                self._fcu_connected = conn
                self._log_event("FCU_CONNECTION", "initial=%s" % conn)
            elif self._fcu_connected != conn:
                self._fcu_connected = conn
                self._log_event(
                    "FCU_CONNECTION_CHANGE",
                    "connected=%s" % conn,
                )
                self._publish_status()

        if is_gcs:
            conn = level != DiagnosticStatus.STALE
            if self._gcs_connected is None:
                self._gcs_connected = conn
                self._log_event("GCS_CONNECTION", "initial=%s" % conn)
            elif self._gcs_connected != conn:
                self._gcs_connected = conn
                self._log_event(
                    "GCS_CONNECTION_CHANGE",
                    "connected=%s" % conn,
                )
                self._publish_status()

        # --- TX overflow detection ---
        self._check_overflow(status)

    def _check_overflow(self, status: DiagnosticStatus):
        """Scan diagnostic messages for TX queue overflow indicators."""
        full_text = status.message
        for value in status.values:
            full_text += " " + str(value.key) + "=" + str(value.value)

        keywords = [
            "TX queue overflow",
            "send_message: TX queue overflow",
            "DROPPED Message-Id",
            "MAVConnSerial::send_message: TX queue overflow",
            "MAVConnUDP::send_message: TX queue overflow",
        ]
        hit = any(kw in full_text for kw in keywords)
        if not hit:
            return

        now = time.time()
        # Extract message-id if present
        msg_id = "unknown"
        if "Message-Id" in full_text:
            m = re.search(r"Message-Id\s*(\d+)", full_text)
            if m:
                msg_id = m.group(1)

        # Rate-limit: only log one overflow event per cooldown per msg-id
        key = msg_id
        if now - self._last_overflow_time < self.overflow_cooldown:
            if key in self._overflow_message_ids:
                return  # already logged this message-id recently

        self._overflow_message_ids.add(key)
        self._last_overflow_time = now
        self._overflow_count += 1

        # Capture current rates for context
        rate_info = self._build_rate_info()

        detail = (
            "count=%d msg_id=%s fcu_ok=%s gcs_ok=%s rates=[%s] diag=[%s]"
            % (
                self._overflow_count,
                msg_id,
                self._fcu_connected,
                self._gcs_connected,
                rate_info,
                full_text[:300],
            )
        )
        self._log_event("TX_OVERFLOW", detail)
        self._publish_status()

    # ---- periodic rate monitoring -----------------------------------------

    def _rate_timer_cb(self, event):
        """Called every rate_window seconds.  Logs topic rates."""
        parts = []
        for topic, tracker in self._trackers.items():
            t, rate = tracker.snapshot_and_reset()
            parts.append("%s=%.1fHz" % (t, rate))
        self._log_event("TOPIC_RATES", " ".join(parts) if parts else "no_topics_watched")
        self._publish_status()

    def _build_rate_info(self) -> str:
        parts = []
        for topic, tracker in self._trackers.items():
            parts.append("%s=%.1fHz" % (topic, tracker.rate()))
        return " ".join(parts) if parts else "none"

    # ---- status -----------------------------------------------------------

    def _build_status_text(self) -> str:
        lines = [
            "--- PX4 Link Monitor ---",
            "session: %s" % _fmt_dt(self._session_start),
            "elapsed: %.0f s" % (datetime.now() - self._session_start).total_seconds(),
            "fcu_connected: %s" % self._fcu_connected,
            "gcs_connected: %s" % self._gcs_connected,
            "overflow_count: %d" % self._overflow_count,
            "rate_window: %.1f s" % self.rate_window,
            "topic_rates: [%s]" % self._build_rate_info(),
            "log_file: %s" % self._log_path,
        ]
        return "\n".join(lines)

    def _publish_status(self):
        text = self._build_status_text()
        self._status_pub.publish(String(data=text))

    def _handle_status_report(self, req):
        text = self._build_status_text()
        return TriggerResponse(success=True, message=text)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    rospy.init_node("px4_link_monitor")
    PX4LinkMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()

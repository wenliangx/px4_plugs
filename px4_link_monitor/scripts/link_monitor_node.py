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

try:
    import yaml
except ImportError:
    yaml = None

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


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
# USBLinkInitializer
# ---------------------------------------------------------------------------

class USBLinkInitializer:
    """Applies a conservative MAVLink USB link profile after PX4 connects."""

    MESSAGE_IDS = {
        "HEARTBEAT": 0,
        "SYS_STATUS": 1,
        "SYSTEM_TIME": 2,
        "GPS_RAW_INT": 24,
        "ATTITUDE": 30,
        "ATTITUDE_QUATERNION": 31,
        "LOCAL_POSITION_NED": 32,
        "GLOBAL_POSITION_INT": 33,
        "NAV_CONTROLLER_OUTPUT": 62,
        "RC_CHANNELS": 65,
        "VFR_HUD": 74,
        "HIGHRES_IMU": 105,
        "TIMESYNC": 111,
        "ATTITUDE_TARGET": 83,
        "POSITION_TARGET_LOCAL_NED": 85,
        "ODOMETRY": 331,
    }

    STREAM_IDS = {
        "ALL": 0,
        "RAW_SENSORS": 1,
        "EXT_STAT": 2,
        "RC_CHANNELS": 3,
        "RAW_CONTROLLER": 4,
        "POSITION": 6,
        "EXTRA1": 10,
        "EXTRA2": 11,
        "EXTRA3": 12,
    }

    def __init__(
        self,
        connection_url: str,
        baudrate: int,
        timeout: float,
        config_file: str,
        dry_run: bool,
        log_event,
        log_dir: str,
    ):
        self.connection_url = connection_url
        self.baudrate = baudrate
        self.timeout = timeout
        self.config_file = os.path.abspath(os.path.expanduser(config_file))
        self.dry_run = dry_run
        self._log_event = log_event
        self._log_dir = log_dir
        self._mav = None

    def run(self, reason: str = "manual") -> dict:
        result = {
            "success": False,
            "reason": reason,
            "dry_run": self.dry_run,
            "message_intervals": 0,
            "streams": 0,
            "params": 0,
            "backup_file": "",
            "error": "",
        }

        try:
            cfg = self._load_config()
            self._log_event(
                "USB_INIT_START",
                "reason=%s dry_run=%s config=%s" % (reason, self.dry_run, self.config_file),
            )

            if not self.dry_run and not self._connect():
                result["error"] = "Cannot connect to PX4"
                self._log_event("USB_INIT_FAILED", result["error"])
                return result

            result["message_intervals"] = self._apply_message_intervals(
                cfg.get("message_intervals", {})
            )
            result["streams"] = self._apply_streams(cfg.get("streams", {}))

            params_enabled = bool(cfg.get("params_enabled", False))
            params = cfg.get("params", {}) or {}
            if params and params_enabled:
                result["backup_file"] = self._backup_params(params)
                result["params"] = self._apply_params(params)
            elif params:
                self._log_event(
                    "USB_INIT_PARAMS_SKIPPED",
                    "params_enabled=false count=%d" % len(params),
                )

            result["success"] = True
            self._log_event(
                "USB_INIT_DONE",
                "dry_run=%s intervals=%d streams=%d params=%d backup=%s"
                % (
                    self.dry_run,
                    result["message_intervals"],
                    result["streams"],
                    result["params"],
                    result["backup_file"] or "none",
                ),
            )
            return result
        except Exception as e:
            result["error"] = str(e)
            self._log_event("USB_INIT_FAILED", result["error"])
            return result
        finally:
            self._close()

    def _load_config(self) -> dict:
        if mavutil is None and not self.dry_run:
            raise RuntimeError("pymavlink not installed. Run: pip install pymavlink")
        if not os.path.isfile(self.config_file):
            raise IOError("init_config not found: %s" % self.config_file)
        if yaml is not None:
            with open(self.config_file, "r") as f:
                cfg = yaml.safe_load(f) or {}
        else:
            cfg = self._load_simple_yaml_mapping(self.config_file)
        if not isinstance(cfg, dict):
            raise ValueError("init_config must be a YAML mapping")
        return cfg

    @classmethod
    def _load_simple_yaml_mapping(cls, path: str) -> dict:
        """Small fallback parser for the simple mapping profile used here."""
        root = {}
        current_key = None
        with open(path, "r") as f:
            for raw_line in f:
                line = raw_line.split("#", 1)[0].rstrip()
                if not line.strip():
                    continue
                indent = len(line) - len(line.lstrip(" "))
                if ":" not in line:
                    continue
                key, value = line.split(":", 1)
                key = key.strip()
                value = value.strip()
                if indent == 0:
                    if value:
                        root[key] = cls._parse_scalar(value)
                        current_key = None
                    else:
                        root[key] = {}
                        current_key = key
                elif current_key:
                    root[current_key][key] = cls._parse_scalar(value)
        return root

    @staticmethod
    def _parse_scalar(value: str):
        lowered = value.lower()
        if lowered == "{}":
            return {}
        if lowered == "true":
            return True
        if lowered == "false":
            return False
        try:
            if "." in value:
                return float(value)
            return int(value)
        except ValueError:
            return value.strip("\"'")

    def _connect(self) -> bool:
        if self._mav is not None:
            return True
        try:
            url = self.connection_url.replace("serial:", "")
            if url.startswith("/dev/") or url.startswith("COM"):
                self._mav = mavutil.mavlink_connection(url, baud=self.baudrate)
            else:
                self._mav = mavutil.mavlink_connection(url)
            self._mav.wait_heartbeat(timeout=self.timeout)
            self._log_event(
                "USB_INIT_CONNECTED",
                "sys=%d comp=%d url=%s" % (
                    self._mav.target_system,
                    self._mav.target_component,
                    self.connection_url,
                ),
            )
            return True
        except Exception as e:
            self._mav = None
            self._log_event("USB_INIT_CONNECT_FAILED", str(e))
            return False

    def _close(self):
        if self._mav is not None:
            try:
                self._mav.close()
            except Exception:
                pass
            self._mav = None

    def _apply_message_intervals(self, intervals: dict) -> int:
        count = 0
        for name, rate in sorted((intervals or {}).items()):
            msg_id = self._message_id(name)
            interval_us = self._message_interval_to_us(rate)
            detail = "message=%s id=%d rate=%s interval_us=%d" % (
                name, msg_id, rate, interval_us,
            )
            if self.dry_run:
                self._log_event("USB_INIT_MSG_INTERVAL_DRYRUN", detail)
            else:
                self._mav.mav.command_long_send(
                    self._mav.target_system,
                    self._mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    interval_us,
                    0, 0, 0, 0, 0,
                )
                self._log_event("USB_INIT_MSG_INTERVAL", detail)
                time.sleep(0.02)
            count += 1
        return count

    def _apply_streams(self, streams: dict) -> int:
        count = 0
        for name, hz in sorted((streams or {}).items()):
            stream_id = self._stream_id(name)
            detail = "stream=%s id=%d hz=%s" % (name, stream_id, hz)
            if self.dry_run:
                self._log_event("USB_INIT_STREAM_DRYRUN", detail)
            else:
                self._mav.mav.request_data_stream_send(
                    self._mav.target_system,
                    self._mav.target_component,
                    stream_id,
                    int(hz),
                    1 if float(hz) > 0 else 0,
                )
                self._log_event("USB_INIT_STREAM", detail)
                time.sleep(0.02)
            count += 1
        return count

    def _backup_params(self, params: dict) -> str:
        os.makedirs(self._log_dir, exist_ok=True)
        path = os.path.join(
            self._log_dir,
            "link_init_backup_%s.yaml" % datetime.now().strftime("%Y%m%d_%H%M%S"),
        )
        backup = {
            "timestamp": _fmt_dt(datetime.now()),
            "connection_url": self.connection_url,
            "params": {},
        }

        if self.dry_run:
            backup["dry_run"] = True
            backup["params"] = {name: None for name in sorted(params)}
        else:
            for name in sorted(params):
                backup["params"][name] = self._read_param(name)

        if yaml is not None:
            with open(path, "w") as f:
                yaml.safe_dump(backup, f, default_flow_style=False, sort_keys=False)
        else:
            with open(path, "w") as f:
                f.write("timestamp: \"%s\"\n" % backup["timestamp"])
                f.write("connection_url: \"%s\"\n" % backup["connection_url"])
                if backup.get("dry_run"):
                    f.write("dry_run: true\n")
                f.write("params:\n")
                for name, value in sorted(backup["params"].items()):
                    f.write("  %s: %s\n" % (name, "null" if value is None else value))
        self._log_event("USB_INIT_PARAM_BACKUP", path)
        return path

    def _apply_params(self, params: dict) -> int:
        count = 0
        for name, value in sorted(params.items()):
            detail = "param=%s value=%s" % (name, value)
            if self.dry_run:
                self._log_event("USB_INIT_PARAM_DRYRUN", detail)
            else:
                ptype = (
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                    if isinstance(value, float)
                    else mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )
                self._mav.mav.param_set_send(
                    self._mav.target_system,
                    self._mav.target_component,
                    name.encode("utf-8")[:16],
                    float(value),
                    ptype,
                )
                self._log_event("USB_INIT_PARAM_SET", detail)
                time.sleep(0.05)
            count += 1
        return count

    def _read_param(self, name: str):
        self._mav.mav.param_request_read_send(
            self._mav.target_system,
            self._mav.target_component,
            name.encode("utf-8")[:16],
            -1,
        )
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            msg = self._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is None:
                continue
            msg_name = msg.param_id.strip("\x00 ")
            if msg_name == name:
                return msg.param_value
        return None

    def _message_id(self, name) -> int:
        if isinstance(name, int):
            return name
        text = str(name).strip()
        if text.isdigit():
            return int(text)
        key = text.upper()
        if key not in self.MESSAGE_IDS:
            raise ValueError("Unknown MAVLink message name: %s" % text)
        return self.MESSAGE_IDS[key]

    def _stream_id(self, name) -> int:
        if isinstance(name, int):
            return name
        text = str(name).strip()
        if text.isdigit():
            return int(text)
        key = text.upper()
        if key not in self.STREAM_IDS:
            raise ValueError("Unknown MAVLink stream name: %s" % text)
        return self.STREAM_IDS[key]

    @staticmethod
    def _message_interval_to_us(rate) -> int:
        if isinstance(rate, str):
            normalized = rate.strip().lower()
            if normalized in ("default", "restore", "restore_default", "qgc_default"):
                return 0
            if normalized in ("disable", "disabled", "off"):
                return -1
            rate = normalized

        hz_value = float(rate)
        if hz_value == 0:
            return 0
        if hz_value < 0:
            return -1
        return int(1000000.0 / hz_value)


# ---------------------------------------------------------------------------
# PX4LinkMonitor
# ---------------------------------------------------------------------------

class PX4LinkMonitor:
    def __init__(self):
        self._load_params()
        self._init_log_file()
        self._lock = threading.Lock()
        self._init_lock = threading.Lock()

        # State
        self._fcu_connected = None
        self._gcs_connected = None
        self._overflow_count = 0
        self._last_overflow_time = 0.0
        self._overflow_message_ids = set()
        self._session_start = datetime.now()
        self._usb_init_started = False
        self._usb_init_running = False
        self._usb_init_last_result = None

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
        rospy.Service("~init_usb_link", Trigger, self._handle_init_usb_link)
        rospy.Service("~init_status", Trigger, self._handle_init_status)

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
        self.enable_usb_init = rospy.get_param("~enable_usb_init", False)
        self.auto_init_on_fcu_connected = rospy.get_param(
            "~auto_init_on_fcu_connected", True
        )
        self.init_config = rospy.get_param("~init_config", "")
        self.connection_url = rospy.get_param("~connection_url", "udp:127.0.0.1:14550")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.timeout = rospy.get_param("~timeout", 10.0)
        self.dry_run = rospy.get_param("~dry_run", True)

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
                if conn:
                    self._maybe_schedule_usb_init("fcu_initial")
            elif self._fcu_connected != conn:
                self._fcu_connected = conn
                self._log_event(
                    "FCU_CONNECTION_CHANGE",
                    "connected=%s" % conn,
                )
                self._publish_status()
                if conn:
                    self._maybe_schedule_usb_init("fcu_reconnected")

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
        init_result = self._format_init_result(self._usb_init_last_result)
        lines = [
            "--- PX4 Link Monitor ---",
            "session: %s" % _fmt_dt(self._session_start),
            "elapsed: %.0f s" % (datetime.now() - self._session_start).total_seconds(),
            "fcu_connected: %s" % self._fcu_connected,
            "gcs_connected: %s" % self._gcs_connected,
            "overflow_count: %d" % self._overflow_count,
            "rate_window: %.1f s" % self.rate_window,
            "topic_rates: [%s]" % self._build_rate_info(),
            "usb_init_enabled: %s" % self.enable_usb_init,
            "usb_init_running: %s" % self._usb_init_running,
            "usb_init_result: %s" % init_result,
            "log_file: %s" % self._log_path,
        ]
        return "\n".join(lines)

    def _publish_status(self):
        text = self._build_status_text()
        self._status_pub.publish(String(data=text))

    def _handle_status_report(self, req):
        text = self._build_status_text()
        return TriggerResponse(success=True, message=text)

    # ---- USB link initialization ------------------------------------------

    def _handle_init_usb_link(self, req):
        result = self._run_usb_init("service")
        return TriggerResponse(
            success=result.get("success", False),
            message=self._format_init_result(result),
        )

    def _handle_init_status(self, req):
        return TriggerResponse(
            success=True,
            message=self._format_init_result(self._usb_init_last_result),
        )

    def _maybe_schedule_usb_init(self, reason: str):
        if not self.enable_usb_init or not self.auto_init_on_fcu_connected:
            return
        if self._usb_init_started:
            return
        self._usb_init_started = True
        timer = threading.Timer(2.0, self._run_usb_init, args=(reason,))
        timer.daemon = True
        timer.start()
        self._log_event("USB_INIT_SCHEDULED", "reason=%s delay=2.0s" % reason)

    def _run_usb_init(self, reason: str) -> dict:
        with self._init_lock:
            if self._usb_init_running:
                return {
                    "success": False,
                    "reason": reason,
                    "error": "USB link initialization already running",
                }
            self._usb_init_running = True
            self._publish_status()

            try:
                if not self.init_config:
                    result = {
                        "success": False,
                        "reason": reason,
                        "error": "No init_config configured",
                    }
                    self._log_event("USB_INIT_FAILED", result["error"])
                    return result

                initializer = USBLinkInitializer(
                    connection_url=self.connection_url,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    config_file=self.init_config,
                    dry_run=self.dry_run,
                    log_event=self._log_event,
                    log_dir=self.log_dir,
                )
                result = initializer.run(reason=reason)
                return result
            finally:
                if "result" in locals():
                    self._usb_init_last_result = result
                self._usb_init_running = False
                self._publish_status()

    @staticmethod
    def _format_init_result(result=None) -> str:
        if not result:
            return "never_run"
        if result.get("success"):
            return (
                "success reason=%s dry_run=%s intervals=%d streams=%d params=%d backup=%s"
                % (
                    result.get("reason", "unknown"),
                    result.get("dry_run", False),
                    result.get("message_intervals", 0),
                    result.get("streams", 0),
                    result.get("params", 0),
                    result.get("backup_file", "") or "none",
                )
            )
        return "failed reason=%s error=%s" % (
            result.get("reason", "unknown"),
            result.get("error", "unknown"),
        )


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    rospy.init_node("px4_link_monitor")
    PX4LinkMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()

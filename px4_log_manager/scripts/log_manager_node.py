#!/usr/bin/env python3
"""
PX4 Log Manager - ROS 1 Node

Provides a ROS service to:
  1. Download flight logs (.ulg) from PX4 over MAVLink
  2. Parse downloaded logs (extract key flight data)
  3. Erase old logs from the flight controller
"""

import os
import struct
import time
import threading
from datetime import datetime

import rospy
from std_srvs.srv import Trigger, TriggerResponse

try:
    from pymavlink import mavutil
except ImportError:
    rospy.logfatal("pymavlink not installed. Run: pip install pymavlink")
    raise

try:
    from pyulog import ULog
except ImportError:
    rospy.logwarn("pyulog not installed. Log parsing disabled. Run: pip install pyulog")
    ULog = None


class PX4LogManager:
    def __init__(self):
        self._load_params()

        self._mav = None
        self._lock = threading.Lock()
        self._download_dir = self._resolve_dir(self.download_dir)

        rospy.Service("~download_logs", Trigger, self._handle_download_logs)
        rospy.Service("~parse_last_log", Trigger, self._handle_parse_last_log)
        rospy.Service("~erase_logs", Trigger, self._handle_erase_logs)
        rospy.Service("~full_cycle", Trigger, self._handle_full_cycle)

        rospy.loginfo("PX4 Log Manager initialized")
        rospy.loginfo("  Connection: %s", self.connection_url)
        rospy.loginfo("  Download dir: %s", self._download_dir)

    # ------------------------------------------------------------------
    # ROS parameter loading
    # ------------------------------------------------------------------
    def _load_params(self):
        self.connection_url = rospy.get_param(
            "~connection_url", "udp:127.0.0.1:14550"
        )
        self.download_dir = rospy.get_param(
            "~download_dir", "~/.px4_logs"
        )
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.timeout = rospy.get_param("~timeout", 10.0)
        self.auto_erase = rospy.get_param("~auto_erase", True)

    @staticmethod
    def _resolve_dir(path):
        return os.path.abspath(os.path.expanduser(path))

    # ------------------------------------------------------------------
    # MAVLink connection
    # ------------------------------------------------------------------
    def _connect(self):
        if self._mav is not None:
            return True
        try:
            if self.connection_url.startswith("serial:") or self.connection_url.startswith("/dev/"):
                url = self.connection_url.replace("serial:", "")
                self._mav = mavutil.mavlink_connection(
                    url, baud=self.baudrate
                )
            else:
                self._mav = mavutil.mavlink_connection(self.connection_url)

            self._mav.wait_heartbeat(timeout=self.timeout)
            rospy.loginfo("Connected to PX4 (type=%d, autopilot=%d)",
                          self._mav.target_system, self._mav.target_component)
            return True
        except Exception as e:
            rospy.logerr("MAVLink connection failed: %s", e)
            self._mav = None
            return False

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------
    def _handle_download_logs(self, req):
        success, msg = self.download_logs()
        return TriggerResponse(success=success, message=msg)

    def _handle_parse_last_log(self, req):
        success, msg = self.parse_last_log()
        return TriggerResponse(success=success, message=msg)

    def _handle_erase_logs(self, req):
        success, msg = self.erase_logs()
        return TriggerResponse(success=success, message=msg)

    def _handle_full_cycle(self, req):
        """Download -> Parse -> Erase in one call."""
        with self._lock:
            ok, msg = self.download_logs()
            if not ok:
                return TriggerResponse(success=False, message="Download failed: " + msg)

            ok2, msg2 = self.parse_last_log()
            if not ok2:
                rospy.logwarn("Parse failed: %s", msg2)

            erase_msg = "skipped"
            if self.auto_erase:
                ok3, erase_msg = self.erase_logs()
                if not ok3:
                    rospy.logwarn("Erase failed: %s", erase_msg)

            return TriggerResponse(
                success=True,
                message="Download OK. Parse: %s. Erase: %s" % (msg2, erase_msg),
            )

    # ------------------------------------------------------------------
    # Core: Download logs
    # ------------------------------------------------------------------
    def download_logs(self):
        if not self._connect():
            return False, "Cannot connect to PX4"

        os.makedirs(self._download_dir, exist_ok=True)

        # Request log list
        self._mav.mav.log_request_list_send(
            self._mav.target_system, self._mav.target_component,
            0, 0xFFFF
        )

        entries = []
        timeout_time = time.time() + self.timeout
        while time.time() < timeout_time:
            msg = self._mav.recv_match(type="LOG_ENTRY", blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.num_logs == 0:
                rospy.loginfo("No logs on flight controller")
                return True, "No logs to download"
            entries.append(msg)
            rospy.loginfo("Log #%d: id=%d size=%d num=%d last=%d",
                          len(entries), msg.id, msg.size, msg.num_logs, msg.last_log_num)
            if msg.id >= msg.last_log_num:
                break

        if not entries:
            return False, "No LOG_ENTRY received (timeout)"

        rospy.loginfo("Found %d log(s) to download", len(entries))

        downloaded = []
        for entry in entries:
            filepath = self._download_one_log(entry)
            if filepath:
                downloaded.append(filepath)

        self._mav.close()
        self._mav = None

        if not downloaded:
            return False, "Failed to download any logs"
        return True, "Downloaded %d log(s) to %s" % (len(downloaded), self._download_dir)

    def _download_one_log(self, entry):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = "log_%d_%s.ulg" % (entry.id, timestamp)
        filepath = os.path.join(self._download_dir, filename)

        self._mav.mav.log_request_data_send(
            self._mav.target_system, self._mav.target_component,
            entry.id, 0, 0xFFFFFFFF
        )

        data = bytearray()
        expected_size = entry.size
        offset = 0

        timeout_time = time.time() + max(self.timeout * 3, expected_size * 0.0001 + 5)
        while len(data) < expected_size and time.time() < timeout_time:
            msg = self._mav.recv_match(type="LOG_DATA", blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.id != entry.id:
                continue

            chunk_offset = msg.ofs
            if chunk_offset == len(data):
                data.extend(msg.data[:msg.count])
            elif chunk_offset > len(data):
                data.extend(b"\x00" * (chunk_offset - len(data)))
                data.extend(msg.data[:msg.count])

            offset = len(data)

        if len(data) >= expected_size:
            with open(filepath, "wb") as f:
                f.write(data[:expected_size])
            rospy.loginfo("Downloaded: %s (%d bytes)", filepath, expected_size)
            return filepath
        else:
            rospy.logerr("Incomplete download: %d/%d bytes for log #%d",
                         len(data), expected_size, entry.id)
            return None

    # ------------------------------------------------------------------
    # Core: Parse last log
    # ------------------------------------------------------------------
    def parse_last_log(self):
        if not os.path.isdir(self._download_dir):
            return False, "Download directory not found: %s" % self._download_dir

        ulg_files = sorted(
            [f for f in os.listdir(self._download_dir) if f.endswith(".ulg")],
            key=lambda f: os.path.getmtime(os.path.join(self._download_dir, f)),
        )
        if not ulg_files:
            return False, "No .ulg files in %s" % self._download_dir

        last_file = os.path.join(self._download_dir, ulg_files[-1])
        return self._parse_ulg(last_file)

    def _parse_ulg(self, filepath):
        if ULog is None:
            return False, "pyulog not installed"

        try:
            ulog = ULog(filepath)
        except Exception as e:
            return False, "Failed to open ULog: %s" % e

        info = {
            "file": os.path.basename(filepath),
            "timestamp": ulog.start_timestamp,
            "duration_s": 0.0,
            "n_messages": len(ulog.data_list),
            "subscribed_topics": [],
        }

        # Extract vehicle attitude / GPS / battery if available
        for d in ulog.data_list:
            info["subscribed_topics"].append(d.name)
            if d.name == "vehicle_attitude" and len(d.data["timestamp"]) > 0:
                info["attitude_samples"] = len(d.data["timestamp"])
            elif d.name == "vehicle_gps_position" and len(d.data["timestamp"]) > 0:
                gps = d.data
                info["gps_samples"] = len(gps["timestamp"])
                if gps.get("lat") and len(gps["lat"]) > 0:
                    info["gps_lat"] = gps["lat"][-1] * 1e-7
                    info["gps_lon"] = gps["lon"][-1] * 1e-7
                    info["gps_alt"] = gps["alt"][-1] * 1e-3
            elif d.name == "battery_status" and len(d.data["timestamp"]) > 0:
                bat = d.data
                info["battery_samples"] = len(bat["timestamp"])
                if bat.get("voltage_v") and len(bat["voltage_v"]) > 0:
                    info["battery_voltage"] = bat["voltage_v"][-1]

        # Duration
        if "timestamp" in info:
            tm = info["timestamp"]
            if isinstance(tm, int):
                info["timestamp"] = datetime.utcfromtimestamp(tm / 1e6).isoformat()

        if "attitude_samples" in info:
            rospy.loginfo("Log parsed: %s", info)

        if hasattr(ulog, "get_dataset_info"):
            try:
                ds = ulog.get_dataset_info("vehicle_local_position")
                if ds and ds.data and len(ds.data["x"]) > 0:
                    # Flight distance estimation
                    import math
                    x, y = ds.data["x"], ds.data["y"]
                    dist = 0.0
                    for i in range(1, len(x)):
                        dist += math.sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2)
                    info["flight_distance_m"] = round(dist, 2)
                    info["duration_s"] = round(
                        (ds.data["timestamp"][-1] - ds.data["timestamp"][0]) / 1e6, 1
                    )
            except Exception:
                pass

        # Publish parsed summary on ROS topic
        self._publish_summary(info)

        return True, str(info)

    def _publish_summary(self, info):
        from std_msgs.msg import String
        pub = rospy.Publisher("~log_summary", String, queue_size=1, latch=True)
        pub.publish(String(data=str(info)))

    # ------------------------------------------------------------------
    # Core: Erase logs on FC
    # ------------------------------------------------------------------
    def erase_logs(self):
        if not self._connect():
            return False, "Cannot connect to PX4"

        self._mav.mav.log_erase_send(
            self._mav.target_system, self._mav.target_component
        )

        time.sleep(1.0)
        res = self._mav.recv_match(type="LOG_ERASE", blocking=True, timeout=3)

        self._mav.close()
        self._mav = None

        if res is not None:
            rospy.loginfo("Erase result: %s", res)
            return True, "Logs erased successfully"
        return True, "Erase command sent (no ack)"


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------
def main():
    rospy.init_node("px4_log_manager")
    PX4LogManager()
    rospy.spin()


if __name__ == "__main__":
    main()

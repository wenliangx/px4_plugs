#!/usr/bin/env python3
"""
PX4 Parameter Migrator - ROS 1 Node

Provides ROS services to:
  1. Export PX4 parameters to a YAML file (with filter)
  2. Import parameters from a YAML file to PX4 (with filter)
  3. Hot-reload the parameter filter file

The filter file (YAML) controls which params to export/import:
  - whitelist mode: only matching params are included
  - blacklist mode: all params EXCEPT matching ones are included
  - Patterns support shell-style wildcards: *, ?, [...]
"""

import fnmatch
import os
import time
import threading

import rospy
import yaml
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

try:
    from pymavlink import mavutil
except ImportError:
    rospy.logfatal("pymavlink not installed. Run: pip install pymavlink")
    raise


# ------------------------------------------------------------------
# Parameter value type mapping (MAVLink param_type -> python)
# ------------------------------------------------------------------
PARAM_TYPE_MAP = {
    1:  int,     # MAV_PARAM_TYPE_UINT8
    2:  int,     # MAV_PARAM_TYPE_INT8
    3:  int,     # MAV_PARAM_TYPE_UINT16
    4:  int,     # MAV_PARAM_TYPE_INT16
    5:  int,     # MAV_PARAM_TYPE_UINT32
    6:  int,     # MAV_PARAM_TYPE_INT32
    7:  float,   # MAV_PARAM_TYPE_REAL32 (float32 actual)
    8:  float,   # MAV_PARAM_TYPE_REAL64 (float64 actual)
}


class ParamFilter:
    """Loads filter rules from a YAML file and matches param names."""

    def __init__(self, filepath=None):
        self._filepath = None
        self._mode = "whitelist"
        self._patterns = []
        if filepath:
            self.load(filepath)

    @property
    def filepath(self):
        return self._filepath

    def load(self, filepath):
        path = os.path.abspath(os.path.expanduser(filepath))
        if not os.path.isfile(path):
            raise IOError("Filter file not found: %s" % path)

        with open(path, "r") as f:
            cfg = yaml.safe_load(f) or {}

        self._filepath = path
        self._mode = cfg.get("mode", "whitelist").lower()
        self._patterns = cfg.get("params", [])

        if self._mode not in ("whitelist", "blacklist"):
            rospy.logwarn("Unknown filter mode '%s', falling back to whitelist", self._mode)
            self._mode = "whitelist"

        rospy.loginfo("Filter loaded: mode=%s, %d pattern(s) from %s",
                      self._mode, len(self._patterns), path)

    def _match(self, name):
        for pat in self._patterns:
            if fnmatch.fnmatch(name, pat):
                return True
        return False

    def filter(self, params_dict):
        """Return a dict containing only params that pass the filter."""
        if not self._patterns:
            return dict(params_dict)

        result = {}
        for name, value in params_dict.items():
            matched = self._match(name)
            if (self._mode == "whitelist" and matched) or \
               (self._mode == "blacklist" and not matched):
                result[name] = value
        return result


# ------------------------------------------------------------------
# ROS Node
# ------------------------------------------------------------------
class PX4ParamMigrator:
    def __init__(self):
        self._load_params()
        self._mav = None
        self._lock = threading.Lock()
        self._filter = ParamFilter()

        # Try loading default filter if provided
        if self.filter_file:
            try:
                self._filter.load(self.filter_file)
            except IOError as e:
                rospy.logwarn("Default filter not loaded: %s", e)

        self._export_dir = self._resolve_dir(self.export_dir)

        rospy.Service("~export_params", Trigger, self._handle_export)
        rospy.Service("~import_params", Trigger, self._handle_import)
        rospy.Service("~reload_filter", Trigger, self._handle_reload_filter)
        rospy.Service("~compare_params", Trigger, self._handle_compare)

        rospy.loginfo("PX4 Param Migrator initialized")
        rospy.loginfo("  Connection: %s", self.connection_url)
        rospy.loginfo("  Filter: %s", self.filter_file or "(none)")
        rospy.loginfo("  Export dir: %s", self._export_dir)

    # ------------------------------------------------------------------
    # ROS parameters
    # ------------------------------------------------------------------
    def _load_params(self):
        self.connection_url = rospy.get_param("~connection_url", "udp:127.0.0.1:14550")
        self.export_dir = rospy.get_param("~export_dir", "~/.px4_params")
        self.filter_file = rospy.get_param("~filter_file", "")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.timeout = rospy.get_param("~timeout", 15.0)
        self.import_file = rospy.get_param("~import_file", "")
        self.reboot_after_import = rospy.get_param("~reboot_after_import", False)

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
            url = self.connection_url
            if url.startswith("serial:"):
                url = url.replace("serial:", "")
            if url.startswith("/dev/") or url.startswith("COM"):
                self._mav = mavutil.mavlink_connection(url, baud=self.baudrate)
            else:
                self._mav = mavutil.mavlink_connection(url)

            self._mav.wait_heartbeat(timeout=self.timeout)
            rospy.loginfo("Connected to PX4 (sys=%d, comp=%d)",
                          self._mav.target_system, self._mav.target_component)
            return True
        except Exception as e:
            rospy.logerr("MAVLink connection failed: %s", e)
            self._mav = None
            return False

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------
    def _handle_export(self, req):
        success, msg = self.export_params()
        return TriggerResponse(success=success, message=msg)

    def _handle_import(self, req):
        success, msg = self.import_params()
        return TriggerResponse(success=success, message=msg)

    def _handle_reload_filter(self, req):
        if not self.filter_file:
            return TriggerResponse(success=False, message="No filter_file configured")
        try:
            self._filter.load(self.filter_file)
            return TriggerResponse(success=True, message="Filter reloaded: %d pattern(s)" % len(self._filter._patterns))
        except Exception as e:
            return TriggerResponse(success=False, message="Reload failed: %s" % e)

    def _handle_compare(self, req):
        success, msg = self.compare_params()
        return TriggerResponse(success=success, message=msg)

    # ------------------------------------------------------------------
    # Core: Read all params from FC
    # ------------------------------------------------------------------
    def _read_all_params(self):
        """Read all parameters from FC, return dict {name: value}."""
        if not self._connect():
            return None

        self._mav.mav.param_request_list_send(
            self._mav.target_system, self._mav.target_component
        )

        params = {}
        count = 0
        timeout_time = time.time() + self.timeout
        last_msg_time = time.time()

        while time.time() < timeout_time:
            msg = self._mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is None:
                if time.time() - last_msg_time > 3 and count > 0:
                    break
                continue

            last_msg_time = time.time()
            name = msg.param_id.strip("\x00 ")
            if name:
                ptype = PARAM_TYPE_MAP.get(msg.param_type, float)
                params[name] = ptype(msg.param_value)
                count += 1

            if msg.param_index + 1 >= msg.param_count:
                break

        rospy.loginfo("Read %d parameters from FC", len(params))
        return params

    # ------------------------------------------------------------------
    # Core: Export params
    # ------------------------------------------------------------------
    def export_params(self, export_file=None):
        """Export FC params to a YAML file, filtered."""
        with self._lock:
            params = self._read_all_params()
            if params is None:
                return False, "Failed to read params from FC"

            self._mav.close()
            self._mav = None

            filtered = self._filter.filter(params)
            skipped = len(params) - len(filtered)
            rospy.loginfo("Export: %d/%d params after filter (%s mode)",
                          len(filtered), len(params), self._filter._mode)

            if not filtered:
                return False, "No params matched filter (mode=%s, %d patterns)" % (
                    self._filter._mode, len(self._filter._patterns))

            os.makedirs(self._export_dir, exist_ok=True)

            if export_file:
                filepath = export_file
            else:
                ts = time.strftime("%Y%m%d_%H%M%S")
                filepath = os.path.join(self._export_dir, "px4_params_%s.yaml" % ts)

            data = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "mode": self._filter._mode,
                "total_exported": len(filtered),
                "total_on_fc": len(params),
                "params": {name: round(value, 8) if isinstance(value, float) else int(value)
                           for name, value in sorted(filtered.items())},
            }

            with open(filepath, "w") as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

            rospy.loginfo("Exported to: %s", filepath)
            self._publish_status("exported", filepath, len(filtered))
            return True, "Exported %d params to %s (skipped %d)" % (len(filtered), filepath, skipped)

    # ------------------------------------------------------------------
    # Core: Import params
    # ------------------------------------------------------------------
    def import_params(self, import_file=None):
        """Read params from YAML file and push to FC, filtered."""
        with self._lock:
            filepath = import_file or self.import_file
            if not filepath:
                # Auto-find latest export
                files = sorted(
                    [f for f in os.listdir(self._export_dir) if f.startswith("px4_params_") and f.endswith(".yaml")],
                    reverse=True,
                )
                if not files:
                    return False, "No param file found in %s" % self._export_dir
                filepath = os.path.join(self._export_dir, files[0])

            filepath = os.path.abspath(os.path.expanduser(filepath))
            if not os.path.isfile(filepath):
                return False, "File not found: %s" % filepath

            with open(filepath, "r") as f:
                data = yaml.safe_load(f)

            source_params = data.get("params", data)  # support flat dict too
            if not isinstance(source_params, dict):
                return False, "Invalid param file format"

            filtered = self._filter.filter(source_params)
            skipped = len(source_params) - len(filtered)
            rospy.loginfo("Import: %d/%d params after filter (%s mode)",
                          len(filtered), len(source_params), self._filter._mode)

            if not filtered:
                return False, "No params to import after filtering"

            if not self._connect():
                return False, "Cannot connect to PX4"

            success_count = 0
            fail_list = []

            for i, (name, value) in enumerate(sorted(filtered.items())):
                ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32 if isinstance(value, float) else mavutil.mavlink.MAV_PARAM_TYPE_INT32
                self._mav.mav.param_set_send(
                    self._mav.target_system, self._mav.target_component,
                    name.encode("utf-8")[:16], float(value), ptype
                )
                success_count += 1

                if (i + 1) % 5 == 0:
                    rospy.loginfo("  Import progress: %d/%d", i + 1, len(filtered))
                time.sleep(0.01)

            self._mav.close()
            self._mav = None

            if self.reboot_after_import:
                rospy.loginfo("Rebooting FC to persist params ...")
                self._reboot_fc()

            rospy.loginfo("Imported %d params (skipped %d, failed %d)",
                          success_count, skipped, len(fail_list))
            self._publish_status("imported", filepath, success_count)
            return True, "Imported %d params from %s (skipped %d)" % (success_count, filepath, skipped)

    # ------------------------------------------------------------------
    # Core: Compare
    # ------------------------------------------------------------------
    def compare_params(self, import_file=None):
        """Compare FC params against a local file — show differences."""
        with self._lock:
            fc_params = self._read_all_params()
            if fc_params is None:
                return False, "Failed to read params from FC"

            self._mav.close()
            self._mav = None

            filepath = import_file or self.import_file
            if not filepath:
                files = sorted(
                    [f for f in os.listdir(self._export_dir) if f.startswith("px4_params_") and f.endswith(".yaml")],
                    reverse=True,
                )
                if not files:
                    return False, "No param file found for comparison"
                filepath = os.path.join(self._export_dir, files[0])

            filepath = os.path.abspath(os.path.expanduser(filepath))
            if not os.path.isfile(filepath):
                return False, "File not found: %s" % filepath

            with open(filepath, "r") as f:
                data = yaml.safe_load(f)
            file_params = data.get("params", data)

            fc_filtered = self._filter.filter(fc_params)
            file_filtered = self._filter.filter(file_params)

            all_names = sorted(set(list(fc_filtered.keys()) + list(file_filtered.keys())))

            added, removed, changed = [], [], []
            for name in all_names:
                in_fc = name in fc_filtered
                in_file = name in file_filtered
                if in_fc and not in_file:
                    removed.append((name, fc_filtered[name]))
                elif not in_fc and in_file:
                    added.append((name, file_filtered[name]))
                elif in_fc and in_file:
                    if abs(fc_filtered[name] - file_filtered[name]) > 1e-8:
                        changed.append((name, fc_filtered[name], file_filtered[name]))

            summary = "Compared against %s:\n" % os.path.basename(filepath)
            summary += "  Added:   %d\n" % len(added)
            summary += "  Removed: %d\n" % len(removed)
            summary += "  Changed: %d\n" % len(changed)

            for name, old, new in changed[:20]:
                summary += "    %s: FC=%s  File=%s\n" % (name, old, new)

            rospy.loginfo(summary)
            self._publish_status("compared", filepath, len(changed))
            return True, summary.strip()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _reboot_fc(self):
        try:
            if self._connect():
                self._mav.mav.command_long_send(
                    self._mav.target_system, self._mav.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                    0, 1, 0, 0, 0, 0, 0, 0
                )
                self._mav.close()
                self._mav = None
        except Exception as e:
            rospy.logwarn("Reboot command failed: %s", e)

    def _publish_status(self, action, filepath, count):
        pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        pub.publish(String(data="%s: %d params, file=%s" % (action, count, filepath)))


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------
def main():
    rospy.init_node("px4_param_migrator")
    PX4ParamMigrator()
    rospy.spin()


if __name__ == "__main__":
    main()

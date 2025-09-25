#!/usr/bin/env python3
"""
ROS2 node: file_state_publisher_stamped

Reads a file that can be modified by another process and publishes:
  - /velocity (geometry_msgs/msg/TwistStamped)     # linear.x = velocity (m/s)
  - /enabled  (my_msgs/msg/BoolStamped)            # stamped bool

Parameters:
  - file_path (string, default: "state.txt")
  - velocity_topic (string, default: "/velocity")
  - enabled_topic (string, default: "/enabled")
  - poll_hz (double, default: 10.0)
  - frame_id (string, default: "base_link")        # for TwistStamped.header.frame_id
  - status_frame_id (string, default: "status")    # for BoolStamped.header.frame_id

Accepted file formats (same as before):
  JSON: {"velocity": 1.25, "enabled": 1}
  Two lines: <velocity>\n<enabled>
  INI: velocity=1.25, enabled=1
  CSV/whitespace: 1.25, 1  OR  1.25 1
"""

import json
import os
import re
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
# NOTE: You'll create this package/message. See instructions below.
from owl_msgs.msg import BoolStamped


def _to_bool(token: str) -> Optional[bool]:
    t = token.strip().lower()
    if t in ("1", "true", "yes", "y", "on"):
        return True
    if t in ("0", "false", "no", "n", "off"):
        return False
    return None


def parse_state_text(text: str) -> Optional[Tuple[float, bool]]:
    s = text.strip()
    if not s:
        return None
    # Try JSON
    try:
        data = json.loads(s)
        if isinstance(data, dict):
            v = float(data["velocity"])
            b_raw = data["enabled"]
            b = bool(b_raw) if isinstance(b_raw, bool) else _to_bool(str(b_raw))
            if b is None:
                return None
            return v, b
    except Exception:
        pass

    # Strip comments and collect non-empty lines
    lines = []
    for line in s.splitlines():
        line = re.split(r"(#|//|;)", line)[0].strip()
        if line:
            lines.append(line)

    # INI-like
    kv = {}
    for line in lines:
        if "=" in line:
            k, v = line.split("=", 1)
            kv[k.strip().lower()] = v.strip()
        elif ":" in line:
            k, v = line.split(":", 1)
            kv[k.strip().lower()] = v.strip()
    if "velocity" in kv and "enabled" in kv:
        try:
            v = float(kv["velocity"])
            b = _to_bool(kv["enabled"])
            if b is not None:
                return v, b
        except Exception:
            pass

    # Two-line
    if len(lines) >= 2:
        try:
            v = float(lines[0])
            b = _to_bool(lines[1])
            if b is not None:
                return v, b
        except Exception:
            pass

    # CSV/whitespace single line
    tokens = re.split(r"[,\s]+", s)
    if len(tokens) >= 2:
        try:
            v = float(tokens[0])
            b = _to_bool(tokens[1])
            if b is not None:
                return v, b
        except Exception:
            pass

    return None


class FileStatePublisherStamped(Node):
    def __init__(self):
        super().__init__("file_state_publisher_stamped")

        self.declare_parameter("file_path", "state.txt")
        self.declare_parameter("velocity_topic", "/velocity")
        self.declare_parameter("enabled_topic", "/enabled")
        self.declare_parameter("poll_hz", 10.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("status_frame_id", "status")

        self.file_path = self.get_parameter("file_path").get_parameter_value().string_value
        self.velocity_topic = self.get_parameter("velocity_topic").get_parameter_value().string_value
        self.enabled_topic = self.get_parameter("enabled_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.status_frame_id = self.get_parameter("status_frame_id").get_parameter_value().string_value

        poll_hz = float(self.get_parameter("poll_hz").get_parameter_value().double_value)
        if poll_hz <= 0.0:
            self.get_logger().warn("poll_hz must be > 0. Falling back to 10.0")
            poll_hz = 10.0

        self.vel_pub = self.create_publisher(TwistStamped, self.velocity_topic, 10)
        self.enabled_pub = self.create_publisher(BoolStamped, self.enabled_topic, 10)

        self.last_mtime: Optional[float] = None
        self.last_values: Optional[Tuple[float, bool]] = None

        self.get_logger().info(
            f"Watching file: {self.file_path}\n"
            f"Publishing velocity -> {self.velocity_topic} (TwistStamped)\n"
            f"Publishing enabled  -> {self.enabled_topic} (BoolStamped)\n"
            f"Polling at {poll_hz:.1f} Hz"
        )
        self.timer = self.create_timer(1.0 / poll_hz, self._poll_once)

    def _poll_once(self):
        try:
            mtime = os.path.getmtime(self.file_path)
        except FileNotFoundError:
            if self.last_mtime is not None:
                self.get_logger().warn(f"File disappeared: {self.file_path}")
            self.last_mtime = None
            return
        except Exception as e:
            self.get_logger().warn(f"Stat failed for {self.file_path}: {e}")
            return

        if self.last_mtime is None or mtime != self.last_mtime:
            try:
                with open(self.file_path, "r", encoding="utf-8") as f:
                    content = f.read()
            except Exception as e:
                self.get_logger().warn(f"Failed to read {self.file_path}: {e}")
                return

            parsed = parse_state_text(content)
            if parsed is None:
                self.get_logger().warn(
                    "Could not parse file. Expected formats like:\n"
                    '  JSON: {"velocity": 1.2, "enabled": 1}\n'
                    "  Two lines: <velocity>\\n<enabled>\n"
                    "  INI: velocity=1.2, enabled=1\n"
                    "  CSV: 1.2, 1"
                )
                return

            velocity, enabled = parsed
            self.last_mtime = mtime

            if self.last_values != (velocity, enabled):
                t = self.get_clock().now().to_msg()

                twist_msg = TwistStamped()
                twist_msg.header.stamp = t
                twist_msg.header.frame_id = self.frame_id
                twist_msg.twist.linear.x = float(velocity)
                twist_msg.twist.linear.y = 0.0
                twist_msg.twist.linear.z = 0.0
                twist_msg.twist.angular.x = 0.0
                twist_msg.twist.angular.y = 0.0
                twist_msg.twist.angular.z = 0.0

                en_msg = BoolStamped()
                en_msg.header.stamp = t
                en_msg.header.frame_id = self.status_frame_id
                en_msg.data = bool(enabled)

                self.vel_pub.publish(twist_msg)
                self.enabled_pub.publish(en_msg)
                self.last_values = (velocity, enabled)

                self.get_logger().info(
                    f"Published TwistStamped vx={velocity:.6g} m/s, BoolStamped={enabled}"
                )


def main():
    rclpy.init()
    node = FileStatePublisherStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


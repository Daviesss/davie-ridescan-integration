#!/usr/bin/env python3
"""
ridescan_bridge_node.py

ROS 2 -> RideScan bridge using the ridescanapi SDK.

Collects telemetry from /odom, /scan, and /cmd_vel, batches it into CSV
files, and uploads them to a RideScan robot mission for inference.

Subscribes:
  /odom     (nav_msgs/Odometry)
  /scan     (sensor_msgs/LaserScan)
  /cmd_vel  (geometry_msgs/Twist)

Author: Davies Iyanuoluwa Ogunsina
"""

import csv
import json
import math
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# CONFIG — update before going live
API_KEY      = "rsk_your_api_key_here"
ROBOT_NAME   = "my_bot"
ROBOT_TYPE   = "SPOT"
MISSION_NAME = f"ros2-mission-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
DRY_RUN      = True    # set to False once API key is received

UPLOAD_INTERVAL_SEC = 60.0
MIN_ROWS_PER_UPLOAD = 50
OUTPUT_DIR          = Path(tempfile.gettempdir())
FILE_TYPE           = "process_file"

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def now_iso():
    return datetime.now(tz=timezone.utc).isoformat()


def stamp_to_iso(stamp):
    seconds = stamp.sec + stamp.nanosec * 1e-9
    return datetime.fromtimestamp(seconds, tz=timezone.utc).isoformat()


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RideScanBridgeNode(Node):

    def __init__(self):
        super().__init__("ridescan_bridge_node")

        self.rows       = []
        self.robot_id   = None
        self.mission_id = None

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        self.create_subscription(Odometry, "/odom", self.odom_callback, SENSOR_QOS)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, SENSOR_QOS)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, RELIABLE_QOS)
        self.create_timer(UPLOAD_INTERVAL_SEC, self.flush)

        mode = "[DRY RUN]" if DRY_RUN else "[LIVE]"
        self.get_logger().info(
            f"RideScan bridge started {mode} | output={OUTPUT_DIR}"
        )

    def odom_callback(self, msg):
        pose  = msg.pose.pose
        twist = msg.twist.twist
        self.add_row(
            "odom",
            stamp_to_iso(msg.header.stamp),
            {
                "x":         pose.position.x,
                "y":         pose.position.y,
                "z":         pose.position.z,
                "yaw":       yaw_from_quaternion(pose.orientation),
                "linear_x":  twist.linear.x,
                "linear_y":  twist.linear.y,
                "angular_z": twist.angular.z,
            },
        )

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.add_row(
            "scan",
            stamp_to_iso(msg.header.stamp),
            {
                "range_min":            msg.range_min,
                "range_max":            msg.range_max,
                "beam_count":           len(msg.ranges),
                "valid_beam_count":     len(valid),
                "min_obstacle_dist":    min(valid) if valid else None,
                "mean_obstacle_dist":   sum(valid) / len(valid) if valid else None,
            },
        )

    def cmd_vel_callback(self, msg):
        self.add_row(
            "cmd_vel",
            now_iso(),
            {
                "linear_x":  msg.linear.x,
                "linear_y":  msg.linear.y,
                "angular_z": msg.angular.z,
            },
        )

    def add_row(self, event_type, timestamp, data):
        self.rows.append(
            {
                "timestamp":  timestamp,
                "event_type": event_type,
                "data":       json.dumps(data, separators=(",", ":")),
            }
        )

    def flush(self):
        if len(self.rows) < MIN_ROWS_PER_UPLOAD:
            return

        rows_to_upload = self.rows
        self.rows = []

        csv_path   = self.write_csv(rows_to_upload)
        event_time = rows_to_upload[0]["timestamp"]

        if DRY_RUN:
            self.get_logger().info(
                f"[DRY RUN] Wrote {len(rows_to_upload)} rows to {csv_path}; not uploading."
            )
            return

        self.upload_file(csv_path, event_time)

    def write_csv(self, rows):
        filename = f"ridescan_ros2_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.csv"
        csv_path = OUTPUT_DIR / filename

        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=["timestamp", "event_type", "data"])
            writer.writeheader()
            writer.writerows(rows)

        return csv_path

    def upload_file(self, csv_path, event_time):
        try:
            from ridescanapi import RideScanClient
            from ridescanapi.exceptions import RideScanError
        except ImportError:
            self.get_logger().error(
                "ridescanapi is not installed. Run: pip install ridescanapi"
            )
            return

        try:
            with RideScanClient(api_key=API_KEY) as client:

                if not self.robot_id:
                    robot = client.create_robot(name=ROBOT_NAME, robot_type=ROBOT_TYPE)
                    self.robot_id = robot["robot_id"]
                    self.get_logger().info(f"Created RideScan robot: {self.robot_id}")

                if not self.mission_id:
                    mission = client.create_mission(
                        robot_id=self.robot_id,
                        mission_name=MISSION_NAME,
                    )
                    self.mission_id = mission["mission_id"]
                    self.get_logger().info(f"Created RideScan mission: {self.mission_id}")

                result = client.upload_files(
                    robot_id=self.robot_id,
                    mission_id=self.mission_id,
                    file_data={str(csv_path): event_time},
                    file_type=FILE_TYPE,
                )

            self.get_logger().info(f"Uploaded {csv_path.name}: {result}")

        except RideScanError as exc:
            self.get_logger().error(f"RideScan SDK error: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Upload failed: {exc}")

    def destroy_node(self):
        if self.rows:
            self.get_logger().info("Flushing remaining rows before shutdown.")
            rows_to_upload = self.rows
            self.rows = []
            csv_path = self.write_csv(rows_to_upload)
            if DRY_RUN:
                self.get_logger().info(f"[DRY RUN] Final CSV written: {csv_path}")
            else:
                self.upload_file(csv_path, rows_to_upload[0]["timestamp"])
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RideScanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
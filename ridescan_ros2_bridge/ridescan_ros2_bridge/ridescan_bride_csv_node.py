#!/usr/bin/env python3
"""
ride_scan_csv_node.py

ROS 2 -> RideScan bridge producing WHEELED_MOBILE-compliant CSV output,
per RideScan Hackathon Technical Documentation (Section: WHEELED_MOBILE Robot).

Required columns (per spec):
    pose_position_x, pose_position_y, pose_position_z
    pose_orientation_roll, pose_orientation_pitch, pose_orientation_yaw
    twist_linear_x, twist_linear_y, twist_linear_z
    twist_angular_x, twist_angular_y, twist_angular_z
    timestamp (float64, required)

Optional columns included as 0.0 (since empty cells are not allowed):
    linear_acceleration_x, linear_acceleration_y, linear_acceleration_z

Author: Davies Iyanuoluwa Ogunsina
"""

import csv
import math
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


OUTPUT_DIR = Path(tempfile.gettempdir())
WRITE_HZ = 10.0  # consistent sampling rate per spec ("Ensure consistent sensor sampling rates")

FIELD_NAMES = [
    "timestamp",
    "pose_position_x",
    "pose_position_y",
    "pose_position_z",
    "pose_orientation_roll",
    "pose_orientation_pitch",
    "pose_orientation_yaw",
    "twist_linear_x",
    "twist_linear_y",
    "twist_linear_z",
    "twist_angular_x",
    "twist_angular_y",
    "twist_angular_z",
    "linear_acceleration_x",
    "linear_acceleration_y",
    "linear_acceleration_z",
]


def quaternion_to_euler(q):
    """Convert geometry_msgs/Quaternion to roll, pitch, yaw (radians)."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for asin domain safety
    pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RideScanBridgeNode(Node):

    def __init__(self):
        super().__init__("ridescan_bridge_node")

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        self.latest_odom = None
        self.rows = []

        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_timer(1.0 / WRITE_HZ, self.write_row)

        self.get_logger().info(
            "RideScan bridge started (WHEELED_MOBILE-compliant flat format)"
        )

    def odom_cb(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        roll, pitch, yaw = quaternion_to_euler(pose.orientation)

        self.latest_odom = {
            "pose_position_x": pose.position.x,
            "pose_position_y": pose.position.y,
            "pose_position_z": pose.position.z,
            "pose_orientation_roll": roll,
            "pose_orientation_pitch": pitch,
            "pose_orientation_yaw": yaw,
            "twist_linear_x": twist.linear.x,
            "twist_linear_y": twist.linear.y,
            "twist_linear_z": twist.linear.z,
            "twist_angular_x": twist.angular.x,
            "twist_angular_y": twist.angular.y,
            "twist_angular_z": twist.angular.z,
        }

    def write_row(self):
        # No NaNs/None/empty cells allowed  default to zeroed pose at origin
        # until the first /odom message arrives.
        odom = self.latest_odom or {
            "pose_position_x": 0.0,
            "pose_position_y": 0.0,
            "pose_position_z": 0.0,
            "pose_orientation_roll": 0.0,
            "pose_orientation_pitch": 0.0,
            "pose_orientation_yaw": 0.0,
            "twist_linear_x": 0.0,
            "twist_linear_y": 0.0,
            "twist_linear_z": 0.0,
            "twist_angular_x": 0.0,
            "twist_angular_y": 0.0,
            "twist_angular_z": 0.0,
        }

        row = {
            # float64 epoch seconds required as a numeric, not ISO 8601 string
            "timestamp": time.time(),
            **odom,
            # Optional fields  included as 0.0 since empty cells fail validation
            "linear_acceleration_x": 0.0,
            "linear_acceleration_y": 0.0,
            "linear_acceleration_z": 0.0,
        }

        self.rows.append(row)

    def save_csv(self):
        filename = f"ridescan_wheeled_mobile_{int(time.time())}.csv"
        path = OUTPUT_DIR / filename

        with path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=FIELD_NAMES)
            writer.writeheader()
            writer.writerows(self.rows)

        self.get_logger().info(f"Saved CSV: {path} ({len(self.rows)} rows)")
        return path

    def destroy_node(self):
        if self.rows:
            self.save_csv()
        return super().destroy_node()


def main():
    rclpy.init()
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
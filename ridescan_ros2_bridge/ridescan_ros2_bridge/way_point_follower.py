#!/usr/bin/env python3
"""

Sends Davie through a fixed sequence of waypoints using Nav2's
NavigateToPose action, then returns to the start position.

Each full run of this script = one mission instance.
Run this script 15 times to collect the calibration dataset.

Usage:
    python3 way_point_follower.py
    
    Author: Davies Iyanuoluwa Ogunsina
"""

import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


# Format: (x, y, yaw_degrees)
WAYPOINTS = [
    ( 1.0,  0.0,   0.0),   # waypoint 1 — dock exit
    ( 1.0,  2.5,  90.0),   # waypoint 2 — corner A
    (-1.0,  2.5, 180.0),   # waypoint 3 — corner B
    (-1.0,  0.0, 270.0),   # waypoint 4 — corner C
    ( 0.0,  0.0,   0.0),   # waypoint 5 — return to dock
]

MISSION_NAME = "warehouse_perimeter_inspection"
NAV_TIMEOUT  = 60.0   # seconds to wait per waypoint before timing out


def yaw_to_quaternion(yaw_deg: float):
    """Convert yaw in degrees to a quaternion (z, w only for 2D)."""
    import math
    yaw_rad = math.radians(yaw_deg)
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw_rad / 2.0),
        "w": math.cos(yaw_rad / 2.0),
    }


# implementation class.....
class MissionRunner(Node):

    def __init__(self):
        super().__init__("mission_runner")
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._waypoint_index = 0
        self._mission_success = False

    def run(self):
        self.get_logger().info(
            f"Mission [{MISSION_NAME}] starting | {len(WAYPOINTS)} waypoints"
        )

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available. Is Nav2 running?")
            return False

        for i, (x, y, yaw) in enumerate(WAYPOINTS):
            self.get_logger().info(
                f"Navigating to waypoint {i + 1}/{len(WAYPOINTS)} "
                f"x={x:.2f}, y={y:.2f}, yaw={yaw:.1f}deg"
            )

            success = self._send_goal(x, y, yaw)

            if not success:
                self.get_logger().error(
                    f"Failed to reach waypoint {i + 1} — aborting mission."
                )
                return False

            self.get_logger().info(f"Waypoint {i + 1} reached.")
            time.sleep(0.5)   # brief pause between waypoints

        self.get_logger().info(
            f"Mission [{MISSION_NAME}] complete. All {len(WAYPOINTS)} waypoints reached."
        )
        return True
   
    # Helper method to send a NavigateToPose goal and wait for the result
    def _send_goal(self, x: float, y: float, yaw_deg: float) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        
        # publish robot goal pose
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
       
       # Convert yaw to quaternion for orientation
        q = yaw_to_quaternion(yaw_deg)
        goal_msg.pose.pose.orientation.x = q["x"]
        goal_msg.pose.pose.orientation.y = q["y"]
        goal_msg.pose.pose.orientation.z = q["z"]
        goal_msg.pose.pose.orientation.w = q["w"]

        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=NAV_TIMEOUT
        )

        if not result_future.done():
            self.get_logger().error(
                f"Waypoint timed out after {NAV_TIMEOUT}s."
            )
            return False

        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            return True

        self.get_logger().error(f"Nav2 goal failed with status={status}")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()

    try:
        success = node.run()
        if success:
            node.get_logger().info("Mission runner finished successfully.")
        else:
            node.get_logger().error("Mission runner finished with errors.")
    except KeyboardInterrupt:
        node.get_logger().info("Mission runner interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
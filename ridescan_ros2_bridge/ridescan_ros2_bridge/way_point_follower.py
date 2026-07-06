"""
way_point_follower.py
 
Drives the robot through a fixed square path of 4 waypoints using a direct
/cmd_vel P-controller (rotate to face the target, then move forward once
aligned). Subscribes to /ridescan/safety_stop and immediately halts/resumes
in response to signals from ridescan_safety_monitor_node.py.
 
Author: Davies Iyanuoluwa Ogunsina
Maintainer: Davies Iyanuoluwa Ogunsina
""" 

#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

WAYPOINTS = [
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),
]

class SquareController(Node):

    def __init__(self):
        super().__init__("square_controller")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        # Listen for a safety-stop signal from the RideScan monitor node.
        self.paused = False
        self.create_subscription(Bool, "/ridescan/safety_stop", self.safety_stop_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.index = 0
        self.timer = self.create_timer(0.1, self.control_loop)

    def safety_stop_cb(self, msg: Bool):
        if msg.data and not self.paused:
            self.get_logger().warn("SAFETY STOP received from RideScan monitor. Halting mission.")
        elif not msg.data and self.paused:
            self.get_logger().info("Safety stop cleared. Resuming mission.")
        self.paused = msg.data

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def control_loop(self):

        # If paused by the safety monitor, publish zero velocity and skip
        # all waypoint logic  don't let it be overwritten next cycle.
        if self.paused:
            self.cmd_pub.publish(Twist())
            return

        if self.index >= len(WAYPOINTS):
            self.get_logger().info("Mission complete.")
            self.stop()
            return

        goal_x, goal_y = WAYPOINTS[self.index]

        dx = goal_x - self.x
        dy = goal_y - self.y

        dist = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)

        angle_error = self.normalize(target_angle - self.yaw)

        cmd = Twist()

        if abs(angle_error) > 0.3:
            cmd.angular.z = 1.5 * angle_error
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = 0.4
            cmd.angular.z = 0.0

        if dist < 0.2:
            self.get_logger().info(f"Reached waypoint {self.index}")
            self.index += 1

        self.cmd_pub.publish(cmd)

    def normalize(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop(self):
        self.cmd_pub.publish(Twist())

def main():
    rclpy.init()
    node = SquareController()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
RideScan safety node.

This node is the autonomous safety response layer for RideScan.
It subscribes to a risk score produced by the RideScan diagnostics system.
When risk exceeds the configured stop threshold, it cancels active Nav2
navigation goals and publishes zero velocity to stop the robot.

RideScan demonstrates risk-aware autonomous navigation by connecting
robot telemetry, anomaly/risk detection, and real-time safety intervention.
"""


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo


# CONFIG
RISK_THRESHOLD_WARN = 0.3
RISK_THRESHOLD_STOP = 0.7
ROBOT_ID = 'my_bot'


RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class RideScanSafetyNode(Node):

    def __init__(self):
        super().__init__('ridescan_safety')

        self._robot_stopped = False
        self._current_risk = 0.0
        self._stop_count = 0

        self.create_subscription(
            Float32,
            '/ridescan/risk_score',
            self._risk_score_cb,
            RELIABLE_QOS,
        )

        self.create_subscription(
            DiagnosticArray,
            '/ridescan/diagnostics',
            self._diagnostics_cb,
            RELIABLE_QOS,
        )

        self._nav_cancel_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
        )

        self._cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            RELIABLE_QOS,
        )

        self.get_logger().info(
            f'RideScan safety node online | robot_id={ROBOT_ID} | '
            f'stop_threshold={RISK_THRESHOLD_STOP}'
        )

    def _risk_score_cb(self, msg: Float32):
        risk = msg.data
        self._current_risk = risk

        if risk >= RISK_THRESHOLD_STOP:
            self._stop_robot(risk)
            return

        if risk >= RISK_THRESHOLD_WARN:
            self.get_logger().warn(
                f'[RIDESCAN] Elevated risk={risk:.2f} - monitoring closely'
            )
            return

        if self._robot_stopped:
            self._resume_robot(risk)
        else:
            self.get_logger().debug(
                f'[RIDESCAN] risk={risk:.2f} - nominal'
            )


    def _diagnostics_cb(self, msg: DiagnosticArray):
        """Log anomaly details from RideScan diagnostics."""
        for status in msg.status:
            if status.level == DiagnosticStatus.ERROR:
                self.get_logger().error(f'[RIDESCAN] {status.message}')
                for kv in status.values:
                    if kv.key != 'risk_score':
                        self.get_logger().error(
                            f'[RIDESCAN] Anomaly: {kv.key} = {kv.value}'
                        )

            elif status.level == DiagnosticStatus.WARN:
                self.get_logger().warn(f'[RIDESCAN] {status.message}')


    # This cancels the navigation goals and robot stops(repeatedly) when the risk score is above the stop threshold. Uncommenting this function will enable the stop behavior, but for demonstration purposes we are keeping it commented to allow the robot to continue navigating even when the risk score is high.
    # def _stop_robot(self, risk: float):
    #     """Cancel Nav2 goals and publish zero velocity."""
    #     if not self._robot_stopped:
    #         self._robot_stopped = True
    #         self._stop_count += 1
    #         self.get_logger().error(
    #             f'[RIDESCAN] HIGH RISK={risk:.2f} - STOPPING ROBOT '
    #             f'(stop #{self._stop_count})'
    #         )

    #     self._publish_zero_velocity()
    #     self._cancel_navigation()
    
    
    # This version of _stop_robot only cancels the navigation goals and publishes zero velocity the first time the risk exceeds the stop threshold. 
    # If the risk remains high, it will continue publishing zero velocity but will not repeatedly cancel navigation goals or log errors until the risk drops below the threshold and then exceeds it again.
    # This allows for a single intervention per high-risk event while still ensuring the robot remains stopped as long as the risk is elevated.
    def _stop_robot(self, risk: float):
        """Cancel Nav2 goals once and keep publishing zero velocity."""
        first_stop = not self._robot_stopped

        if first_stop:
            self._robot_stopped = True
            self._stop_count += 1
            self.get_logger().error(
                f'[RIDESCAN] HIGH RISK={risk:.2f} - STOPPING ROBOT '
                f'(stop #{self._stop_count})'
            )
            self._cancel_navigation()

        self._publish_zero_velocity()
        


    def _resume_robot(self, risk: float):
        """Risk has dropped below threshold."""
        self._robot_stopped = False
        self.get_logger().info(
            f'[RIDESCAN] Risk normalised to {risk:.2f} - '
            f'robot is held. Send a new /goal_pose to resume navigation.'
        )

    def _publish_zero_velocity(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self._cmd_vel_pub.publish(stop_msg)

    def _cancel_navigation(self):
        """Cancel all active NavigateToPose goals."""
        if not self._nav_cancel_client.service_is_ready():
            if not self._nav_cancel_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn('[RIDESCAN] Nav2 cancel service not ready')
                return None

        self.get_logger().error(
            '[RIDESCAN] Cancelling Nav2 goal(s) due to high risk'
        )

        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        # Empty GoalInfo cancels all active goals for this action server.

        future = self._nav_cancel_client.call_async(request)
        future.add_done_callback(self._cancel_navigation_done)

        return future

    def _cancel_navigation_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'[RIDESCAN] Failed to cancel Nav2 goal(s): {exc}'
            )
            return

        if response is None:
            self.get_logger().warn('[RIDESCAN] Empty response from Nav2 cancel service')
            return

        self.get_logger().info(
            f'[RIDESCAN] Nav2 cancel response code={response.return_code}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RideScanSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
ridescan_bridge_node.py
-----------------------
ROS 2 bridge that streams robot telemetry to the RideScan monitoring API.

A ROS 2 node that sits between the robot and the RideScan cloud.
Robot publishes data to ROS topics as normal,
this node listens to all of them, packages the data, and sends it to RideScan's API every second.
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)

from sensor_msgs.msg import JointState, LaserScan, Image
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import TransitionEvent

import requests
import json
import time
import threading
from collections import deque
from datetime import datetime, timezone


# ═══════════════════════════════════════════════════════════════
#  CONFIG
#  DRY_RUN = True  logs to terminal, no API key needed
#  DRY_RUN = False posts live to RideScan (fill in API key first)
#
#  To go live:
#    1. Add to ~/.bashrc:  export RIDESCAN_API_KEY='your_key_here'
#    2. Set DRY_RUN = False below
#    3. Rebuild and run
# ═══════════════════════════════════════════════════════════════

RIDESCAN_API_URL    = 'https://api.ridescan.ai/v1/telemetry'
RIDESCAN_API_KEY    = os.environ.get('RIDESCAN_API_KEY', 'YOUR_API_KEY_HERE')
ROBOT_ID            = 'my_bot'
FLUSH_INTERVAL_SEC  = 1.0     # POST to API every N seconds
BATCH_SIZE          = 30      # max events per POST
RETRY_LIMIT         = 3       # retries on failed POST
COSTMAP_SAMPLE_RATE = 5       # send every Nth costmap update (grids are large)
DRY_RUN             = False    # set to False once API key is received


# QoS profiles 

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

TRANSIENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class _SafeEncoder(json.JSONEncoder):
    """Handles bytes and NaN values that ROS messages sometimes produce."""
    def default(self, obj):
        if isinstance(obj, bytes):
            return obj.decode('utf-8', errors='replace')
        if isinstance(obj, float) and (obj != obj):  # NaN check
            return None
        return str(obj)


class RideScanBridgeNode(Node):

    def __init__(self):
        super().__init__('ridescan_bridge')

        # Internal state 
        self._payload_queue   = deque(maxlen=1000)
        self._lock            = threading.Lock()
        self._costmap_counter = {'local': 0, 'global': 0}
        self._total_sent      = 0  # running event count for dry-run display

        # Subscriptions 

        # Navigation & pose
        self.create_subscription(Odometry, '/odom',
            self._odom_cb, SENSOR_QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
            self._amcl_pose_cb, RELIABLE_QOS)
        self.create_subscription(PoseStamped, '/goal_pose',
            self._goal_pose_cb, RELIABLE_QOS)

        # Planning
        self.create_subscription(Path, '/plan',
            self._plan_cb, RELIABLE_QOS)
        self.create_subscription(Path, '/local_plan',
            self._local_plan_cb, RELIABLE_QOS)
        self.create_subscription(Path, '/plan_smoothed',
            self._plan_smoothed_cb, RELIABLE_QOS)

        # Velocity
        self.create_subscription(Twist, '/cmd_vel',
            self._cmd_vel_cb, RELIABLE_QOS)
        self.create_subscription(Twist, '/cmd_vel_nav',
            self._cmd_vel_nav_cb, RELIABLE_QOS)

        # Perception
        self.create_subscription(LaserScan, '/scan',
            self._scan_cb, SENSOR_QOS)
        self.create_subscription(Image, '/camera/image_raw',
            self._camera_meta_cb, SENSOR_QOS)
        self.create_subscription(Image, '/camera/depth/image_raw',
            self._depth_meta_cb, SENSOR_QOS)

        # Costmaps (sampled — full grids are large)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
            self._local_costmap_cb, TRANSIENT_QOS)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
            self._global_costmap_cb, TRANSIENT_QOS)

        # Robot state
        self.create_subscription(JointState, '/joint_states',
            self._joint_cb, RELIABLE_QOS)
        self.create_subscription(DiagnosticArray, '/diagnostics',
            self._diagnostics_cb, RELIABLE_QOS)

        # Nav2 lifecycle — detects server crashes/restarts
        for server in [
            'bt_navigator', 'controller_server',
            'planner_server', 'smoother_server',
            'behavior_server', 'waypoint_follower',
            'velocity_smoother', 'map_server', 'amcl',
        ]:
            self.create_subscription(
                TransitionEvent,
                f'/{server}/transition_event',
                self._make_lifecycle_cb(server),
                RELIABLE_QOS,
            )

        # Flush timer
        self.create_timer(FLUSH_INTERVAL_SEC, self._flush_to_ridescan)

        mode = '[DRY RUN - logging only]' if DRY_RUN else '[LIVE - posting to API]'
        self.get_logger().info(
            f'RideScan bridge online | robot_id={ROBOT_ID} | '
            f'flush={FLUSH_INTERVAL_SEC}s | {mode}'
        )

    #  Callbacks 

    def _odom_cb(self, msg: Odometry):
        self._enqueue({
            'type': 'odometry',
            'timestamp': self._stamp(msg.header.stamp),
            'position': self._vec3(msg.pose.pose.position),
            'orientation': self._quat(msg.pose.pose.orientation),
            'linear_velocity': self._vec3(msg.twist.twist.linear),
            'angular_velocity_z': msg.twist.twist.angular.z,
            'pose_covariance_trace': sum(
                msg.pose.covariance[i] for i in [0, 7, 35]
            ),
        })

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        self._enqueue({
            'type': 'amcl_pose',
            'timestamp': self._stamp(msg.header.stamp),
            'position': self._vec3(msg.pose.pose.position),
            'orientation': self._quat(msg.pose.pose.orientation),
            'covariance_xx': cov[0],
            'covariance_yy': cov[7],
            'covariance_yaw': cov[35],
        })

    def _goal_pose_cb(self, msg: PoseStamped):
        self._enqueue({
            'type': 'goal_pose',
            'timestamp': self._stamp(msg.header.stamp),
            'position': self._vec3(msg.pose.position),
            'orientation': self._quat(msg.pose.orientation),
        })

    def _plan_cb(self, msg: Path):
        self._enqueue({
            'type': 'global_plan',
            'timestamp': self._stamp(msg.header.stamp),
            'waypoint_count': len(msg.poses),
            'start': self._vec3(msg.poses[0].pose.position) if msg.poses else None,
            'end': self._vec3(msg.poses[-1].pose.position) if msg.poses else None,
        })

    def _local_plan_cb(self, msg: Path):
        self._enqueue({
            'type': 'local_plan',
            'timestamp': self._stamp(msg.header.stamp),
            'waypoint_count': len(msg.poses),
        })

    def _plan_smoothed_cb(self, msg: Path):
        self._enqueue({
            'type': 'plan_smoothed',
            'timestamp': self._stamp(msg.header.stamp),
            'waypoint_count': len(msg.poses),
        })

    def _cmd_vel_cb(self, msg: Twist):
        self._enqueue({
            'type': 'cmd_vel',
            'timestamp': self._now_iso(),
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
        })

    def _cmd_vel_nav_cb(self, msg: Twist):
        self._enqueue({
            'type': 'cmd_vel_nav',
            'timestamp': self._now_iso(),
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z,
        })

    def _scan_cb(self, msg: LaserScan):
        ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self._enqueue({
            'type': 'lidar_scan',
            'timestamp': self._stamp(msg.header.stamp),
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'num_beams': len(msg.ranges),
            'num_valid': len(ranges),
            'min_range_observed': min(ranges) if ranges else None,
            'mean_range': sum(ranges) / len(ranges) if ranges else None,
        })

    def _camera_meta_cb(self, msg: Image):
        # Metadata only — never stream raw pixels to a monitoring API
        self._enqueue({
            'type': 'camera_frame',
            'timestamp': self._stamp(msg.header.stamp),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'is_bigendian': msg.is_bigendian,
        })

    def _depth_meta_cb(self, msg: Image):
        self._enqueue({
            'type': 'depth_frame',
            'timestamp': self._stamp(msg.header.stamp),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
        })

    def _local_costmap_cb(self, msg: OccupancyGrid):
        self._costmap_counter['local'] += 1
        if self._costmap_counter['local'] % COSTMAP_SAMPLE_RATE != 0:
            return
        data = list(msg.data)
        lethal  = data.count(100)
        unknown = data.count(-1)
        self._enqueue({
            'type': 'local_costmap_stats',
            'timestamp': self._stamp(msg.header.stamp),
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'lethal_cells': lethal,
            'unknown_cells': unknown,
            'free_cells': len(data) - lethal - unknown,
        })

    def _global_costmap_cb(self, msg: OccupancyGrid):
        self._costmap_counter['global'] += 1
        if self._costmap_counter['global'] % COSTMAP_SAMPLE_RATE != 0:
            return
        data = list(msg.data)
        lethal = data.count(100)
        self._enqueue({
            'type': 'global_costmap_stats',
            'timestamp': self._stamp(msg.header.stamp),
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'lethal_cells': lethal,
            'free_cells': len(data) - lethal - data.count(-1),
        })

    def _joint_cb(self, msg: JointState):
        self._enqueue({
            'type': 'joint_states',
            'timestamp': self._stamp(msg.header.stamp),
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
        })

    def _diagnostics_cb(self, msg: DiagnosticArray):
        statuses = [
            {
                'name': s.name,
                'level': s.level,
                'message': s.message,
                'hardware_id': s.hardware_id,
            }
            for s in msg.status
        ]
        if statuses:
            self._enqueue({
                'type': 'diagnostics',
                'timestamp': self._now_iso(),
                'statuses': statuses,
            })

    def _make_lifecycle_cb(self, server_name: str):
        def _cb(msg: TransitionEvent):
            self._enqueue({
                'type': 'lifecycle_transition',
                'timestamp': self._now_iso(),
                'server': server_name,
                'start_state': msg.start_state.label,
                'goal_state': msg.goal_state.label,
            })
        return _cb

    #  Queue & flush 

    def _enqueue(self, payload: dict):
        payload['robot_id'] = ROBOT_ID
        with self._lock:
            self._payload_queue.append(payload)

    def _flush_to_ridescan(self):
        with self._lock:
            if not self._payload_queue:
                return
            batch = [
                self._payload_queue.popleft()
                for _ in range(min(BATCH_SIZE, len(self._payload_queue)))
            ]

        if DRY_RUN:
            self._dry_run_print(batch)
        else:
            self._post_with_retry(batch)

    def _dry_run_print(self, batch: list):
        """Print a clean summary of what would be sent to RideScan."""
        self._total_sent += len(batch)

        type_counts = {}
        for event in batch:
            t = event.get('type', 'unknown')
            type_counts[t] = type_counts.get(t, 0) + 1

        summary = ' | '.join(f'{t}={c}' for t, c in sorted(type_counts.items()))

        self.get_logger().info(
            f'[DRY RUN] Would POST {len(batch)} events to RideScan '
            f'(total so far: {self._total_sent}) → {summary}'
        )

        self.get_logger().info(
            f'[DRY RUN] Sample payload: {json.dumps(batch[0], indent=2, cls=_SafeEncoder)}'
        )

    def _post_with_retry(self, batch: list, attempt: int = 0):
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {RIDESCAN_API_KEY}',
            'X-Robot-ID': ROBOT_ID,
        }
        try:
            resp = requests.post(
                RIDESCAN_API_URL,
                headers=headers,
                data=json.dumps({'events': batch}, cls=_SafeEncoder),
                timeout=5.0,
            )
            if resp.status_code == 200:
                self.get_logger().info(f'Flushed {len(batch)} events → RideScan')
            else:
                self.get_logger().warn(
                    f'RideScan {resp.status_code}: {resp.text[:120]}'
                )
                self._maybe_retry(batch, attempt)
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'RideScan POST error: {e}')
            self._maybe_retry(batch, attempt)

    def _maybe_retry(self, batch: list, attempt: int):
        if attempt < RETRY_LIMIT:
            backoff = 2 ** attempt
            self.get_logger().info(f'Retry in {backoff}s (attempt {attempt + 1})')
            time.sleep(backoff)
            self._post_with_retry(batch, attempt + 1)
        else:
            self.get_logger().error(
                f'Dropping {len(batch)} events after {RETRY_LIMIT} retries'
            )

    # ── Serialisation helpers ──────────────────────────────────

    @staticmethod
    def _stamp(s) -> str:
        ts = s.sec + s.nanosec * 1e-9
        return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()

    def _now_iso(self) -> str:
        return datetime.now(tz=timezone.utc).isoformat()

    @staticmethod
    def _vec3(v) -> dict:
        return {'x': v.x, 'y': v.y, 'z': v.z}

    @staticmethod
    def _quat(q) -> dict:
        return {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}


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


if __name__ == '__main__':
    main()
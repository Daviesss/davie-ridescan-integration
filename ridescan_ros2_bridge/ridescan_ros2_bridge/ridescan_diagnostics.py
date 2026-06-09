#!/usr/bin/env python3
"""
ridescan_diagnostics_node.py

Polls the RideScan API every 5 seconds for risk scores and anomalies,
then publishes them back into ROS 2 so the robot can react.

Publishes:
  /ridescan/risk_score    (std_msgs/Float32)    0.0 (healthy) to 1.0 (critical)
  /ridescan/diagnostics   (diagnostic_msgs/DiagnosticArray) OK / WARN / ERROR

How it works:
  1. Every 5 seconds, sends GET request to RideScan API
  2. RideScan responds with risk score and anomaly list
  3. Node publishes risk score to /ridescan/risk_score
  4. Node publishes diagnostic status to /ridescan/diagnostics
  5. Other nodes (e.g. safety node) can subscribe and react
  
  Author: Davies Iyanuoluwa Ogunsina
"""

import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import requests
import json



#  CONFIG
#  Same API key as the bridge node
#  DRY_RUN = True  simulates a RideScan response locally
#  DRY_RUN = False  polls real RideScan API


RIDESCAN_API_URL = 'https://api.ridescan.ai/v1/status'
RIDESCAN_API_KEY = os.environ.get('RIDESCAN_API_KEY', 'YOUR_API_KEY_HERE')
ROBOT_ID         = 'my_bot' # robot ID name
POLL_INTERVAL    = 5.0    # seconds between polls
DRY_RUN          = True   # set to False once API key is received

# Risk score thresholds
THRESHOLD_WARN  = 0.3     # above this → WARN
THRESHOLD_ERROR = 0.7     # above this → ERROR / stop robot


class RideScanDiagnosticsNode(Node):

    def __init__(self):
        super().__init__('ridescan_diagnostics')

        # ── Publishers ─────────────────────────────────────────
        self._risk_pub = self.create_publisher(
            Float32, '/ridescan/risk_score', 10
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/ridescan/diagnostics', 10
        )

        # ── Poll timer ─────────────────────────────────────────
        self.create_timer(POLL_INTERVAL, self._poll_ridescan)

        mode = '[DRY RUN - simulated response]' if DRY_RUN else '[LIVE - polling API]'
        self.get_logger().info(
            f'RideScan diagnostics online | robot_id={ROBOT_ID} | '
            f'poll={POLL_INTERVAL}s | {mode}'
        )

    # ── Poll ───────────────────────────────────────────────────

    def _poll_ridescan(self):
        if DRY_RUN:
            # Simulate a RideScan response so you can test without API key
            data = self._simulated_response()
            self.get_logger().info(
                f'[DRY RUN] Simulated RideScan response: '
                f'risk_score={data["risk_score"]} | '
                f'anomalies={len(data["anomalies"])}'
            )
        else:
            data = self._fetch_from_ridescan()
            if data is None:
                return  # fetch failed, already logged

        self._publish(data)

    def _fetch_from_ridescan(self):
        """GET risk score and anomalies from RideScan API."""
        headers = {
            'Authorization': f'Bearer {RIDESCAN_API_KEY}',
            'X-Robot-ID': ROBOT_ID,
        }
        try:
            resp = requests.get(
                f'{RIDESCAN_API_URL}/{ROBOT_ID}',
                headers=headers,
                timeout=5.0,
            )
            if resp.status_code == 200:
                return resp.json()
            else:
                self.get_logger().warn(
                    f'RideScan API {resp.status_code}: {resp.text[:120]}'
                )
                return None
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'RideScan poll error: {e}')
            return None

    def _simulated_response(self) -> dict:
        """
        Simulates what RideScan would return.
        Cycles through increasing risk scores so you can see
        the node publishing different levels during dry run.
        """
        import math
        import time
        # Oscillate risk score between 0.1 and 0.9 over time
        t = time.time()
        score = round(0.5 + 0.4 * math.sin(t / 10.0), 2)

        anomalies = []
        if score > THRESHOLD_WARN:
            anomalies.append({'type': 'localisation_uncertainty', 'severity': 'medium'})
        if score > THRESHOLD_ERROR:
            anomalies.append({'type': 'wheel_velocity_anomaly', 'severity': 'high'})

        return {
            'risk_score': score,
            'anomalies': anomalies,
        }

    # ── Publish ────────────────────────────────────────────────

    def _publish(self, data: dict):
        risk_score = float(data.get('risk_score', 0.0))
        anomalies  = data.get('anomalies', [])

        # ── /ridescan/risk_score ──────────────────────────────
        risk_msg = Float32()
        risk_msg.data = risk_score
        self._risk_pub.publish(risk_msg)

        # ── /ridescan/diagnostics ─────────────────────────────
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name        = 'RideScan Monitor'
        status.hardware_id = ROBOT_ID

        if risk_score < THRESHOLD_WARN:
            status.level   = DiagnosticStatus.OK
            status.message = f'Robot operating normally | risk={risk_score:.2f}'
        elif risk_score < THRESHOLD_ERROR:
            status.level   = DiagnosticStatus.WARN
            status.message = f'Elevated risk detected | risk={risk_score:.2f}'
        else:
            status.level   = DiagnosticStatus.ERROR
            status.message = f'HIGH RISK — intervention recommended | risk={risk_score:.2f}'

        # Attach anomalies as key-value pairs
        for anomaly in anomalies:
            kv       = KeyValue()
            kv.key   = anomaly.get('type', 'unknown')
            kv.value = anomaly.get('severity', '')
            status.values.append(kv)

        # Always attach the raw risk score
        kv_score       = KeyValue()
        kv_score.key   = 'risk_score'
        kv_score.value = str(risk_score)
        status.values.append(kv_score)

        diag_array.status.append(status)
        self._diag_pub.publish(diag_array)

        self.get_logger().info(
            f'RideScan | risk={risk_score:.2f} | '
            f'level={status.message} | '
            f'anomalies={len(anomalies)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RideScanDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

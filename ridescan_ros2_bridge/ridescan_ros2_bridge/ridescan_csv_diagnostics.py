#!/usr/bin/env python3
"""

Polls the RideScan API for inference results and publishes risk scores
back into ROS 2 so the robot can react.

The RideScan SDK is async: files are uploaded by the bridge node, inference
is triggered here, and get_model_status() is polled until results are ready.

Publishes:
  /ridescan/risk_score    (std_msgs/Float32)         0.0 (healthy) to 1.0 (critical)
  /ridescan/diagnostics   (diagnostic_msgs/DiagnosticArray)  OK / WARN / ERROR
  
  Author: Davies Iyanuoluwa Ogunsina
"""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Float32

# CONFIG — fill these in before going live
API_KEY    = "rsk_your_api_key_here"
ROBOT_ID   = "my_bot"
MISSION_ID = "your-mission-uuid-here"
DRY_RUN    = True    # set to False once API key is received

POLL_INTERVAL   = 10.0   # seconds between get_model_status() calls

# Risk score thresholds
THRESHOLD_WARN  = 0.3    # above this -> WARN
THRESHOLD_ERROR = 0.7    # above this -> ERROR


class RideScanDiagnosticsNode(Node):

    def __init__(self):
        super().__init__("ridescan_diagnostics")

        self._risk_pub = self.create_publisher(Float32, "/ridescan/risk_score", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/ridescan/diagnostics", 10)

        self._inference_triggered = False

        self.create_timer(POLL_INTERVAL, self._poll)

        mode = "[DRY RUN]" if DRY_RUN else "[LIVE]"
        self.get_logger().info(
            f"RideScan diagnostics online {mode} | "
            f"robot_id={ROBOT_ID} | mission_id={MISSION_ID} | poll={POLL_INTERVAL}s"
        )

    def _poll(self):
        if DRY_RUN:
            data = self._simulated_status()
            self.get_logger().info(
                f"[DRY RUN] mission_avg_risk_score={data['mission_avg_risk_score']} | "
                f"inference_status={data['inference_status']}"
            )
            self._publish(data)
            return

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

                if not self._inference_triggered:
                    self.get_logger().info("Triggering RideScan inference...")
                    client.run_inference(
                        robot_id=ROBOT_ID,
                        mission_id=MISSION_ID,
                        device="cpu",
                    )
                    self._inference_triggered = True
                    self.get_logger().info("Inference queued. Polling for results...")
                    return

                data = client.get_model_status(mission_id=MISSION_ID)

        except RideScanError as exc:
            self.get_logger().error(f"RideScan SDK error: {exc}")
            return
        except Exception as exc:
            self.get_logger().error(f"Unexpected error: {exc}")
            return

        inference_status = data.get("inference_status", "unknown")

        if inference_status != "completed":
            self.get_logger().info(f"Inference not ready yet | status={inference_status}")
            return

        self._publish(data)

    def _publish(self, data: dict):
        risk_score = float(data.get("mission_avg_risk_score") or 0.0)
        inference_status = data.get("inference_status", "unknown")
        files = data.get("files", [])

        # /ridescan/risk_score
        risk_msg = Float32()
        risk_msg.data = risk_score
        self._risk_pub.publish(risk_msg)

        # /ridescan/diagnostics
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name        = "RideScan Monitor"
        status.hardware_id = ROBOT_ID

        if risk_score < THRESHOLD_WARN:
            status.level   = DiagnosticStatus.OK
            status.message = f"Robot operating normally | risk={risk_score:.2f}"
        elif risk_score < THRESHOLD_ERROR:
            status.level   = DiagnosticStatus.WARN
            status.message = f"Elevated risk detected | risk={risk_score:.2f}"
        else:
            status.level   = DiagnosticStatus.ERROR
            status.message = f"HIGH RISK - intervention recommended | risk={risk_score:.2f}"

        kv = KeyValue()
        kv.key   = "mission_avg_risk_score"
        kv.value = str(risk_score)
        status.values.append(kv)

        for f in files:
            kv = KeyValue()
            kv.key   = f.get("filename", "unknown")
            kv.value = str(f.get("risk_score") or "pending")
            status.values.append(kv)

        kv = KeyValue()
        kv.key   = "inference_status"
        kv.value = inference_status
        status.values.append(kv)

        diag_array.status.append(status)
        self._diag_pub.publish(diag_array)

        self.get_logger().info(
            f"RideScan | risk={risk_score:.2f} | status={status.message} | files={len(files)}"
        )

    def _simulated_status(self) -> dict:
        """Simulates a completed get_model_status() response for dry-run testing."""
        t = time.time()
        score = round(0.5 + 0.4 * math.sin(t / 10.0), 2)
        return {
            "inference_status": "completed",
            "mission_avg_risk_score": score,
            "robot_avg_risk_score": score,
            "files": [
                {"filename": "run1.csv", "risk_score": round(score - 0.05, 2)},
                {"filename": "run2.csv", "risk_score": round(score + 0.05, 2)},
            ],
        }


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


if __name__ == "__main__":
    main()
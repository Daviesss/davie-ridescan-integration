"""
ridescan_safety_monitor_node.py

Buffers /odom into rolling CSV batches, uploads each batch to RideScan as a
process_file, runs inference, and if the returned risk_score exceeds a
threshold, publishes a safety-stop signal that square_controller.py listens
for (since this robot uses a direct /cmd_vel P-controller).

Assumes calibration has ALREADY been completed for this robot_id/mission_id.(calibration initially done by ride_scan_calibration_risk_score.py node)

Author : Davies Iyanuoluwa Ogunsina
Maintainer : Davies Iyanuoluwa Ogunsina
"""
import csv
import os
import tempfile
import threading
import math
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool

from .ridescan_client import RideScanClient

try:
    from twilio.rest import Client as TwilioClient
    TWILIO_AVAILABLE = True
except ImportError:
    TWILIO_AVAILABLE = False
# --- Twilio disabled for now ---
TWILIO_AVAILABLE = False


class RideScanSafetyMonitor(Node):
    def __init__(self):
        super().__init__("ridescan_safety_monitor")

        # ---- Config - real values from your completed calibration ----
        self.declare_parameter("api_key", os.environ.get("RIDESCAN_API_KEY", "YOUR_API_KEY"))
        self.declare_parameter("robot_id", "eaa3a139-8e8d-447f-94de-839baa1de39b") 
        self.declare_parameter("mission_id", "56c59a55-46fb-4abf-8b91-d11a93ddb009") 
        self.declare_parameter("robot_type", "wheeled_mobile")
        self.declare_parameter("batch_seconds", 30.0)
        # Calibration range was ~0-29 (max was mission_eight.csv at 29.19).
        # Start conservative; tune upward once you've observed real live scores.
        # self.declare_parameter("risk_threshold", 5.0)
        self.declare_parameter("risk_threshold", 40.0)

        # Twilio SMS alerting set these via env vars, not hardcoded
        self.declare_parameter("twilio_account_sid", os.environ.get("TWILIO_ACCOUNT_SID", ""))
        self.declare_parameter("twilio_auth_token", os.environ.get("TWILIO_AUTH_TOKEN", ""))
        self.declare_parameter("twilio_from_number", os.environ.get("TWILIO_FROM_NUMBER", ""))
        self.declare_parameter("twilio_to_number", os.environ.get("TWILIO_TO_NUMBER", ""))

        self.api_key = self.get_parameter("api_key").value
        self.robot_id = self.get_parameter("robot_id").value
        self.mission_id = self.get_parameter("mission_id").value
        self.robot_type = self.get_parameter("robot_type").value
        self.batch_seconds = self.get_parameter("batch_seconds").value
        self.risk_threshold = self.get_parameter("risk_threshold").value

        # Twilio setup
        self.twilio_sid = self.get_parameter("twilio_account_sid").value
        self.twilio_token = self.get_parameter("twilio_auth_token").value
        self.twilio_from = self.get_parameter("twilio_from_number").value
        self.twilio_to = self.get_parameter("twilio_to_number").value
        self.twilio_client = None
        if TWILIO_AVAILABLE and self.twilio_sid and self.twilio_token:
            self.twilio_client = TwilioClient(self.twilio_sid, self.twilio_token)
            self.get_logger().info("Twilio SMS alerting enabled.")
        else:
            self.get_logger().warn(
                "Twilio not configured (missing library or credentials) - "
                "SMS alerts disabled, will only log locally.")

        self.client = RideScanClient(api_key=self.api_key)

        # ---- Odometry buffering ----
        self._buffer_lock = threading.Lock()
        self._rows = []
        # MUST match the exact schema of your original training CSVs:
        # timestamp,pose_position_x,pose_position_y,pose_position_z,
        # pose_orientation_roll,pose_orientation_pitch,pose_orientation_yaw,
        # twist_linear_x,twist_linear_y,twist_linear_z,
        # twist_angular_x,twist_angular_y,twist_angular_z,
        # linear_acceleration_x,linear_acceleration_y,linear_acceleration_z
        self._row_header = [
            "timestamp",
            "pose_position_x", "pose_position_y", "pose_position_z",
            "pose_orientation_roll", "pose_orientation_pitch", "pose_orientation_yaw",
            "twist_linear_x", "twist_linear_y", "twist_linear_z",
            "twist_angular_x", "twist_angular_y", "twist_angular_z",
            "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z",
        ]
        self._prev_twist_linear = None
        self._prev_time = None

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10)

        # Safety action interfaces
        # This robot uses square_controller.py (direct /cmd_vel P-controller),
        # NOT Nav2 actions , so we signal it via a Bool topic it subscribes to,
        # rather than trying to cancel a Nav2 goal that doesn't exist here.
        self.safety_stop_pub = self.create_publisher(Bool, "/ridescan/safety_stop", 10)
        self.risk_score_pub = self.create_publisher(Float32, "/ridescan/risk_score", 10)
        self._is_stopped = False
        self._processing = False  # guards against overlapping batch cycles

        self.create_timer(self.batch_seconds, self._trigger_batch_cycle)

        self.get_logger().info(
            f"RideScan safety monitor started. Batch every {self.batch_seconds}s, "
            f"risk threshold={self.risk_threshold}")

    def _send_sms_alert(self, message: str):
        if not self.twilio_client:
            return
        try:
            self.twilio_client.messages.create(
                body=message,
                from_=self.twilio_from,
                to=self.twilio_to,
            )
            self.get_logger().info("SMS alert sent.")
        except Exception as e:
            self.get_logger().error(f"Failed to send SMS alert: {e}")

    def _quaternion_to_euler(self, q):
        """Convert geometry_msgs/Quaternion to (roll, pitch, yaw) in radians."""
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))  # clamp for numerical safety
        pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _odom_callback(self, msg: Odometry):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        roll, pitch, yaw = self._quaternion_to_euler(msg.pose.pose.orientation)

        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        # Odometry has no native acceleration field  approximate it via
        # finite difference of linear velocity between consecutive callbacks,
        # matching the linear_acceleration_x/y/z columns your training data
        # includes (likely originally sourced from an IMU).
        if self._prev_twist_linear is not None and self._prev_time is not None:
            dt = now_sec - self._prev_time
            if dt > 1e-6:
                accel_x = (lin.x - self._prev_twist_linear.x) / dt
                accel_y = (lin.y - self._prev_twist_linear.y) / dt
                accel_z = (lin.z - self._prev_twist_linear.z) / dt
            else:
                accel_x = accel_y = accel_z = 0.0
        else:
            accel_x = accel_y = accel_z = 0.0

        self._prev_twist_linear = lin
        self._prev_time = now_sec

        row = [
            self.get_clock().now().to_msg().sec,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            roll, pitch, yaw,
            lin.x, lin.y, lin.z,
            ang.x, ang.y, ang.z,
            accel_x, accel_y, accel_z,
        ]
        with self._buffer_lock:
            self._rows.append(row)

    def _trigger_batch_cycle(self):
        if self._processing:
            self.get_logger().warn("Previous batch still processing; skipping this cycle.")
            return

        with self._buffer_lock:
            if not self._rows:
                return
            rows_to_send = self._rows
            self._rows = []

        self._processing = True
        thread = threading.Thread(
            target=self._process_batch, args=(rows_to_send,), daemon=True)
        thread.start()

    def _process_batch(self, rows):
        fd, path = tempfile.mkstemp(suffix=".csv", prefix="ridescan_batch_")
        try:
            with os.fdopen(fd, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(self._row_header)
                writer.writerows(rows)

            event_time = datetime.now(timezone.utc).isoformat()

            try:
                upload_resp = self.client.upload_files(
                    self.robot_id, self.mission_id, "process_file",
                    [path], event_times=[event_time])
                uploaded = upload_resp["data"]["uploaded_files"]
            except Exception as upload_err:
                # RideScan's gateway sometimes 502s even when the file was
                # actually saved server-side. Instead of giving up, check
                # the inference files list for a recently-uploaded file
                # matching our local filename before treating this as a
                # real failure. Retry the check a few times since the
                # server may take a moment to register the file even
                # though the upload itself already landed.
                self.get_logger().warn(
                    f"Upload request failed ({upload_err}); checking if it "
                    f"landed on the server anyway before giving up...")
                local_name = os.path.basename(path)
                uploaded = []
                import requests as _requests
                import time as _time
                for attempt in range(4):
                    _time.sleep(3)  # give the backend a moment to register it
                    try:
                        r = _requests.get(
                            f"{self.client.base_url}/api/inference/files",
                            params={"robot_id": self.robot_id, "mission_id": self.mission_id},
                            headers=self.client.headers_multipart, timeout=self.client.timeout)
                        r.raise_for_status()
                        recent_files = r.json().get("data", {}).get("files", [])
                        for f in recent_files:
                            if local_name in f.get("unique_filename", "") or \
                               local_name in f.get("original_filename", ""):
                                uploaded = [f]
                                self.get_logger().info(
                                    "Confirmed: file actually landed despite the "
                                    "502 - continuing with inference.")
                                break
                        if uploaded:
                            break
                        self.get_logger().warn(
                            f"Verification attempt {attempt + 1}/4: file not "
                            f"found yet ({len(recent_files)} files on record).")
                    except Exception as check_err:
                        self.get_logger().warn(
                            f"Verification attempt {attempt + 1}/4 failed: {check_err}")
                if not uploaded:
                    self.get_logger().error(
                        "File could not be confirmed on server after retries; "
                        "treating upload as genuinely failed.")
                    raise upload_err

            if not uploaded:
                self.get_logger().warn("Upload returned no files; skipping inference")
                return
            blob_name = uploaded[0]["unique_filename"]

            infer_resp = self.client.run_inference(
                self.robot_id, self.mission_id,
                blob_names=[blob_name], robot_type=self.robot_type)
            infer_id = infer_resp["data"]["infer_id"]

            result = self.client.wait_for_inference(
                self.robot_id, self.mission_id, infer_id,
                poll_interval=5, max_wait=300)

            scores = [f["risk_score"] for f in result.get("files", [])]
            if not scores:
                return
            max_score = max(scores)

            self.get_logger().info(f"Batch risk score: {max_score}")
            self.risk_score_pub.publish(Float32(data=float(max_score)))

            if max_score >= self.risk_threshold:
                if not self._is_stopped:
                    self.get_logger().warn(
                        f"Risk score {max_score} >= threshold {self.risk_threshold}. "
                        f"Publishing safety stop.")
                    self._send_sms_alert(
                        f"RideScan ALERT: Davie-Perimeter-Bot safety stop triggered. "
                        f"Risk score {max_score:.2f} (threshold {self.risk_threshold}).")
                self._is_stopped = True
            else:
                if self._is_stopped:
                    self.get_logger().info(
                        f"Risk score {max_score} back below threshold. Clearing stop.")
                    self._send_sms_alert(
                        f"RideScan: Davie-Perimeter-Bot resumed. "
                        f"Risk score {max_score:.2f} back below threshold.")
                self._is_stopped = False

            self.safety_stop_pub.publish(Bool(data=self._is_stopped))

        except Exception as e:
            self.get_logger().error(f"RideScan batch processing failed: {e}")
        finally:
            if os.path.exists(path):
                os.remove(path)
            self._processing = False


def main(args=None):
    rclpy.init(args=args)
    node = RideScanSafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
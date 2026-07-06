"""
RideScan Stage 3 API client.
"""
import time
import requests
from datetime import datetime, timezone

BASE_URL = ""


class RideScanClient:
    def __init__(self, api_key: str, base_url: str = BASE_URL, timeout: int = 15):
        self.base_url = base_url
        self.timeout = timeout
        self.headers_json = {"Content-Type": "application/json", "X-API-KEY": api_key}
        self.headers_multipart = {"X-API-KEY": api_key}

    def create_robot(self, robot_name: str, robot_type: str = "wheeled_mobile"):
        payload = {"params": {"robot_name": robot_name, "robot_type": robot_type}}
        r = requests.post(f"{self.base_url}/api/robot/create",
                           json=payload, headers=self.headers_json, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def get_robots(self, criteria: dict = None):
        r = requests.post(f"{self.base_url}/api/getrobot",
                           json={"criteria": criteria or {}}, headers=self.headers_json,
                           timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def create_mission(self, robot_id: str, mission_name: str):
        payload = {"params": {"robot_id": robot_id, "mission_name": mission_name}}
        r = requests.post(f"{self.base_url}/api/createmission",
                           json=payload, headers=self.headers_json, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def upload_files(self, robot_id: str, mission_id: str, file_type: str,
                      filepaths, event_times=None):
        if event_times is None:
            now_iso = datetime.now(timezone.utc).isoformat()
            event_times = [now_iso] * len(filepaths)
        files = [("files", (fp.split("/")[-1], open(fp, "rb"), "text/csv")) for fp in filepaths]
        data = {
            "robot_id": robot_id,
            "mission_id": mission_id,
            "file_type": file_type,
            "event_times": event_times,
        }
        try:
            r = requests.post(f"{self.base_url}/api/file/upload", headers=self.headers_multipart,
                               data=data, files=files, timeout=self.timeout)
            r.raise_for_status()
            return r.json()
        finally:
            for _, (_, fh, _) in files:
                fh.close()

    def get_calibration_files(self, robot_id: str, mission_id: str):
        r = requests.get(f"{self.base_url}/api/calibration/files",
                          params={"robot_id": robot_id, "mission_id": mission_id},
                          headers=self.headers_multipart, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def calibrate(self, robot_id: str, mission_id: str, robot_type: str,
                  blob_names=None, retrain=None):
        payload = {"robot_id": robot_id, "mission_id": mission_id, "robot_type": robot_type}
        if blob_names:
            payload["blob_names"] = blob_names
        if retrain is not None:
            payload["retrain"] = retrain
        r = requests.post(f"{self.base_url}/api/calibrate",
                           json=payload, headers=self.headers_json, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def get_calibration_details(self, robot_id: str, mission_id: str):
        r = requests.get(f"{self.base_url}/api/calibration/details",
                          params={"robot_id": robot_id, "mission_id": mission_id},
                          headers=self.headers_multipart, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def wait_for_calibration(self, robot_id: str, mission_id: str,
                              poll_interval: int = 10, max_wait: int = 600):
        waited = 0
        while waited < max_wait:
            details = self.get_calibration_details(robot_id, mission_id)
            calibs = details.get("data", {}).get("calibrations", [])
            if calibs and calibs[0].get("status") == "calibration_completed":
                return calibs[0]
            time.sleep(poll_interval)
            waited += poll_interval
        raise TimeoutError("Calibration did not complete within max_wait seconds")

    def run_inference(self, robot_id: str, mission_id: str, blob_names=None, robot_type=None):
        payload = {"robot_id": robot_id, "mission_id": mission_id}
        if blob_names:
            payload["blob_names"] = blob_names
        if robot_type:
            payload["robot_type"] = robot_type
        r = requests.post(f"{self.base_url}/api/process",
                           json=payload, headers=self.headers_json, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def get_inference_details(self, robot_id: str, mission_id: str):
        r = requests.get(f"{self.base_url}/api/inference/details",
                          params={"robot_id": robot_id, "mission_id": mission_id},
                          headers=self.headers_multipart, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def wait_for_inference(self, robot_id: str, mission_id: str, infer_id: str,
                            poll_interval: int = 5, max_wait: int = 300):
        waited = 0
        while waited < max_wait:
            details = self.get_inference_details(robot_id, mission_id)
            for inf in details.get("data", {}).get("inferences", []):
                if inf.get("infer_id") == infer_id and inf.get("inference_status") == "processed":
                    return inf
            time.sleep(poll_interval)
            waited += poll_interval
        raise TimeoutError("Inference did not complete within max_wait seconds")
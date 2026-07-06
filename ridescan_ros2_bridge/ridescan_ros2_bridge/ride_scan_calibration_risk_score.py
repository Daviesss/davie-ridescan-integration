"""
RideScan Stage 3 API client.
Wraps the full pipeline: robot -> mission -> calibration -> inference -> risk score.
No live per-point streaming endpoint exists; ingestion is file-based (CSV upload).


Author: Davies Iyanuoluwa Ogunsina
Maintanier : Davies Iyanuoluwa Ogunsina
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


# Main function...
if __name__ == "__main__":
    import os
    API_KEY = os.environ.get("RIDESCAN_API_KEY", "YOUR_API_KEY")
    client = RideScanClient(api_key=API_KEY)

    robots = client.get_robots({"robot_name": "Davie_Perimeter_Bot"}) # was Davie-Perimeter-Bot
    robot_list = robots.get("robot_list", [])
    if not robot_list:
        raise SystemExit("No robot found — check name or create it first.")
    robot_id = robot_list[0]["robot_id"]
    robot_type = robot_list[0]["robot_type"]
    print("Robot:", robot_id, "| type:", robot_type)

    mission_resp = requests.post(
        f"{client.base_url}/api/getmission",
        json={"criteria": {"robot_id": robot_id, "mission_name": "Warehouse-Perimeter-Inspection"}},
        headers=client.headers_json,
        timeout=client.timeout,
    )
    mission_resp.raise_for_status()
    mission_list = mission_resp.json().get("mission_list", [])
    if not mission_list:
        raise SystemExit("No mission found — check name or create it first.")
    mission_id = mission_list[0]["mission_id"]
    print("Mission:", mission_id)

    files_data = client.get_calibration_files(robot_id, mission_id)
    calib_files = files_data.get("data", {}).get("files", [])
    print(f"Found {len(calib_files)} calibration files registered")
    if len(calib_files) < 15:
        print(f"WARNING: docs require a minimum of 15 calib files, found {len(calib_files)}")
    blob_names = [f["unique_filename"] for f in calib_files]

    # mision_one.csv has failed in every calibration attempt so far
    # ("Updated 14 files, failed 1 files" - always this one file).
    # Exclude it and calibrate on the remaining 15, which still meets
    # the documented minimum of 15 files.
    blob_names = [b for b in blob_names if "mision_one" not in b]
    print(f"Excluding mision_one.csv. Calibrating on {len(blob_names)} files.")
    print("blob_names:", blob_names)

    # A calibration is already in progress (status: "calibrating") from an
    # earlier successful trigger. Don't retrigger — just wait for it.
    print("\nChecking existing calibration status first...")
    existing_details = client.get_calibration_details(robot_id, mission_id)
    existing_calibs = existing_details.get("data", {}).get("calibrations", [])

    if existing_calibs and existing_calibs[0].get("status") == "calibration_completed":
        print("\n>>> Calibration already completed! <<<")
        print(existing_calibs[0])
    elif existing_calibs and existing_calibs[0].get("status") == "calibrating":
        print(f"\nCalibration already in progress (calib_id={existing_calibs[0]['calib_id']}). "
              f"Waiting for it to finish (polling every 15s)...")
        completed = client.wait_for_calibration(robot_id, mission_id,
                                                  poll_interval=15, max_wait=1200)
        print("\n>>> Calibration completed! <<<")
        print(completed)
    else:
        # No calibration running/completed - safe to trigger a fresh one
        try:
            calib_trigger = client.calibrate(robot_id, mission_id, robot_type,
                                              blob_names=blob_names)
            print("Calibration queued:", calib_trigger)
            completed = client.wait_for_calibration(robot_id, mission_id,
                                                      poll_interval=15, max_wait=1200)
            print("Calibration completed:", completed)
        except requests.exceptions.HTTPError as e:
            print("Status code:", e.response.status_code)
            print("Response body:", e.response.text)
            raise

    print("Waiting for calibration to complete...")
    completed = client.wait_for_calibration(robot_id, mission_id)
    print("Calibration completed:", completed)

    # ---------------------------------------------------------------
    # Step 6: Run inference
    # Upload a live/test telemetry file as process_file, trigger
    # inference, then poll for the risk score.
    # ---------------------------------------------------------------
    print("\n--- Step 6: Run inference ---")

    # Point this at a real file - reuse an existing CSV as a test,
    # or a fresh capture from your waypoint node.
    process_csvs = ["/home/ubuntu/ros2_ws/src/davie-ridescan-integration/ridescan_ros2_bridge/log_file/mission_nine.csv"]

    proc_upload = client.upload_files(robot_id, mission_id, "process_file", process_csvs)
    print("Process upload:", proc_upload)
    uploaded_blob_names = [f["unique_filename"] for f in proc_upload["data"]["uploaded_files"]]
    print("Uploaded blob names:", uploaded_blob_names)

    try:
        infer_trigger = client.run_inference(robot_id, mission_id,
                                              blob_names=uploaded_blob_names,
                                              robot_type=robot_type)
        print("Inference queued:", infer_trigger)
    except requests.exceptions.HTTPError as e:
        print("Status code:", e.response.status_code)
        print("Response body:", e.response.text)
        raise

    print("Waiting for inference to complete...")
    result = client.wait_for_inference(robot_id, mission_id, infer_trigger["data"]["infer_id"])
    print("\n>>> Risk scores <<<")
    for f in result["files"]:
        print(f"  {f['original_filename']}: {f['risk_score']}")

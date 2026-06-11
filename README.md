# davie-ridescan-integration
A production-grade ROS 2 integration for the RideScan Safety Layer API: telemetry bridge, risk diagnostics, and autonomous safety response for mobile robots.


## Nodes

### 1. `ridescan_bridge_node`

The telemetry extraction and upload layer. Subscribes to `/odom`, `/scan`,
and `/cmd_vel`, batches the telemetry into timestamped CSV files, and
uploads them to a RideScan robot mission.

In RideScan's architecture terms, this node is what produces the Mission
Instance files. Every time the robot completes one mission run, this node
has been silently recording everything and writes it out as one clean CSV
representing that single run.

- Registers the robot and mission on RideScan automatically on first upload
- Batches telemetry rows into CSV files every 60 seconds
- Uploads each CSV to RideScan as a mission file
- Flushes remaining data to disk on shutdown so no telemetry is lost
- The mode writes CSVs locally without uploading

**Terminal 1  start the bridge node first and leave it running:**
```bash
ros2 run ridescan_ros2_bridge ride_scan_csv_node
```

**Terminal 2  run the mission:**
```bash
for i in {1..15}; do
  echo "Starting calibration run $i of 15..."
  ros2 run ridescan_ros2_bridge way_point_follower_node
  echo "Run $i complete."
  sleep 2
done
```

**Alternative.. one bridge per run (cleanest CSV-per-run boundary):**
```bash
for i in {1..15}; do
  echo "Starting calibration run $i of 15..."
  
  # start bridge node in background
  ros2 run ridescan_ros2_bridge ride_scan_csv_node &
  BRIDGE_PID=$!
  
  # run one mission
  ros2 run ridescan_ros2_bridge way_point_follower_node
  
  # kill bridge node.... destroy_node() flushes remaining rows to CSV
  kill $BRIDGE_PID
  
  echo "Run $i complete. CSV written."
  sleep 3
done
```

---

### 2. `way_point_follower_node`

The mission execution layer. Sends Davie through a fixed 5-waypoint
perimeter loop using Nav2's `NavigateToPose` action client.

**The route (warehouse perimeter inspection):**

| Waypoint | x | y | Yaw | Description |
|---|---|---|---|---|
| 1 | 1.0 | 0.0 | 0° | Dock exit |
| 2 | 1.0 | 2.5 | 90° | Corner A |
| 3 | -1.0 | 2.5 | 180° | Corner B |
| 4 | -1.0 | 0.0 | 270° | Corner C |
| 5 | 0.0 | 0.0 | 0° | Return to dock |

For each waypoint, the node converts the yaw angle to a quaternion, sends
a `NavigateToPose` goal to Nav2, and waits for confirmation of arrival
before proceeding to the next. If any waypoint fails or times out (60s),
the mission aborts and logs the failure.

One full execution of this script = one complete mission run. Run it 15
times (alongside `ridescan_bridge_node`) to produce the calibration
baseline dataset.

```bash
ros2 run ridescan_ros2_bridge way_point_follower_node
```

**Role in the Stage 2 calibration setup:**
This is the node that generates the consistent, repeatable navigation
behavior that `ridescan_bridge_node` records as telemetry. Run it 15 times
with the bridge running alongside, and together they produce the calibration
baseline dataset , one complete perimeter inspection per run, captured as a
timestamped CSV.

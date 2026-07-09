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

**Terminal 1 start the bridge node first and leave it running:**
```bash
ros2 run ridescan_ros2_bridge ride_scan_csv_node
```

**Terminal 2 run the mission:**
```bash
for i in {1..15}; do
  echo "Starting calibration run $i of 15..."
  ros2 run ridescan_ros2_bridge way_point_follower_node
  echo "Run $i complete."
  sleep 2
done
```

**Alternative one bridge per run (cleanest CSV-per-run boundary):**
```bash
for i in {1..15}; do
  echo "Starting calibration run $i of 15..."
  
  # start bridge node in background
  ros2 run ridescan_ros2_bridge ride_scan_csv_node &
  BRIDGE_PID=$!
  
  # run one mission
  ros2 run ridescan_ros2_bridge way_point_follower_node
  
  # kill bridge node destroy_node() flushes remaining rows to CSV
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
baseline dataset one complete perimeter inspection per run, captured as a
timestamped CSV.

---

## Mission Briefing

### What is the Mission?

The mission is a **Warehouse Perimeter Inspection** executed by Davie,
a simulated differential-drive mobile robot running on ROS 2 Humble and
Gazebo Sim. Starting from a fixed dock position, Davie navigates
autonomously through 5 predefined waypoints that trace the boundary of a
simulated warehouse environment, then returns to its origin.

The mission is executed entirely autonomously via the `way_point_follower_node`,
which sends each waypoint as a Nav2 `NavigateToPose` action goal, waits for
confirmed arrival, then proceeds to the next. No manual intervention is
required between waypoints. Each run is identical in route, speed, and
behavior producing a clean, repeatable telemetry baseline across all 15
calibration instances.

### Real-World Commercial Use Case

Warehouse perimeter inspection is one of the highest-frequency autonomous
robot deployments in operation today. In real-world facilities, robots patrol
boundaries, monitor access points, detect environmental anomalies, verify
asset placement, and flag unauthorized activity all without human
supervision, across multiple shifts, every single day.

The scale of this problem is significant:
- A single warehouse may run 50–200 inspection loops per day
- Robots operate unsupervised for hours at a time
- Hardware degradation is gradual and often invisible until failure
- A single missed anomaly can escalate into a mission failure, hardware
  loss, or a safety incident

Real-world deployments this mission maps directly to:

| Industry | Application |
|---|---|
| Warehouse automation | Amazon Robotics, Fetch Robotics, 6 River Systems |
| Facility security | Access point monitoring, perimeter patrol |
| Industrial inspection | Oil & gas plants, manufacturing floors |
| Healthcare | Hospital corridor patrol, asset tracking |
| Hospitality | Hotel and office campus delivery and monitoring |

### Mission Illustration Video
The following video demonstrates one complete execution of the Warehouse Perimeter Inspection mission in Gazebo Sim. Davie autonomously exits the dock, navigates through the five predefined perimeter waypoints using Nav2, and returns to its starting position without manual intervention.

The video illustrates the exact behavior used to generate the RideScan calibration baseline dataset. Each of the 15 calibration runs follows this same route and operational profile, allowing RideScan to learn the robot's normal behavioral envelope.

Video contents:
- Gazebo simulation environment
- Nav2-driven waypoint execution
- Davie's traversal through all five waypoints
- Return to dock
- Terminal output showing mission progress
- RideScan bridge node recording telemetry in parallel

Video file: [Watch the Warehouse Perimeter Inspection Demo/Demonstration video](https://youtu.be/x1DSrypx_-4)

---

## System Architecture

The following diagram illustrates the end-to-end flow of the Davie–RideScan
integration, from autonomous mission execution in simulation through
telemetry extraction, calibration, and future risk scoring.

```text
                    Gazebo Sim
                         │
                         ▼
                    Waypoint Follower
                         │
                         ▼
               Davie Executes Mission
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
     /odom            /scan           /cmd_vel
        │                │                │
        └────────────────┼────────────────┘
                         │
                         ▼
              ridescan_bridge_node
                         │
                         ▼
                Mission CSV Files
                  (15 Instances)
                         │
                         ▼
              RideScan Calibration
              (Baseline Learning)
                         │
                         ▼
               Future RISQ Scoring
               (Inference Phase)
```

During calibration, the telemetry collected from each mission execution is
persisted as a separate Mission Instance CSV. RideScan uses these 15 clean,
near-identical mission instances to learn the robot's normal behavioral
fingerprint. Once deployed, future mission runs can be compared against this
baseline to quantify operational risk and detect early signs of anomalous
behavior.



### How RideScan Monitors This Mission

RideScan acts as an independent safety and reliability layer a behavioral
health monitor that learns what a normal, healthy inspection run looks like
and flags any deviation as a quantified risk signal.

**Step 1 Telemetry Collection**

During every mission run, `ridescan_bridge_node` collects timestamped
telemetry from three ROS 2 topics:

| Signal | Topic | What It Captures |
|---|---|---|
| Odometry | `/odom` | Position, velocity, heading per timestep |
| Laser scan | `/scan` | Obstacle distances, environment geometry |
| Velocity commands | `/cmd_vel` | Motor commands, speed profile per segment |

Each run produces one CSV file one Mission Instance in RideScan's
architecture.

**Step 2 Calibration (Learning Normal Behavior)**

15 clean, sequential runs of the identical mission are collected under
consistent conditions. RideScan processes these 15 files to learn the
robot's normal behavioral envelope:
- Expected velocity profile between each waypoint
- Typical obstacle distances along the route
- Normal odometry progression and heading changes
- Baseline motor command patterns

---

### What the Calibration Files Do

Each CSV file is a complete behavioral record of one mission run. Together, the 15 files form the dataset RideScan uses to learn what normal looks like for this robot on this mission.

**What each file contains:**

Every row in a CSV is a timestamped telemetry message from one of three ROS 2
topics, captured in real time as Davie navigated the perimeter loop. A single
run produces hundreds of rows interleaving `odom`, `scan`, and `cmd_vel`
messages across the full mission duration.


**What RideScan learns from them:**

By processing all 15 files, RideScan builds a statistical model of normal behavior across every phase of the mission:

| Phase | What the files capture |
|---|---|
| Dock exit | Initial acceleration profile, heading establishment |
| Straight segments | Cruise velocity, obstacle clearance distances, heading stability |
| Waypoint turns | Angular velocity ramp-up and ramp-down signature, turn radius |
| Waypoint arrival | Deceleration profile, stop position accuracy |
| Return to dock | Full route odometry progression, cumulative heading change |

**Why 15 runs:**

A single run could be noise. Two or three runs could share a systematic bias. Fifteen runs gives RideScan enough samples to distinguish genuine behavioral patterns from run-to-run variation, producing a statistically robust baseline. Any future run that deviates meaningfully from this envelope will be flagged as a quantified risk signal rather than dismissed as natural variance.

### What constitutes one clean run

- Davie successfully navigates all 5 waypoints without aborting
- The bridge node is active for the full duration of the run
- No unexpected obstacles or environment changes during the run
- One CSV file is written per run on bridge shutdown


### Calibration Setup and Consistency

The 15 calibration runs in this dataset were collected under controlled,
deterministic conditions. This was a deliberate design decision to give
RideScan the cleanest possible baseline to learn from.

**What this means for RideScan:**

Because every run follows the same route from the same starting position in
the same environment, the behavioral variation between runs is minimal
limited only to minor floating-point differences in how Nav2 executes the
path at runtime.

RideScan does not have to account for algorithmic randomness or shifting
starting conditions when building the baseline.

The result is a tight, precise behavioral fingerprint rather than a wide,
averaged envelope.

Each of the three telemetry signals tells nearly the same story across all
15 runs:

| Signal | What stays consistent across runs |
|---|---|
| `/odom` | Position progression, velocity profile, heading changes at each waypoint |
| `/scan` | Obstacle distances at each route segment, environment geometry |
| `/cmd_vel` | Motor command patterns, acceleration and deceleration profiles, turn signatures |

This consistency is what makes the calibration baseline reliable. When
RideScan flags a future run as anomalous, it is comparing against a baseline
built from runs that were as close to identical as simulation allows not a
baseline built from runs that were each slightly different by design.

---

---

# Stage 3 — Live API Integration & Autonomous Safety Response

## Overview

Stage 3 transforms the calibration pipeline built in Stage 2 into a fully
operational, real-time safety system. Where Stage 2 produced the behavioral
baseline, Stage 3 puts that baseline to work: live telemetry from Davie's
ongoing missions is streamed to RideScan's inference API, risk scores are
returned in real time, and the robot responds autonomously to any score that
breaches the critical threshold.

The result is a closed-loop autonomous safety system:

```text
Robot moves → Telemetry streams → RideScan scores risk →
Score breaches threshold → Safety stop triggers → Human operator alerted via SMS
```

This is not a logging pipeline. Every component in Stage 3 closes a real
operational loop......the robot makes autonomous decisions based on live API
intelligence, and a human is notified the moment something goes wrong.

---

## Stage 3 Architecture

```text
                        Gazebo Sim
                             │
                             ▼
                        Waypoint Follower
                             │
                             ▼
                   Davie Executes Mission
                             │
               ┌─────────────┼─────────────┐
               │             │             │
               ▼             ▼             ▼
            /odom         /scan        /cmd_vel
               │             │             │
               └─────────────┼─────────────┘
                             │
                             ▼
              ridescan_safety_monitor_node
              (buffers telemetry → CSV batch)
                             │
                             ▼
                  RideScan Inference API
                  (process_file endpoint)
                             │
                             ▼
                      Risk Score Returned
                             │
               ┌─────────────┼─────────────┐
               │             │             │
               ▼             ▼             ▼
        /ridescan/      /ridescan/     Africa's Talking
        safety_stop     risk_score      SMS Alert
               │
               ▼
     way_point_follower_node
     (halts robot on True)
```

---

## Stage 3 Nodes

### 1. `ridescan_safety_monitor_node`

The core of the Stage 3 integration. This node buffers live odometry into
rolling CSV batches, uploads each batch to RideScan's `process_file`
endpoint, triggers inference, and publishes a safety stop signal if the
returned risk score exceeds the configured threshold.

**Responsibilities:**
- Subscribes to `/odom` and buffers telemetry rows in memory
- Computes linear acceleration via finite difference of consecutive velocity
  readings, approximating IMU-derived acceleration without requiring a
  physical IMU topic
- Every `batch_seconds` (default: 30s), writes the buffer to a temporary CSV
  and uploads it to RideScan
- Calls the inference endpoint and polls until a risk score is returned
- Publishes the risk score to `/ridescan/risk_score` (Float32)
- If `risk_score >= risk_threshold`, publishes `True` to `/ridescan/safety_stop` (Bool)
- Fires an SMS alert via Africa's Talking on both safety stop and recovery events
- Handles 502 gateway errors gracefully..... verifies whether the file actually
  landed on the server before treating the upload as a genuine failure
- Guards against overlapping batch cycles with a `_processing` flag

**Key parameters:**

| Parameter | Default | Description |
|---|---|---|
| `api_key` | `$RIDESCAN_API_KEY` | RideScan API key |
| `robot_id` | `e.......` | Registered robot UUID |
| `mission_id` | `5.......` | Active mission UUID |
| `robot_type` | `wheeled_mobile` | Robot classification |
| `batch_seconds` | `30.0` | Telemetry batch interval |
| `risk_threshold` | `40.0` | Safety stop trigger level |

**Topics published:**

| Topic | Type | Description |
|---|---|---|
| `/ridescan/safety_stop` | `std_msgs/Bool` | True when risk exceeds threshold |
| `/ridescan/risk_score` | `std_msgs/Float32` | Latest batch risk score |

**Topics subscribed:**

| Topic | Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Robot pose and velocity |

---

### 2. `way_point_follower_node` (Stage 3 extension)

The mission execution layer, extended in Stage 3 to subscribe to
`/ridescan/safety_stop`. When the safety monitor publishes `True`, the
waypoint follower immediately halts the robot by publishing a zero-velocity
`Twist` to `/cmd_vel` and suspends further waypoint navigation until the
stop is cleared.

This is the mechanism that closes the loop... the API's risk assessment
directly controls whether the robot continues its mission.

**Safety stop behavior:**
- Receives `Bool` on `/ridescan/safety_stop`
- On `True`: publishes zero `Twist` to `/cmd_vel`, pauses waypoint execution
- On `False` (recovery): resumes mission from current waypoint
- Logs all stop and resume events with the associated risk score

---

### 3. `odom_live_plot_path` (Visualisation Node)

A real-time matplotlib visualiser that renders Davie's path as the mission
executes. Anomaly events from the safety monitor are overlaid as red diamond
markers at the exact coordinates where the risk score exceeded the threshold.

**What the plot shows:**

| Element | Description |
|---|---|
| Blue line | Live robot trajectory |
| Red dot | Current robot position (updates at 100ms) |
| Green triangle | Mission start position |
| Black × | Predefined waypoints |
| Red diamonds | Anomaly positions (risk ≥ threshold) |
| Title bar | Live sample count, total distance, anomaly count |

The node uses `rclpy.spin_once()` inside matplotlib's `FuncAnimation`
callback, allowing ROS callbacks and the GUI to share a single thread
without blocking.

**Topics subscribed:**

| Topic | Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Robot position for path rendering |
| `/ridescan/risk_assessment` | `std_msgs/String` | Anomaly events for overlay markers |

---

## SMS Alerting — Africa's Talking Integration

Stage 3 integrates Africa's Talking as the SMS alerting provider. When the
safety monitor detects a risk score above threshold, an SMS is dispatched
immediately to the configured operator number. A second SMS is sent when
the risk score drops back below threshold and the robot resumes.

**Alert messages:**

| Event | SMS Content |
|---|---|
| Safety stop triggered | `RideScan ALERT: Davie-Perimeter-Bot safety stop triggered. Risk score {score} (threshold {threshold}).` |
| Robot resumed | `RideScan: Davie-Perimeter-Bot resumed. Risk score {score} back below threshold.` |

**Setup:**

```bash
pip install africastalking --break-system-packages
```

```bash
export AT_USERNAME=sandbox          # use 'sandbox' for testing
export AT_API_KEY=your_key_here
export AT_TO_NUMBER=phoneNumber
```

The SMS integration is non-blocking , a failure to send does not interrupt
the safety stop logic or the mission. Errors are logged to the ROS console
only.

---

## Environment Variables

All credentials are loaded from environment variables. Never hardcode keys.

```bash
# RideScan
export RIDESCAN_API_KEY=ridescan_api_key

# Africa's Talking
export AT_USERNAME=sandbox
export AT_API_KEY=africastalking_api_key
export AT_TO_NUMBER=phoneNumber
```

Add these to `~/.bashrc` for persistence:

```bash
echo 'export RIDESCAN_API_KEY=your_key' >> ~/.bashrc
echo 'export AT_USERNAME=sandbox' >> ~/.bashrc
echo 'export AT_API_KEY=your_key' >> ~/.bashrc
echo 'export AT_TO_NUMBER=+2349033429138' >> ~/.bashrc
source ~/.bashrc
```

---

## Running the Full Stage 3 Stack

Four terminals are required for a complete Stage 3 run:

**Terminal 1 — Gazebo simulation:**
```bash
ros2 launch robot gazebo_sim.launch.py
```

**Terminal 2 — Safety monitor (start first, before the mission):**
```bash
ros2 run ridescan_ros2_bridge ridescan_safety_monitor_node
```

**Terminal 3 — Waypoint follower (mission execution):**
```bash
ros2 run ridescan_ros2_bridge way_point_follower_node
```

**Terminal 4 — Live path visualiser (optional but recommended):**
```bash
ros2 run ridescan_ros2_bridge odom_plotter_node
```

**Expected sequence of events:**
1. Safety monitor starts and begins buffering odometry
2. Waypoint follower sends Davie through the perimeter loop
3. Every 30 seconds, a telemetry batch is uploaded to RideScan
4. RideScan returns a risk score
5. Score is published to `/ridescan/risk_score` and visible in the plotter
6. If score ≥ threshold, `True` is published to `/ridescan/safety_stop`
7. Waypoint follower halts the robot
8. SMS alert fires to the operator number
9. Risk score and anomaly position are logged and rendered on the plotter
10. Dashboard at `hackathon.ridescan.cloud` updates with the new execution cycle

---

## RideScan Dashboard Results

The Warehouse-Perimeter-Inspection mission is registered and actively
monitored at `hackathon.ridescan.cloud` under the Hackathon workspace.

**Mission registration:**

| Field | Value |
|---|---|
| Mission name | Warehouse-Perimeter-Inspection |
| Robot name | Davie_Perimeter_Bot |
| Robot type | Wheeled Mobile Robot |
| Calibration possible | ✅ True |
| Inference possible | ✅ True |

**What the dashboard shows:**

**Multi-Mission Risk Comparison graph**

The risk score curve for Warehouse-Perimeter-Inspection starts at
approximately 20 at execution cycle 0 and climbs steadily, crossing
RideScan's Critical Threshold of 50 at cycle 1 and peaking near 100 by
cycle 5. The red dotted critical threshold line is drawn by RideScan's
own dashboard — not a local configuration — confirming that the risk
events recorded are genuine API-scored anomalies, not locally simulated
values.

**Multi-Robot Risk Heatmap**

| Date | Risk Level | Activity |
|---|---|---|
| 07/07/2026 | Low (green) | First live missions, initial telemetry streaming |
| 07/08/2026 | High (orange/red) | Risk threshold breached, safety stops triggered |

**Execution Volume**

Multiple execution cycles recorded across both days, all processed through
RideScan's live inference endpoint. Each cycle represents one complete
telemetry batch uploaded, scored, and acted upon.

---

## Closed Loop Summary

The defining characteristic of this Stage 3 integration is that the risk
score is not merely logged — it controls the robot.

| Layer | Component | Role |
|---|---|---|
| Sensing | `/odom` subscription | Captures robot state at every timestep |
| Processing | `ridescan_safety_monitor_node` | Batches, uploads, and scores telemetry |
| Intelligence | RideScan Inference API | Returns risk score based on calibrated baseline |
| Decision | Safety stop publisher | Converts score into a binary stop/go signal |
| Actuation | `way_point_follower_node` | Halts robot when stop signal is True |
| Alerting | Africa's Talking SMS | Notifies human operator immediately |
| Visualisation | `odom_live_plot_path` | Renders path and anomaly positions in real time |
| Monitoring | RideScan dashboard | Records all execution cycles for evaluation |

Every layer is connected. A risk event detected by the API propagates
through the system in under one batch cycle (30 seconds), stopping the
robot autonomously and alerting a human operator — without any manual
intervention required.

---

## Stage 3 Demonstration Video

*Video link to be added prior to final submission.*

The demonstration video will show:
- Full Gazebo simulation environment with Davie executing the perimeter loop
- Safety monitor node terminal showing batch uploads and risk scores returned
- Risk score climbing above the Critical Threshold (50) and safety stop triggering
- Robot halting mid-mission in response to the API response
- Africa's Talking SMS alert firing on stop and recovery
- Live path plotter with red anomaly diamond overlaid at the stop position
- RideScan dashboard updating with the new execution cycle in real time

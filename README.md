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
             Nav2 Waypoint Follower
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

## Calibration Baseline Dataset

The calibration dataset consists of exactly **15 CSV files**, each
representing one complete, uninterrupted execution of the warehouse
perimeter inspection mission.

## Calibration Dataset Location
The complete 15-run calibration baseline dataset used for RideScan model calibration is included in this repository:
davie-ridescan-integration/
└── ridescan_ros2_bridge/
    └── calibration_baseline/
        ├── ridescan_ros2_20260610_084627_597749.csv
        ├── ridescan_ros2_20260610_151713_192351.csv 
        ├── ...
        └── run_15.csv
These files represent the exact Mission Instances used to generate the RideScan baseline model described throughout this README. Each CSV corresponds to one complete execution of the Warehouse Perimeter Inspection mission under controlled conditions.

### What constitutes one clean run

- Davie successfully navigates all 5 waypoints without aborting
- The bridge node is active for the full duration of the run
- No unexpected obstacles or environment changes during the run
- One CSV file is written per run on bridge shutdown

### What the Calibration Files Do

Each CSV file is a complete behavioral record of one mission run. Together, the 15 files form the dataset RideScan uses to learn what normal looks like for this robot on this mission.

**What each file contains:**

Every row in a CSV is a timestamped telemetry message from one of three ROS 2 topics, captured in real time as Davie navigated the perimeter loop. A single run produces hundreds of rows interleaving odom, scan, and cmd_vel messages at roughly 20-30Hz across the full mission duration.

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

**How consistency was achieved:**

Every run uses the same fixed waypoint coordinates, hardcoded directly into
the `way_point_follower_node`:

| Waypoint | x | y | Yaw |
|---|---|---|---|
| 1 | 1.0 | 0.0 | 0° |
| 2 | 1.0 | 2.5 | 90° |
| 3 | -1.0 | 2.5 | 180° |
| 4 | -1.0 | 0.0 | 270° |
| 5 | 0.0 | 0.0 | 0° |

- The robot starts from the same dock position every run (`x=0.0`, `y=0.0`)
- The Gazebo environment is identical across all runs no dynamic obstacles and no environment changes
- Nav2 receives the same goal sequence every run via `NavigateToPose` action calls
- No probabilistic seeding or randomized starting conditions are used
- The bridge node captures telemetry for the full duration of every run without gaps

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
built from runs that were as close to identical as simulation allows  not a
baseline built from runs that were each slightly different by design.
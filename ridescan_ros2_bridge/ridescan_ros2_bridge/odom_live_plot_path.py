#!/usr/bin/env python3
"""
odom_live_plot_path.py

Real-time path plotter subscribes to /odom and draws the robot's
trajectory live using matplotlib. Anomaly events from the RideScan
API bridge are overlaid as red warning markers on the path.

Run alongside your waypoint controller and API bridge node:
    ros2 run ridescan_ros2_bridge odom_plotter_node

Dependencies:
    pip install matplotlib --break-system-packages

Author: Davies Iyanuoluwa Ogunsina
"""

import re

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches


WAYPOINTS = [
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),
]

PLOT_PADDING  = 0.5    # metres of padding around the trajectory
UPDATE_MS     = 100    # animation refresh interval in milliseconds
RISK_THRESHOLD = 40.0   # flag positions above this score as anomalies


class OdomPlotterNode(Node):

    def __init__(self):
        super().__init__("odom_plotter_node")

        # --- path history ---
        self.xs: list[float] = []
        self.ys: list[float] = []

        # --- anomaly events: list of (x, y, risk_score, label) ---
        self.anomaly_events: list[tuple[float, float, float, str]] = []

        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(
            String, "/ridescan/risk_score", self.risk_cb, 10
        )

        self.get_logger().info(
            "Odom plotter started ... waiting for /odom and /ridescan/risk_score..."
        )

    def odom_cb(self, msg: Odometry) -> None:
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)

    def risk_cb(self, msg: String) -> None:
        """
        Parses the String published by RideScanAPIBridgeNode.handle_response().
        Expected format:  "risk_score=0.85 anomalies=['SPEED', 'TILT']"
        Records the robot's current position when risk exceeds the threshold.
        """
        text = msg.data

        # Pull out risk score
        score_match = re.search(r"risk_score=([\d.]+)", text)
        if not score_match:
            return
        risk_score = float(score_match.group(1))

        # Only mark positions that are actually anomalous
        if risk_score < RISK_THRESHOLD:
            return

        # Grab current robot position (may be empty at startup)
        if not self.xs:
            return

        x, y = self.xs[-1], self.ys[-1]

        # Pull anomaly flag labels for the tooltip
        flags_match = re.search(r"anomalies=(\[.*?\])", text)
        label = flags_match.group(1) if flags_match else "anomaly"

        self.anomaly_events.append((x, y, risk_score, label))
        self.get_logger().warn(
            f"Anomaly recorded at ({x:.2f}, {y:.2f})  score={risk_score}  flags={label}"
        )


def main() -> None:
    rclpy.init()
    node = OdomPlotterNode()

    # figure setup 
    fig, ax = plt.subplots(figsize=(6, 6))
    fig.canvas.manager.set_window_title("RideScan Live Path Plotter")

    (path_line,) = ax.plot([], [], "b-",  linewidth=1.5, label="Path")
    (robot_dot,) = ax.plot([], [], "ro",  markersize=8,  label="Robot")
    (start_dot,) = ax.plot([], [], "g^",  markersize=10, label="Start")
    ax.plot(
        [w[0] for w in WAYPOINTS],
        [w[1] for w in WAYPOINTS],
        "kx", markersize=10, markeredgewidth=2, label="Waypoints",
    )

    # Anomaly scatter populated dynamically
    anomaly_scatter = ax.scatter(
        [], [], c="red", s=120, zorder=5,
        marker="D", label=f"Anomaly (risk ≥ {RISK_THRESHOLD})"
    )

    for i, (wx, wy) in enumerate(WAYPOINTS):
        ax.annotate(
            f"WP{i}", (wx, wy),
            textcoords="offset points", xytext=(6, 6),
            fontsize=8, color="gray",
        )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Live Robot Path")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--", alpha=0.4)

    # Anomaly count badge in corner
    anomaly_text = ax.text(
        0.02, 0.97, "Anomalies: 0",
        transform=ax.transAxes,
        fontsize=9, verticalalignment="top",
        color="red",
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="red", alpha=0.8),
    )

    # annotation tooltip (shown on anomaly hover) 
    annot = ax.annotate(
        "", xy=(0, 0), xytext=(10, 10),
        textcoords="offset points",
        bbox=dict(boxstyle="round", fc="w", ec="red"),
        arrowprops=dict(arrowstyle="->"),
        fontsize=7, visible=False,
    )

    # animation callback 
    def update(_frame):
        rclpy.spin_once(node, timeout_sec=0.0)

        xs, ys = node.xs, node.ys
        if not xs:
            return path_line, robot_dot, start_dot, anomaly_scatter, anomaly_text

        path_line.set_data(xs, ys)
        robot_dot.set_data([xs[-1]], [ys[-1]])
        start_dot.set_data([xs[0]],  [ys[0]])

        # Update anomaly markers
        events = node.anomaly_events
        if events:
            ax_coords = [[e[0], e[1]] for e in events]
            anomaly_scatter.set_offsets(ax_coords)
            # Colour-code by risk score (higher = darker red)
            anomaly_scatter.set_array(
                [e[2] for e in events]
            )
            anomaly_scatter.set_clim(RISK_THRESHOLD, 1.0)
            anomaly_text.set_text(f"Anomalies: {len(events)}")

        # Auto-scale
        all_x = xs + [w[0] for w in WAYPOINTS]
        all_y = ys + [w[1] for w in WAYPOINTS]
        ax.set_xlim(min(all_x) - PLOT_PADDING, max(all_x) + PLOT_PADDING)
        ax.set_ylim(min(all_y) - PLOT_PADDING, max(all_y) + PLOT_PADDING)

        dist = sum(
            ((xs[i] - xs[i-1])**2 + (ys[i] - ys[i-1])**2) ** 0.5
            for i in range(1, len(xs))
        )
        ax.set_title(
            f"Live Robot Path  |  samples={len(xs)}  "
            f"dist={dist:.2f} m  anomalies={len(events)}"
        )

        return path_line, robot_dot, start_dot, anomaly_scatter, anomaly_text

    ani = animation.FuncAnimation(          # noqa: F841
        fig, update,
        interval=UPDATE_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
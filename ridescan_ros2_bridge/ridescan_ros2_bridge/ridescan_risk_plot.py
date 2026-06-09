#!/usr/bin/env python3

"""
ridescan_risk_plot.py

Real-time risk score visualiser.
Subscribes to /ridescan/risk_score and plots it live with
colour-coded danger zones matching the safety node thresholds.

Run alongside the diagnostics and safety nodes:
  ros2 run ridescan_ros2_bridge ridescan_risk_plot
  
  Author: Davies Iyanuoluwa Ogunsina
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

WINDOW          = 60    # samples of history to display
THRESHOLD_WARN  = 0.3
THRESHOLD_STOP  = 0.7


class RiskPlotNode(Node):

    def __init__(self):
        super().__init__('ridescan_risk_plot')
        self._scores = deque(maxlen=WINDOW)
        self._times  = deque(maxlen=WINDOW)
        self._t      = 0

        self.create_subscription(
            Float32, '/ridescan/risk_score', self._cb, 10
        )
        self.get_logger().info('RideScan risk plot online — waiting for data...')

    def _cb(self, msg: Float32):
        self._scores.append(msg.data)
        self._times.append(self._t)
        self._t += 1


def main():
    rclpy.init()
    node = RiskPlotNode()

    fig, ax = plt.subplots(figsize=(9, 3))
    fig.patch.set_facecolor('#0f1117')
    ax.set_facecolor('#0f1117')

    # Colour zones 
    ax.axhspan(0.0,             THRESHOLD_WARN, alpha=0.08, color='green')
    ax.axhspan(THRESHOLD_WARN,  THRESHOLD_STOP, alpha=0.12, color='orange')
    ax.axhspan(THRESHOLD_STOP,  1.0,            alpha=0.15, color='red')

    #  Threshold lines 
    ax.axhline(THRESHOLD_WARN, color='orange', linestyle='--',
               linewidth=1.0, label='WARN  0.3')
    ax.axhline(THRESHOLD_STOP, color='red',    linestyle='--',
               linewidth=1.0, label='STOP  0.7')

    # Live line 
    line, = ax.plot([], [], lw=2, color='royalblue', label='Risk score')

    # Styling 
    ax.set_ylim(0.0, 1.0)
    ax.set_xlim(0, WINDOW)
    ax.set_title('RideScan — Live Risk Score', color='white', fontsize=13, pad=10)
    ax.set_ylabel('Risk Score', color='white')
    ax.set_xlabel('Samples', color='white')
    ax.tick_params(colors='white')
    ax.spines[:].set_color('#333333')

    legend = ax.legend(loc='upper right', facecolor='#1a1a2e', labelcolor='white')

    # Zone labels 
    ax.text(1, 0.15,           'NOMINAL',  color='green',  alpha=0.7, fontsize=8)
    ax.text(1, THRESHOLD_WARN + 0.05, 'WARN',    color='orange', alpha=0.9, fontsize=8)
    ax.text(1, THRESHOLD_STOP + 0.05, 'DANGER',  color='red',    alpha=0.9, fontsize=8)

    def update(_):
        rclpy.spin_once(node, timeout_sec=0)

        if not node._scores:
            return line,

        xs = list(node._times)
        ys = list(node._scores)

        line.set_data(xs, ys)
        ax.set_xlim(max(0, node._t - WINDOW), max(WINDOW, node._t))

        # Colour the live line by current risk level
        current = ys[-1]
        if current >= THRESHOLD_STOP:
            line.set_color('red')
        elif current >= THRESHOLD_WARN:
            line.set_color('orange')
        else:
            line.set_color('royalblue')

        return line,

    ani = animation.FuncAnimation(
        fig, update, interval=500, blit=True, cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
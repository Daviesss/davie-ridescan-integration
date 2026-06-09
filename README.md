# davie-ridescan-integration
A production-grade ROS 2 integration for the RideScan Safety Layer API: telemetry bridge, risk diagnostics, and autonomous safety response for mobile robots.

## Overview

This repository contains three ROS 2 nodes that connect any mobile robot
running ROS 2 to the RideScan Safety Layer API. The integration collects
robot telemetry, uploads it to RideScan for risk analysis, and feeds the
resulting risk score back into the robot to trigger autonomous safety
responses.
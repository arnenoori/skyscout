# Navigation Bridge

MAVLink/PX4 interface for low-level drone control.

## Overview

Bridges between ROS2 mission commands and PX4 flight controller via MAVLink protocol.

## Topics

**Subscribers:**
- `/navigation/waypoint` (geometry_msgs/Point) - Target positions
- `/navigation/arm` (std_msgs/Bool) - Arm/disarm commands

**Publishers:**
- `/navigation/current_position` (geometry_msgs/Point) - Drone position
- `/navigation/battery_level` (std_msgs/Float32) - Battery percentage
- `/navigation/status` (std_msgs/String) - Flight status

## Parameters

- `connection_string` - MAVLink connection (default: "udp://:14540")
- `system_id` - MAVLink system ID (default: 1)

## Safety Features

- Automatic RTL on low battery (<20%)
- Geofence enforcement
- RC override priority

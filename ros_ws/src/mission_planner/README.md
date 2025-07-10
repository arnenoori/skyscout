# Mission Planner

Executes structured mission plans using a state machine approach.

## Overview

Converts JSON mission plans into coordinated drone behaviors, managing states like planning, executing, and landing.

## States

- IDLE - Waiting for mission
- PLANNING - Generating waypoints
- VALIDATING - Safety checks
- ARMING - Preparing for flight
- TAKEOFF - Ascending to altitude
- EXECUTING - Following mission plan
- LANDING - Returning to ground
- EMERGENCY_LAND - Safety response

## Topics

**Subscribers:**
- `/mission_plan` (std_msgs/String) - JSON mission plans
- `/detected_objects` (vision_msgs/Detection2DArray) - Object detections

**Publishers:**
- `/navigation/waypoint` (geometry_msgs/Point) - Waypoints for navigation
- `/mission_status` (std_msgs/String) - Mission state updates

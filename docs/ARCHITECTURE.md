# Architecture

SkyScout is organized as a small ROS2 graph plus a Next.js dashboard.

```text
web_frontend
  -> rosbridge websocket
  -> command_interface
  -> llm_agent
  -> mission_planner
  -> navigation_bridge
  -> MockDrone
```

The perception node publishes detections to `mission_planner`. The navigation bridge publishes telemetry back to the dashboard.

## Packages

`command_interface`

- Receives `/web_command`
- Routes `SIMULATE:*` commands to `/navigation/simulate`
- Routes mission commands to `/natural_language_command`

`llm_agent`

- Converts natural-language commands into mission-plan JSON
- Uses OpenAI or Gemini when configured
- Publishes `/mission_plan`

`mission_planner`

- Validates mission structure
- Expands supported flight patterns into waypoints
- Publishes navigation commands and mission status

`navigation_bridge`

- Owns the drone interface
- Uses `MockDrone` by default
- Publishes position, battery, armed state, and status

`perception`

- Placeholder detection node
- Currently emits mock detections
- Intended integration point for YOLO or another detector

`web_frontend`

- Next.js dashboard
- Sends natural-language commands
- Displays connection state, telemetry, mission state, command history, mock controls, and map position

## Topic Contracts

Inputs:

- `/web_command`: `std_msgs/String`
- `/natural_language_command`: `std_msgs/String`
- `/mission_plan`: `std_msgs/String` containing mission JSON
- `/navigation/waypoint`: `geometry_msgs/Point`
- `/navigation/arm`: `std_msgs/Bool`
- `/navigation/takeoff`: `std_msgs/Float32`
- `/navigation/land`: `std_msgs/Bool`
- `/navigation/rtl`: `std_msgs/Bool`
- `/navigation/simulate`: `std_msgs/String`

Outputs:

- `/mission_status`: `std_msgs/String` containing status JSON
- `/navigation/current_position`: `geometry_msgs/Point`
- `/navigation/battery_level`: `std_msgs/Float32`
- `/navigation/armed`: `std_msgs/Bool`
- `/navigation/status`: `std_msgs/String` containing status JSON
- `/detected_objects`: `vision_msgs/Detection2DArray`

## Current Boundary

This repository models the system shape and simulated control flow. Hardware flight support, production perception, authenticated command channels, and full mission safety policy are future work.

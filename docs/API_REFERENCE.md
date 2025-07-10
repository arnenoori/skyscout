# API Reference

Quick reference for SkyScout's ROS2 interfaces and web APIs.

## ROS2 Topics

### Command Interface
- `/web_command` (std_msgs/String) - Commands from web UI
- `/natural_language_command` (std_msgs/String) - Validated commands to LLM
- `/emergency_stop` (std_msgs/Bool) - Emergency stop trigger

### Mission Planning
- `/mission_plan` (std_msgs/String) - JSON mission plans from LLM
- `/mission_status` (std_msgs/String) - Current mission state and progress

### Perception
- `/camera/image_raw` (sensor_msgs/Image) - Raw camera feed
- `/detected_objects` (vision_msgs/Detection2DArray) - Detected objects with bounding boxes

### Navigation
- `/navigation/waypoint` (geometry_msgs/Point) - Target waypoints
- `/navigation/current_position` (geometry_msgs/Point) - Drone position
- `/navigation/battery_level` (std_msgs/Float32) - Battery percentage
- `/navigation/status` (std_msgs/String) - Flight status updates

## Mission Plan Schema

```json
{
  "mission_type": "search|inspect|count|map",
  "target_description": "string",
  "flight_pattern": "grid|spiral|perimeter|waypoints",
  "parameters": {
    "altitude": 10-120,
    "speed": 1-10,
    "coverage_area": {
      "center": {"lat": 0.0, "lon": 0.0},
      "radius": 50-500
    }
  },
  "safety": {
    "geofence": {
      "max_altitude": 120,
      "max_distance": 500
    },
    "rtl_battery_threshold": 20
  }
}
```

## Web API Endpoints

- `GET /api/health` - System health check
- `POST /api/command` - Submit natural language command
- `GET /api/mission/status` - Current mission status
- `POST /api/mission/cancel` - Cancel active mission
- `GET /ws` - WebSocket connection for real-time updates

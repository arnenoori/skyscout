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
- `/mission_template` (std_msgs/String) - Available mission templates

### Perception
- `/camera/image_raw` (sensor_msgs/Image) - Raw camera feed
- `/detected_objects` (vision_msgs/Detection2DArray) - Detected objects with bounding boxes

### Navigation
- `/navigation/waypoint` (geometry_msgs/Point) - Target waypoints
- `/navigation/current_position` (geometry_msgs/Point) - Drone position
- `/navigation/battery_level` (std_msgs/Float32) - Battery percentage
- `/navigation/status` (std_msgs/String) - Flight status updates
- `/navigation/arm` (std_msgs/Bool) - Arm/disarm command
- `/navigation/takeoff` (std_msgs/Float32) - Takeoff to altitude
- `/navigation/land` (std_msgs/Bool) - Land command
- `/navigation/rtl` (std_msgs/Bool) - Return to launch

### Weather & Safety
- `/weather/current` (std_msgs/String) - Current weather conditions
- `/safety/geofence_violation` (std_msgs/Bool) - Geofence breach alert
- `/safety/battery_warning` (std_msgs/Bool) - Low battery warning

## ROS2 Services

### System Control
- `/set_connection_mode` (std_srvs/SetBool) - Toggle mock/real drone mode
- `/get_system_status` (custom_msgs/GetSystemStatus) - Get system health
- `/cancel_mission` (std_srvs/Trigger) - Cancel active mission

### Mission Services
- `/get_mission_templates` (custom_msgs/GetTemplates) - List available templates
- `/validate_mission` (custom_msgs/ValidateMission) - Pre-flight validation

## Mission Plan Schema

```json
{
  "mission_type": "search|inspect|count|map|delivery|patrol|survey|emergency|follow",
  "target_description": "string - what to look for or do",
  "flight_pattern": "grid|spiral|perimeter|waypoints|zigzag|circle|polygon",
  "parameters": {
    "altitude": 10-120,  // meters
    "speed": 1-10,       // m/s
    "coverage_area": {
      "center": {"lat": 0.0, "lon": 0.0},
      "radius": 50-500,  // meters
      "vertices": [      // optional, for polygon pattern
        {"x": 0, "y": 0},
        {"x": 100, "y": 0},
        {"x": 100, "y": 100},
        {"x": 0, "y": 100}
      ]
    }
  },
  "safety": {
    "geofence": {
      "max_altitude": 120,     // meters
      "max_distance": 500      // meters from home
    },
    "rtl_battery_threshold": 20,  // percentage
    "weather_check": true,         // enable weather safety
    "max_wind_speed": 10.0        // m/s
  }
}
```

## Mission Types

| Type | Description | Typical Pattern | Use Case |
|------|-------------|----------------|----------|
| search | Find objects/people | grid, spiral, zigzag | Missing person, lost items |
| inspect | Detailed examination | circle, perimeter | Infrastructure check |
| count | Enumerate objects | grid, zigzag | Inventory, crowd size |
| map | Area overview | grid, polygon | Terrain survey |
| delivery | Transport items | waypoints | Package delivery |
| patrol | Security monitoring | perimeter, polygon | Boundary watch |
| survey | Agricultural analysis | zigzag, grid | Crop health |
| emergency | Rapid response | waypoints | Medical, disaster |
| follow | Track target | dynamic | Moving object |

## Flight Patterns

| Pattern | Description | Parameters | Best For |
|---------|-------------|------------|----------|
| grid | Back-and-forth rows | spacing | Systematic coverage |
| spiral | Expanding from center | turns | Uncertain location |
| perimeter | Boundary following | offset | Edge inspection |
| waypoints | Point-to-point | coordinates | Known locations |
| zigzag | Tight coverage | spacing | Large areas |
| circle | Orbit target | radius | 360° inspection |
| polygon | Custom shape | vertices | Irregular areas |

## Web API Endpoints

### Health & Status
- `GET /api/health` - System health check
- `GET /api/status` - Detailed system status
- `GET /api/weather` - Current weather conditions

### Mission Control
- `POST /api/command` - Submit natural language command
- `GET /api/mission/status` - Current mission status
- `POST /api/mission/cancel` - Cancel active mission
- `GET /api/mission/templates` - List available templates
- `POST /api/mission/validate` - Pre-validate mission plan

### Real-time Communication
- `GET /ws` - WebSocket connection for real-time updates

## WebSocket Events

### Client → Server
```javascript
// Send command
{
  "type": "command",
  "data": {
    "text": "Find all red cars",
    "priority": "normal|high|emergency"
  }
}

// Emergency stop
{
  "type": "emergency_stop"
}

// Request status
{
  "type": "status_request"
}
```

### Server → Client
```javascript
// Mission update
{
  "type": "mission_update",
  "data": {
    "state": "PLANNING|EXECUTING|COMPLETED",
    "progress": 0.75,
    "message": "Searching sector 3 of 4"
  }
}

// Detection event
{
  "type": "detection",
  "data": {
    "object": "red car",
    "confidence": 0.92,
    "location": {"x": 10.5, "y": -20.3},
    "timestamp": "2024-01-15T10:30:00Z"
  }
}

// Telemetry update
{
  "type": "telemetry",
  "data": {
    "position": {"x": 0, "y": 0, "z": 50},
    "battery": 85,
    "gps_fix": "3D",
    "flight_mode": "AUTO"
  }
}

// Weather alert
{
  "type": "weather_alert",
  "data": {
    "wind_speed": 12.5,
    "safe_to_fly": false,
    "message": "Wind speed exceeds safety limit"
  }
}
```

## Error Codes

| Code | Description | Recovery Action |
|------|-------------|-----------------|
| 1001 | Invalid command format | Check command syntax |
| 1002 | LLM API error | Retry or use fallback |
| 2001 | Mission validation failed | Adjust parameters |
| 2002 | Weather unsafe | Wait or override |
| 3001 | GPS signal lost | RTL activated |
| 3002 | Low battery | RTL activated |
| 3003 | Geofence violation | Return to safe area |
| 4001 | Camera failure | Mission abort |
| 4002 | Object detection error | Continue with warning |

## Configuration Files

### Mission Templates (`mission_templates.yaml`)
```yaml
quick_search:
  mission_type: search
  flight_pattern: spiral
  altitude: 50
  speed: 8
  radius: 200
  rtl_battery: 30

building_inspection:
  mission_type: inspect
  flight_pattern: circle
  altitude: 30
  speed: 3
  radius: 50
  rtl_battery: 25
```

### Safety Parameters (`safety_config.yaml`)
```yaml
weather:
  max_wind_speed: 10.0  # m/s
  min_visibility: 3.0   # km
  max_rain: 2.5        # mm/hour

battery:
  critical_level: 15   # %
  warning_level: 25    # %
  rtl_threshold: 20    # %

geofence:
  max_altitude: 120    # meters
  max_distance: 500    # meters
  home_radius: 5       # meters
```

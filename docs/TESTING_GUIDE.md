# Testing Guide for SkyScout

This guide will help you test the complete SkyScout system with the mock drone.

## Prerequisites

1. Ensure you have the following installed:
   - ROS2 Iron
   - rosbridge_suite: `sudo apt install ros-iron-rosbridge-suite`
   - Node.js 18+
   - Set API keys: `export OPENAI_API_KEY="your-key"` or `export GEMINI_API_KEY="your-key"`

## Quick Start

From the project root directory:

```bash
./launch_system.sh
```

This will start:
1. All ROS2 nodes (including mock drone)
2. rosbridge WebSocket server
3. Web development server

## Testing the System

### 1. Access the Web Interface

Open your browser to: http://localhost:3000

### 2. Check Connection Status

- Look for "Connected" status in the telemetry display
- The command input should be enabled

### 3. Send Test Commands

Try these natural language commands:

```
"Take off and hover at 20 meters"
"Search for red cars in a 100 meter radius"
"Fly in a grid pattern looking for people"
"Inspect the building to the north"
"Return to home and land"
```

### 4. Monitor the System

Watch for:
- **Mission Status**: Shows planning → executing → completed states
- **Telemetry**: Battery level decreases, altitude changes
- **2D Map**: Drone position updates, waypoints appear
- **Command History**: Your commands are logged

### 5. Test Safety Features

- **Emergency Stop**: Click the red square button
- **Low Battery**: The drone will RTL at 20% battery
- **Geofence**: Commands outside 500m will be rejected

## Manual Testing (Without Web UI)

### Terminal 1 - Launch ROS nodes:
```bash
cd ros_ws
source install/setup.bash
ros2 launch skyscout_bringup skyscout.launch.py
```

### Terminal 2 - Send commands:
```bash
ros2 topic pub /natural_language_command std_msgs/String "data: 'fly to 50 meters and search for cars'"
```

### Terminal 3 - Monitor status:
```bash
# Watch mission status
ros2 topic echo /mission_status

# Watch telemetry
ros2 topic echo /navigation/battery_level
ros2 topic echo /navigation/current_position
```

## Verifying Components

### Check Running Nodes
```bash
ros2 node list
```

Should show:
- /command_interface_node
- /llm_agent_node
- /mission_planner_node
- /navigation_bridge_node
- /perception_node

### Check Topics
```bash
ros2 topic list
```

### Monitor Logs
```bash
# See all node logs
ros2 run skyscout_bringup skyscout.launch.py

# Or individual nodes
ros2 run llm_agent llm_agent_node --ros-args --log-level debug
```

## Common Issues

### "Not connected to ROS"
- Ensure rosbridge is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check WebSocket URL is correct: ws://localhost:9090

### "LLM client not initialized"
- Set your API key: `export OPENAI_API_KEY="your-key"`
- Check internet connection

### "Mission rejected"
- Drone must be armed first
- Check battery level > 25%
- Verify command is within geofence

## Expected Behavior

1. **Takeoff**: Drone climbs at 3m/s to target altitude
2. **Navigation**: Moves at 5m/s horizontally
3. **Battery**: Drains at 0.1% per second when armed
4. **RTL**: Automatic at 20% battery
5. **Landing**: Descends at 3m/s, auto-disarms on ground

## Performance Metrics

- Command → Mission Plan: <2 seconds
- Mission Plan → First Movement: <0.5 seconds
- Telemetry Update Rate: 10Hz
- Position Update Rate: 20Hz

Happy testing! 🚁

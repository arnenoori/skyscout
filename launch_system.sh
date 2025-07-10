#!/bin/bash
# Launch script for SkyScout system

echo "Starting SkyScout drone control system..."

# Function to cleanup on exit
cleanup() {
    echo "Shutting down SkyScout..."
    kill $ROS_PID $ROSBRIDGE_PID $WEB_PID 2>/dev/null
    exit 0
}

trap cleanup EXIT INT TERM

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2..."
    source /opt/ros/iron/setup.bash
fi

# Source workspace
cd ros_ws
source install/setup.bash
cd ..

# Launch ROS2 nodes
echo "Launching ROS2 nodes..."
ros2 launch skyscout_bringup skyscout.launch.py &
ROS_PID=$!
sleep 3

# Launch rosbridge
echo "Starting rosbridge websocket server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!
sleep 2

# Launch web frontend
echo "Starting web interface..."
cd web_frontend
npm run dev &
WEB_PID=$!
cd ..

echo "----------------------------------------"
echo "SkyScout is running!"
echo "Web interface: http://localhost:3000"
echo "ROS bridge: ws://localhost:9090"
echo "Press Ctrl+C to stop"
echo "----------------------------------------"

# Wait for processes
wait

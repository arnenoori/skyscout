# SkyScout Web Frontend

This is the web interface for controlling the SkyScout drone system using natural language commands.

## Features

- **Natural Language Input**: Send commands in plain English
- **Real-time Telemetry**: Monitor battery, position, altitude, and drone status
- **Mission Monitoring**: Track mission planning and execution states
- **2D Visualization**: View drone position and waypoints on a simple map
- **Command History**: Keep track of sent commands
- **Emergency Stop**: Quickly halt drone operations

## Tech Stack

- **Next.js 15**: React framework with App Router
- **shadcn/ui**: Modern UI components built on Radix UI
- **Tailwind CSS**: Utility-first CSS framework
- **roslib.js**: WebSocket connection to ROS2 via rosbridge
- **TypeScript**: Type-safe development

## Prerequisites

1. ROS2 Iron installed and sourced
2. rosbridge_suite installed: `sudo apt install ros-iron-rosbridge-suite`
3. Node.js 18+ and npm

## Installation

```bash
npm install
```

## Development

```bash
npm run dev
```

The app will be available at http://localhost:3000

## ROS2 Connection

The frontend connects to ROS2 via rosbridge WebSocket on `ws://localhost:9090`.

Make sure rosbridge is running:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Topics

The frontend subscribes to:
- `/navigation/battery_level` - Battery percentage
- `/navigation/current_position` - Drone position
- `/navigation/armed` - Armed status
- `/navigation/status` - Flight mode and status
- `/mission_status` - Mission execution state
- `/mission_plan` - Current mission plan

The frontend publishes to:
- `/web_command` - Natural language commands

## UI Components

- **CommandInput**: Main input for natural language commands
- **TelemetryDisplay**: Shows real-time drone telemetry
- **MissionStatus**: Displays current mission state and details
- **DroneMap**: 2D visualization of drone position
- **CommandHistory**: List of previously sent commands

## Running the Complete System

Use the launch script from the project root:
```bash
../launch_system.sh
```

This will start:
1. All ROS2 nodes
2. rosbridge WebSocket server
3. Web development server

## Build for Production

```bash
npm run build
npm start
```

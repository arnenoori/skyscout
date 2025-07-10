# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SkyScout is a natural language-controlled drone system that allows non-experts to command drones using plain English. The system processes natural language commands through LLMs to generate structured outputs that control the drone autonomously.

**Current Phase**: Starting with cloud LLM APIs (OpenAI/Gemini) for rapid prototyping, with future migration to on-device models.

## System Architecture

```
Web UI (Next.js) ─► rosbridge_ws ─► command_interface (ROS2) ─► llm_agent
                                           ▲                           │
                                           │                           ▼
                       navigation_bridge ◄─ mission_planner ◄──── perception (YOLO)
                                                 │
                                                 ▼
                                            PX4 / MAVLink
```

## Key Components

- **command_interface**: ROS2 service that receives natural language commands
- **llm_agent**: Interfaces with cloud LLMs (OpenAI/Gemini) to convert NL to structured JSON mission plans
- **perception**: Real-time object detection for target identification
- **mission_planner**: Executes structured mission plans as drone behaviors
- **navigation_bridge**: MAVLink/PX4 interface for drone control
- **web_frontend**: Next.js interface for sending commands and viewing results

## Development Commands

### ROS2 Workspace
```bash
# Build ROS packages
cd ros_ws
colcon build

# Source ROS environment
source /opt/ros/iron/setup.bash
source install/setup.bash

# Create new ROS package
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy <other_deps>

# Run individual nodes
ros2 run <package_name> <node_name>
```

### Frontend
```bash
cd web_frontend
npm install          # Install dependencies
npm run dev         # Development server
npm run build       # Production build
npm run lint        # Lint TypeScript/React code
```

### Testing
```bash
# ROS2 tests
cd ros_ws
colcon test

# Frontend tests
cd web_frontend
npm test
```

## API Configuration

Set environment variables for LLM APIs:
```bash
export OPENAI_API_KEY="your-key-here"
export GEMINI_API_KEY="your-key-here"
```

## Structured Output Schema

The LLM must produce JSON mission plans following this structure:
```json
{
  "mission_type": "search|inspect|count|map",
  "target_description": "string describing what to look for",
  "flight_pattern": "grid|spiral|perimeter|waypoints",
  "parameters": {
    "altitude": 10-120,
    "speed": 1-10,
    "coverage_area": {...}
  },
  "safety": {
    "geofence": {...},
    "rtl_battery_threshold": 20
  }
}
```

## Development Priorities

1. Cloud API integration for natural language → structured JSON
2. JSON schema validation to ensure safe mission plans
3. SITL testing with Gazebo before real flights
4. Structured output reliability (handling LLM hallucinations)
5. Migration path from cloud to on-device models

## Git Workflow

After completing each milestone or significant feature:
- User will request to commit changes
- Use descriptive commit messages
- Follow conventional commits format when appropriate

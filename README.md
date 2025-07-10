# SkyScout

Natural language-controlled drone system that enables non-experts to command drones using plain English.

## Overview

SkyScout processes natural language commands through LLMs to generate structured outputs that control drones autonomously. No joystick or GCS waypoint configuration required - just tell the drone what you need.

## Architecture

```
Web UI (Next.js) ─► rosbridge_ws ─► command_interface (ROS2) ─► llm_agent
                                           ▲                           │
                                           │                           ▼
                       navigation_bridge ◄─ mission_planner ◄──── perception (YOLO)
                                                 │
                                                 ▼
                                            PX4 / MAVLink
```

## Quick Start

### Prerequisites
- ROS2 Iron
- Node.js 20+
- Python 3.10+
- Docker (for devcontainer)

### Development Setup

1. Clone the repository:
```bash
git clone https://github.com/yourusername/skyscout.git
cd skyscout
```

2. Build ROS2 packages:
```bash
cd ros_ws
source /opt/ros/iron/setup.bash
colcon build
source install/setup.bash
```

3. Start the frontend:
```bash
cd web_frontend
npm install
npm run dev
```

### Using DevContainer

Open the project in VS Code and use the "Reopen in Container" option for a pre-configured development environment.

## Components

- **command_interface**: Natural language command receiver
- **llm_agent**: Cloud LLM integration (OpenAI/Gemini)
- **perception**: Real-time object detection
- **mission_planner**: Mission execution state machine
- **navigation_bridge**: PX4/MAVLink interface
- **web_frontend**: Command interface dashboard

## Configuration

Set your LLM API keys:
```bash
export OPENAI_API_KEY="your-key-here"
export GEMINI_API_KEY="your-key-here"
```

## License

[Add license information]

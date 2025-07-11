# Getting Started with SkyScout 🚁

Welcome to SkyScout! This guide will walk you through setting up your development environment and running your first natural language drone mission.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation Options](#installation-options)
  - [Docker Setup (Recommended)](#docker-setup-recommended)
  - [Native Ubuntu Setup](#native-ubuntu-setup)
  - [macOS Development Setup](#macos-development-setup)
- [First Flight Tutorial](#first-flight-tutorial)
- [Troubleshooting](#troubleshooting)
- [Next Steps](#next-steps)

## Prerequisites

### Hardware Requirements
- **Development Machine**:
  - 8GB RAM minimum (16GB recommended)
  - 20GB free disk space
  - Ubuntu 22.04, macOS 12+, or Windows with WSL2

- **For Real Drone Testing**:
  - Raspberry Pi 5 (8GB model recommended)
  - Pixhawk 6C flight controller
  - USB or CSI camera
  - PX4-compatible drone frame
  - RC transmitter for safety override

### Software Requirements
- Git
- Docker (for containerized setup)
- OR native installations of:
  - ROS2 Iron
  - Node.js 20+
  - Python 3.10+

### API Keys
You'll need at least one of these:
- OpenAI API key ([Get one here](https://platform.openai.com/api-keys))
- Google Gemini API key ([Get one here](https://makersuite.google.com/app/apikey))

Optional but recommended:
- OpenWeather API key ([Get one here](https://openweathermap.org/api)) - For weather safety checks

## Installation Options

### Docker Setup (Recommended) 🐳

This is the easiest way to get started, especially on macOS or Windows.

1. **Install Docker**:
   - [Docker Desktop](https://www.docker.com/products/docker-desktop/) for macOS/Windows
   - Linux: `curl -fsSL https://get.docker.com | sh`

2. **Clone the repository**:
   ```bash
   git clone https://github.com/arnenoori/skyscout.git
   cd skyscout
   ```

3. **Create environment file**:
   ```bash
   cp .env.example .env
   # Edit .env and add your API keys:
   # OPENAI_API_KEY=your-key-here
   # GEMINI_API_KEY=your-key-here
   # OPENWEATHER_API_KEY=your-key-here (optional)
   ```

4. **Start the system**:
   ```bash
   docker-compose up -d
   ```

5. **Access the interfaces**:
   - Web UI: http://localhost:3000
   - ROS Bridge: ws://localhost:9090
   - API docs: http://localhost:8080/docs

### Native Ubuntu Setup 🐧

For the best performance and hardware access on Linux.

1. **Install ROS2 Iron**:
   ```bash
   # Add ROS2 apt repository
   sudo apt update && sudo apt install curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install ROS2 Iron
   sudo apt update
   sudo apt install ros-iron-desktop python3-argcomplete ros-dev-tools
   ```

2. **Install Node.js 20**:
   ```bash
   curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
   sudo apt-get install -y nodejs
   ```

3. **Install Python dependencies**:
   ```bash
   sudo apt install python3-pip python3-venv
   pip3 install --user pipx
   pipx ensurepath
   ```

4. **Clone and setup SkyScout**:
   ```bash
   git clone https://github.com/arnenoori/skyscout.git
   cd skyscout

   # Install ROS dependencies
   sudo apt install ros-iron-rosbridge-suite

   # Build ROS workspace
   cd ros_ws
   source /opt/ros/iron/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install

   # Setup web frontend
   cd ../web_frontend
   npm install
   ```

5. **Configure environment**:
   ```bash
   # Add to ~/.bashrc
   echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
   echo "source ~/skyscout/ros_ws/install/setup.bash" >> ~/.bashrc

   # Set API keys
   echo "export OPENAI_API_KEY='your-key-here'" >> ~/.bashrc
   echo "export OPENWEATHER_API_KEY='your-key-here'" >> ~/.bashrc  # Optional
   source ~/.bashrc
   ```

### macOS Development Setup 🍎

For development and simulation only (no real drone control).

1. **Install Homebrew** (if not already installed):
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

2. **Install dependencies**:
   ```bash
   brew install node@20 python@3.10
   ```

3. **Clone and setup**:
   ```bash
   git clone https://github.com/arnenoori/skyscout.git
   cd skyscout

   # Frontend only (ROS runs in Docker)
   cd web_frontend
   npm install
   ```

4. **Use VS Code DevContainer** (recommended):
   - Install VS Code and Docker Desktop
   - Open project in VS Code
   - Click "Reopen in Container"

## First Flight Tutorial 🎮

### 1. Start the System

**Docker**:
```bash
docker-compose up
```

**Native**:
```bash
# Terminal 1: ROS Bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: SkyScout nodes
ros2 launch skyscout_bringup skyscout.launch.py

# Terminal 3: Web frontend
cd web_frontend && npm run dev
```

### 2. Verify System Health

Open http://localhost:3000 and check:
- ✅ ROS connection indicator (green)
- ✅ LLM API status (green)
- ✅ All nodes reporting healthy

### 3. Run in Simulation

Before real flights, test in Gazebo:

```bash
# Terminal 4: Launch PX4 SITL with Gazebo
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

### 4. Your First Command

In the web interface, try these commands:
- "Take off and hover at 10 meters"
- "Search for red cars using zigzag pattern"
- "Inspect the building roof for damage" (uses circle pattern)
- "Do a quick search for missing person" (uses template)
- "Patrol the perimeter if weather permits"
- "Emergency response to accident site"

### 5. Monitor Execution

Watch as SkyScout:
1. Processes your natural language
2. Checks weather conditions (if enabled)
3. Generates a mission plan with appropriate flight pattern
4. Validates safety parameters
5. Executes the mission autonomously
6. Returns results and detections

## Troubleshooting 🔧

### Common Issues

**ROS Bridge Connection Failed**
```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Restart rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**LLM API Errors**
```bash
# Verify API key is set
echo $OPENAI_API_KEY

# Test API directly
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

**Build Failures**
```bash
# Clean and rebuild
cd ros_ws
rm -rf build install log
colcon build --symlink-install
```

**Camera Not Detected**
```bash
# List USB devices
ls /dev/video*

# Test camera
ros2 run image_tools cam2image
```

### Debug Mode

Enable verbose logging:
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

## Next Steps 🚀

Now that you have SkyScout running:

1. **Read the Architecture Guide**: Understand how components work together in [ARCHITECTURE.md](ARCHITECTURE.md)

2. **Try More Examples**: See advanced commands in [EXAMPLES.md](EXAMPLES.md)

3. **Set Up Hardware**: Connect real drone hardware following [HARDWARE_SETUP.md](HARDWARE_SETUP.md)

4. **Customize the System**:
   - Modify LLM prompts in `llm_agent/prompts/`
   - Add new mission types in `mission_planner/`
   - Create custom mission templates in `llm_agent/mission_templates.py`
   - Adjust weather safety limits in `llm_agent/weather_client.py`
   - Add new flight patterns in `mission_planner/node.py`
   - Integrate different object detection models

5. **Join the Community**:
   - Report issues on [GitHub](https://github.com/arnenoori/skyscout/issues)
   - Share your missions in [Discussions](https://github.com/arnenoori/skyscout/discussions)

## Safety First! ⚠️

Remember:
- Always test in simulation before real flights
- Maintain visual line of sight
- Keep RC override ready
- Follow local regulations
- Never fly over people
- Check weather conditions (automatic with OpenWeather API)
- Monitor battery levels (automatic RTL at threshold)
- Stay within geofence boundaries

Happy flying! 🚁✨

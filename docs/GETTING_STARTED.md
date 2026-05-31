# Getting Started

This guide covers the current development workflow. The system is a prototype and defaults to the mock navigation bridge.

## Frontend

```bash
cd web_frontend
npm ci
npm run dev
```

Open `http://localhost:3000`.

Run frontend checks:

```bash
npm run verify
```

## ROS Workspace

Use Ubuntu 22.04 with ROS2 Iron.

```bash
cd ros_ws
source /opt/ros/iron/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro iron
colcon build --symlink-install
source install/setup.bash
```

Start the SkyScout stack:

```bash
ros2 launch skyscout_bringup skyscout.launch.py
```

Start rosbridge separately:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## API Keys

The LLM agent reads keys from environment variables:

```bash
export OPENAI_API_KEY="..."
export GEMINI_API_KEY="..."
```

At least one provider key is required for LLM-backed mission planning. Without a configured provider, the agent falls back to a conservative default mission.

## Fast Tests

These do not require ROS:

```bash
python -m unittest discover -s tests -v
```

Install local Python tooling:

```bash
python -m pip install -r requirements-dev.txt
ruff check .
ruff format --check .
```

## Manual Smoke Test

After ROS nodes are running:

```bash
python test_system.py "Inspect the building roof for damage"
```

## Safety

Keep testing in simulation or mock mode until real navigation, perception, geofencing, manual override, and regulatory requirements have been fully validated.

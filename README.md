# SkyScout

SkyScout is a prototype ROS2 drone stack for turning natural-language commands into structured mission plans, then routing those plans through a mission planner, navigation bridge, perception node, and Next.js operator dashboard.

The repository is still a development prototype. It is useful for experimenting with the system architecture, ROS topic contracts, simulated navigation, and the web control surface. It is not ready for autonomous real-world flight without a full safety review, hardware integration, simulator validation, and regulatory compliance work.

## What Is In This Repo

```text
skyscout/
├── ros_ws/
│   └── src/
│       ├── command_interface/    # Forwards web commands into ROS
│       ├── llm_agent/            # Converts natural language to mission JSON
│       ├── mission_planner/      # Expands mission plans into waypoints
│       ├── navigation_bridge/    # Mock PX4/MAVLink-style navigation bridge
│       ├── perception/           # Placeholder object-detection node
│       └── skyscout_bringup/     # ROS launch file
├── web_frontend/                 # Next.js operator dashboard
├── tests/                        # Fast pure-Python unit tests
├── docs/                         # Design notes and operating references
└── test_system.py                # Manual ROS system smoke test
```

## Architecture

```text
Next.js dashboard
  -> rosbridge websocket
  -> command_interface
  -> llm_agent
  -> mission_planner
  -> navigation_bridge
  -> mock drone or future PX4/MAVLink adapter

perception publishes detections back to mission_planner
navigation_bridge publishes telemetry back to the dashboard
```

## Requirements

For frontend development:

- Node.js 22+
- npm

For ROS development:

- Ubuntu 22.04
- ROS2 Iron
- `rosbridge_suite`
- Python 3.10+
- `colcon`

For LLM-backed mission planning:

- `OPENAI_API_KEY` or `GEMINI_API_KEY`

## Frontend Setup

```bash
cd web_frontend
npm ci
npm run dev
```

The dashboard runs at `http://localhost:3000` and expects rosbridge at `ws://localhost:9090`.

Useful frontend checks:

```bash
npm run lint
npm run typecheck
npm run format:check
npm run knip
npm run doctor
npm run audit
npm run build
```

To run the complete frontend gate:

```bash
npm run verify
```

## ROS Setup

```bash
cd ros_ws
source /opt/ros/iron/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro iron
colcon build --symlink-install
source install/setup.bash
```

Start the ROS stack:

```bash
ros2 launch skyscout_bringup skyscout.launch.py
```

Start rosbridge in a separate terminal:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Tests And Quality Gates

Fast local Python tests do not require ROS:

```bash
python -m unittest discover -s tests -v
```

Python lint/format checks:

```bash
python -m pip install -r requirements-dev.txt
ruff check .
ruff format --check .
```

ROS package tests:

```bash
cd ros_ws
source /opt/ros/iron/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

CI runs three jobs:

- Web quality gates: npm install, ESLint, TypeScript, Prettier, Knip, React Doctor, npm audit, Next build
- Python checks: Ruff and fast unit tests
- ROS workspace: dependency install, colcon build, colcon test

## Current Limitations

- The navigation bridge uses `MockDrone` by default.
- Real MAVLink/PX4 support is a placeholder.
- The perception node publishes mock detections.
- The LLM agent needs API keys and should be hardened before operational use.
- Mission validation is intentionally conservative but not flight-certification-grade.

## Safety

Do not use this prototype for unsupervised real-world flight. Before hardware use, validate every mission path in simulation, verify geofencing and return-to-launch behavior, keep manual RC override available, and comply with local aviation rules.

## License

MIT. See [LICENSE](LICENSE).

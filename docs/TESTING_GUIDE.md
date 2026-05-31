# Testing Guide

## Frontend Gates

```bash
cd web_frontend
npm ci
npm run verify
```

`npm run verify` runs:

- ESLint
- TypeScript
- Prettier check
- Knip
- React Doctor
- npm production audit
- Next build

## Python Gates

```bash
python -m pip install -r requirements-dev.txt
ruff check .
ruff format --check .
python -m unittest discover -s tests -v
```

The unit tests under `tests/` intentionally avoid ROS imports so they can run quickly on any development machine.

## ROS Gates

```bash
cd ros_ws
source /opt/ros/iron/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## Manual End-To-End Smoke Test

Start the stack:

```bash
ros2 launch skyscout_bringup skyscout.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then publish a command through the manual test script:

```bash
python test_system.py "Search the parking lot for red vehicles"
```

Watch:

- `/mission_plan`
- `/mission_status`
- `/navigation/status`
- `/navigation/current_position`

## CI

GitHub Actions runs separate web, Python, and ROS jobs. A pull request should pass all three before merge.

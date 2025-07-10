# Command Interface

ROS2 node that receives natural language commands from the web interface and forwards them to the LLM agent.

## Overview

Acts as the gateway between the web UI (via rosbridge) and the ROS2 system.

## Topics

**Subscribers:**
- `/web_command` (std_msgs/String) - Raw commands from web UI
- `/emergency_stop` (std_msgs/Bool) - Emergency stop trigger

**Publishers:**
- `/natural_language_command` (std_msgs/String) - Validated commands

## Usage

```bash
ros2 run command_interface command_interface_node
```

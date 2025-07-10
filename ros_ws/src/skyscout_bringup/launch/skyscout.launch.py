#!/usr/bin/env python3
"""
Launch file for the complete SkyScout system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Declare launch arguments
    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="true",
        description="Use mock drone instead of real MAVLink connection",
    )

    llm_provider_arg = DeclareLaunchArgument(
        "llm_provider",
        default_value="openai",
        description="LLM provider to use (openai or gemini)",
    )

    # Get configurations
    use_mock = LaunchConfiguration("use_mock")
    llm_provider = LaunchConfiguration("llm_provider")

    # LLM Agent Node
    llm_agent_node = Node(
        package="llm_agent",
        executable="llm_agent_node",
        name="llm_agent",
        output="screen",
        parameters=[
            {
                "llm_provider": llm_provider,
                "api_key": os.getenv("OPENAI_API_KEY", "")
                if llm_provider == "openai"
                else os.getenv("GEMINI_API_KEY", ""),
            }
        ],
    )

    # Mission Planner Node
    mission_planner_node = Node(
        package="mission_planner",
        executable="mission_planner_node",
        name="mission_planner",
        output="screen",
    )

    # Navigation Bridge Node
    navigation_bridge_node = Node(
        package="navigation_bridge",
        executable="navigation_bridge_node",
        name="navigation_bridge",
        output="screen",
        parameters=[{"use_mock": use_mock, "connection_string": "mock"}],
    )

    # Command Interface Node (optional, mainly for rosbridge integration)
    command_interface_node = Node(
        package="command_interface",
        executable="command_interface_node",
        name="command_interface",
        output="screen",
    )

    # Perception Node (disabled for now as we don't have camera simulation)
    # perception_node = Node(
    #     package='perception',
    #     executable='perception_node',
    #     name='perception',
    #     output='screen'
    # )

    return LaunchDescription(
        [
            # Launch arguments
            use_mock_arg,
            llm_provider_arg,
            # Log startup message
            LogInfo(msg="Starting SkyScout system..."),
            LogInfo(msg=["Using LLM provider: ", llm_provider]),
            LogInfo(msg=["Using mock drone: ", use_mock]),
            # Nodes
            llm_agent_node,
            mission_planner_node,
            navigation_bridge_node,
            command_interface_node,
        ]
    )

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
import json
import logging
from typing import Dict, Any
from datetime import datetime

from .llm_client import create_llm_client


class LLMAgentNode(Node):
    """
    ROS2 node that interfaces with cloud LLMs (OpenAI/Gemini) to convert
    natural language commands into structured JSON mission plans.
    """

    def __init__(self):
        super().__init__("llm_agent_node")

        # Configure logging
        logging.basicConfig(level=logging.INFO)

        # Declare parameters for API configuration
        self.declare_parameter("llm_provider", "openai")  # 'openai' or 'gemini'
        self.declare_parameter("api_key", "")
        self.declare_parameter("model", "")  # Optional model override

        # Get parameters
        provider = self.get_parameter("llm_provider").get_parameter_value().string_value
        api_key = self.get_parameter("api_key").get_parameter_value().string_value

        # Initialize LLM client
        try:
            self.llm_client = create_llm_client(provider, api_key)
            self.get_logger().info(f"Initialized {provider} LLM client")
        except ValueError as e:
            self.get_logger().error(f"Failed to initialize LLM client: {e}")
            self.llm_client = None

        # Create subscriber for natural language commands
        self.command_subscriber = self.create_subscription(
            String, "natural_language_command", self.command_callback, 10
        )

        # Create publisher for structured mission plans
        self.mission_publisher = self.create_publisher(String, "mission_plan", 10)

        # Subscribe to telemetry for context
        self.battery_subscriber = self.create_subscription(
            Float32, "navigation/battery_level", self.battery_callback, 10
        )
        self.position_subscriber = self.create_subscription(
            Point, "navigation/current_position", self.position_callback, 10
        )

        # Context for mission planning
        self.context = {
            "battery_percent": 100,
            "position": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
            "weather": "Clear",  # Could integrate weather API
            "time": "Day",
        }

        self.get_logger().info("LLM Agent Node initialized")

    def command_callback(self, msg):
        """Process natural language command and convert to structured mission plan."""
        self.get_logger().info(f"Processing command: {msg.data}")

        if not self.llm_client:
            self.get_logger().error("LLM client not initialized, using default mission")
            mission_plan = self._create_default_mission(msg.data)
        else:
            try:
                # Update context with current time
                current_hour = datetime.now().hour
                self.context["time"] = (
                    "Night" if current_hour < 6 or current_hour > 20 else "Day"
                )

                # Generate mission plan using LLM
                mission_plan = self.llm_client.generate_mission_plan(
                    msg.data, self.context
                )
                self.get_logger().info(
                    f"Generated mission: {mission_plan.mission_type} - {mission_plan.target_description}"
                )

            except Exception as e:
                self.get_logger().error(f"Error generating mission plan: {e}")
                mission_plan = self._create_default_mission(msg.data)

        # Convert to dict and publish
        mission_dict = (
            mission_plan.model_dump()
            if hasattr(mission_plan, "model_dump")
            else mission_plan.dict()
        )
        mission_msg = String()
        mission_msg.data = json.dumps(mission_dict, indent=2)
        self.mission_publisher.publish(mission_msg)

        self.get_logger().info(
            f"Published mission plan for: {mission_plan.target_description}"
        )
        self._log_mission_details(mission_dict)

    def battery_callback(self, msg):
        """Update battery level in context."""
        self.context["battery_percent"] = int(msg.data)

    def position_callback(self, msg):
        """Update current position in context."""
        self.context["position"] = {
            "lat": msg.x,  # In a real system, this would be proper GPS coords
            "lon": msg.y,
            "alt": msg.z,
        }

    def _create_default_mission(self, command: str) -> Dict[str, Any]:
        """Create a safe default mission when LLM is unavailable."""
        return {
            "mission_type": "search",
            "target_description": command[:50] + "..."
            if len(command) > 50
            else command,
            "flight_pattern": "grid",
            "parameters": {
                "altitude": 20,
                "speed": 3,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 50},
            },
            "safety": {
                "geofence": {"max_altitude": 50, "max_distance": 100},
                "rtl_battery_threshold": 30,
            },
        }

    def _log_mission_details(self, mission: Dict[str, Any]):
        """Log mission details for debugging."""
        self.get_logger().info(
            f"Mission details:\n"
            f"  Type: {mission['mission_type']}\n"
            f"  Target: {mission['target_description']}\n"
            f"  Pattern: {mission['flight_pattern']}\n"
            f"  Altitude: {mission['parameters']['altitude']}m\n"
            f"  Speed: {mission['parameters']['speed']}m/s\n"
            f"  Coverage radius: {mission['parameters']['coverage_area']['radius']}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LLMAgentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os


class LLMAgentNode(Node):
    """
    ROS2 node that interfaces with cloud LLMs (OpenAI/Gemini) to convert
    natural language commands into structured JSON mission plans.
    """

    def __init__(self):
        super().__init__("llm_agent_node")

        # Declare parameters for API configuration
        self.declare_parameter("llm_provider", "openai")  # 'openai' or 'gemini'
        self.declare_parameter("api_key", "")

        # Create subscriber for natural language commands
        self.command_subscriber = self.create_subscription(
            String, "natural_language_command", self.command_callback, 10
        )

        # Create publisher for structured mission plans
        self.mission_publisher = self.create_publisher(String, "mission_plan", 10)

        # Check for API key from environment or parameter
        self.api_key = self.get_parameter("api_key").get_parameter_value().string_value
        if not self.api_key:
            self.api_key = os.getenv("OPENAI_API_KEY") or os.getenv("GEMINI_API_KEY")

        self.get_logger().info("LLM Agent Node initialized")

    def command_callback(self, msg):
        """Process natural language command and convert to structured mission plan."""
        self.get_logger().info(f"Processing command: {msg.data}")

        # TODO: Implement actual LLM API calls
        # For now, return a mock structured response
        mission_plan = self.generate_mission_plan(msg.data)

        # Publish the structured mission plan
        mission_msg = String()
        mission_msg.data = json.dumps(mission_plan)
        self.mission_publisher.publish(mission_msg)
        self.get_logger().info(f"Published mission plan: {mission_msg.data}")

    def generate_mission_plan(self, command):
        """
        Generate structured mission plan from natural language command.
        TODO: Implement actual LLM API integration
        """
        # Mock response for now
        return {
            "mission_type": "search",
            "target_description": "red vehicle",
            "flight_pattern": "grid",
            "parameters": {
                "altitude": 30,
                "speed": 5,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 100},
            },
            "safety": {
                "geofence": {"max_altitude": 120, "max_distance": 500},
                "rtl_battery_threshold": 20,
            },
        }


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

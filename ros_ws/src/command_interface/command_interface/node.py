#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommandInterfaceNode(Node):
    """
    ROS2 node for receiving natural language commands and forwarding them
    to the LLM agent for processing.
    """

    def __init__(self):
        super().__init__("command_interface_node")

        # Create publisher for forwarding commands
        self.command_publisher = self.create_publisher(
            String, "natural_language_command", 10
        )

        # Create subscriber for receiving commands from web interface
        self.web_command_subscriber = self.create_subscription(
            String, "web_command", self.web_command_callback, 10
        )

        self.get_logger().info("Command Interface Node initialized")

    def web_command_callback(self, msg):
        """Process incoming commands from web interface."""
        self.get_logger().info(f"Received command: {msg.data}")

        # Forward the command to LLM agent
        self.command_publisher.publish(msg)
        self.get_logger().info("Command forwarded to LLM agent")


def main(args=None):
    rclpy.init(args=args)
    node = CommandInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

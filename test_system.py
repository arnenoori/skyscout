#!/usr/bin/env python3
"""
Test script for the complete SkyScout system.
This simulates sending a natural language command and observing the drone response.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys


class SystemTester(Node):
    def __init__(self):
        super().__init__("system_tester")

        # Publisher for commands
        self.command_pub = self.create_publisher(
            String, "/natural_language_command", 10
        )

        # Subscribers for monitoring
        self.mission_plan_sub = self.create_subscription(
            String, "/mission_plan", self.mission_plan_callback, 10
        )

        self.mission_status_sub = self.create_subscription(
            String, "/mission_status", self.mission_status_callback, 10
        )

        self.nav_status_sub = self.create_subscription(
            String, "/navigation/status", self.nav_status_callback, 10
        )

        self.get_logger().info("System tester initialized")

    def mission_plan_callback(self, msg):
        self.get_logger().info(f"Mission Plan Generated: {msg.data[:100]}...")

    def mission_status_callback(self, msg):
        self.get_logger().info(f"Mission Status: {msg.data}")

    def nav_status_callback(self, msg):
        self.get_logger().info(f"Navigation Status: {msg.data}")

    def send_command(self, command: str):
        """Send a natural language command."""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"Sent command: {command}")


def main():
    print("\n=== SkyScout System Test ===\n")

    if len(sys.argv) < 2:
        print("Usage: python test_system.py <command>")
        print("\nExample commands:")
        print("  'Find all red vehicles in the parking lot'")
        print("  'Inspect the solar panels for damage'")
        print("  'Count people wearing safety helmets'")
        print("  'Map the perimeter of the building'")
        return

    command = " ".join(sys.argv[1:])

    print("Before running this test, make sure all nodes are running:")
    print("  Terminal 1: ros2 run llm_agent llm_agent_node")
    print("  Terminal 2: ros2 run mission_planner mission_planner_node")
    print("  Terminal 3: ros2 run navigation_bridge navigation_bridge_node")
    print("  Terminal 4: ros2 run command_interface command_interface_node (optional)")
    print("\nOr use: ros2 launch skyscout_bringup skyscout.launch.py")
    print("\nPress Enter to continue...")
    input()

    rclpy.init()
    tester = SystemTester()

    # Send command after a short delay
    time.sleep(1.0)
    tester.send_command(command)

    print(f"\nSent command: '{command}'")
    print("Monitoring system response...")
    print("Press Ctrl+C to stop\n")

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\nTest stopped")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

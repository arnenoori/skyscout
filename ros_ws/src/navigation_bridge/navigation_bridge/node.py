#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, String, Bool
import json


class NavigationBridgeNode(Node):
    """
    ROS2 node that bridges between mission planner and PX4/MAVLink.
    Handles low-level drone control commands and telemetry.
    """

    def __init__(self):
        super().__init__("navigation_bridge_node")

        # Declare parameters
        self.declare_parameter("connection_string", "udp://:14540")
        self.declare_parameter("system_id", 1)
        self.declare_parameter("component_id", 1)

        # Current drone state
        self.is_armed = False
        self.current_mode = "MANUAL"
        self.battery_level = 100.0
        self.current_position = Point()

        # Create subscribers
        self.waypoint_subscriber = self.create_subscription(
            Point, "navigation/waypoint", self.waypoint_callback, 10
        )

        self.arm_subscriber = self.create_subscription(
            Bool, "navigation/arm", self.arm_callback, 10
        )

        # Create publishers
        self.position_publisher = self.create_publisher(
            Point, "navigation/current_position", 10
        )

        self.battery_publisher = self.create_publisher(
            Float32, "navigation/battery_level", 10
        )

        self.status_publisher = self.create_publisher(String, "navigation/status", 10)

        # Create timer for telemetry updates
        self.telemetry_timer = self.create_timer(0.1, self.publish_telemetry)

        # TODO: Initialize MAVLink connection
        self.connection_string = self.get_parameter("connection_string").value
        self.get_logger().info(
            f"Navigation Bridge initialized with connection: {self.connection_string}"
        )

    def waypoint_callback(self, msg):
        """Handle waypoint navigation commands."""
        self.get_logger().info(f"Received waypoint: x={msg.x}, y={msg.y}, z={msg.z}")

        # TODO: Implement MAVLink waypoint command
        # For now, just update internal position (mock)
        self.current_position = msg

        # Send status update
        self.publish_status(
            {
                "action": "waypoint_received",
                "position": {"x": msg.x, "y": msg.y, "z": msg.z},
            }
        )

    def arm_callback(self, msg):
        """Handle arm/disarm commands."""
        if msg.data:
            self.get_logger().info("Arming drone")
            # TODO: Send MAVLink arm command
            self.is_armed = True
            self.publish_status({"action": "armed"})
        else:
            self.get_logger().info("Disarming drone")
            # TODO: Send MAVLink disarm command
            self.is_armed = False
            self.publish_status({"action": "disarmed"})

    def publish_telemetry(self):
        """Publish current telemetry data."""
        # Publish position
        self.position_publisher.publish(self.current_position)

        # Publish battery (mock decreasing battery)
        if self.is_armed and self.battery_level > 0:
            self.battery_level -= 0.01  # Mock battery drain

        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_publisher.publish(battery_msg)

        # Check for low battery RTL
        if self.battery_level < 20.0 and self.is_armed:
            self.get_logger().warning("Low battery! Initiating RTL")
            self.return_to_launch()

    def return_to_launch(self):
        """Initiate return to launch procedure."""
        self.get_logger().info("Executing Return to Launch")
        # TODO: Send MAVLink RTL command
        self.publish_status({"action": "rtl", "reason": "low_battery"})

    def publish_status(self, status_data):
        """Publish navigation status update."""
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_publisher.publish(status_msg)

    def set_flight_mode(self, mode):
        """Set PX4 flight mode."""
        self.get_logger().info(f"Setting flight mode to: {mode}")
        # TODO: Implement MAVLink mode change
        self.current_mode = mode
        self.publish_status({"action": "mode_change", "mode": mode})


def main(args=None):
    rclpy.init(args=args)
    node = NavigationBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

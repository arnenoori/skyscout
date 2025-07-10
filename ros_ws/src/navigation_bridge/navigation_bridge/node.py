#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, String, Bool
import json

from .mock_drone import MockDrone


class NavigationBridgeNode(Node):
    """
    ROS2 node that bridges between mission planner and PX4/MAVLink.
    Handles low-level drone control commands and telemetry.
    """

    def __init__(self):
        super().__init__("navigation_bridge_node")

        # Declare parameters
        self.declare_parameter(
            "connection_string", "mock"
        )  # "mock" or real MAVLink connection
        self.declare_parameter("system_id", 1)
        self.declare_parameter("component_id", 1)
        self.declare_parameter("use_mock", True)  # Use mock drone by default

        # Get parameters
        self.connection_string = self.get_parameter("connection_string").value
        self.use_mock = self.get_parameter("use_mock").value

        # Initialize drone interface
        if self.use_mock or self.connection_string == "mock":
            self.drone = MockDrone()
            self.get_logger().info("Using mock drone for testing")
        else:
            # TODO: Implement real MAVLink connection
            self.get_logger().warn("Real MAVLink not implemented yet, using mock")
            self.drone = MockDrone()

        # Create subscribers
        self.waypoint_subscriber = self.create_subscription(
            Point, "navigation/waypoint", self.waypoint_callback, 10
        )

        self.arm_subscriber = self.create_subscription(
            Bool, "navigation/arm", self.arm_callback, 10
        )

        self.takeoff_subscriber = self.create_subscription(
            Float32, "navigation/takeoff", self.takeoff_callback, 10
        )

        self.land_subscriber = self.create_subscription(
            Bool, "navigation/land", self.land_callback, 10
        )

        self.rtl_subscriber = self.create_subscription(
            Bool, "navigation/rtl", self.rtl_callback, 10
        )

        # Create publishers
        self.position_publisher = self.create_publisher(
            Point, "navigation/current_position", 10
        )

        self.battery_publisher = self.create_publisher(
            Float32, "navigation/battery_level", 10
        )

        self.status_publisher = self.create_publisher(String, "navigation/status", 10)

        self.armed_publisher = self.create_publisher(Bool, "navigation/armed", 10)

        # Create timer for telemetry updates
        self.telemetry_timer = self.create_timer(0.1, self.publish_telemetry)  # 10Hz

        # Create timer for drone updates (if using mock)
        if self.use_mock:
            self.update_timer = self.create_timer(0.05, self.update_drone)  # 20Hz

        self.get_logger().info("Navigation Bridge initialized")

    def waypoint_callback(self, msg):
        """Handle waypoint navigation commands."""
        self.get_logger().info(f"Received waypoint: x={msg.x}, y={msg.y}, z={msg.z}")

        # Convert local coordinates to GPS (simplified)
        # In real implementation, this would use proper coordinate transformation
        lat = self.drone.home_position["lat"] + (msg.y / 111111.0)
        lon = self.drone.home_position["lon"] + (msg.x / 111111.0)
        alt = msg.z

        success = self.drone.goto(lat, lon, alt)

        if success:
            self.publish_status(
                {
                    "action": "waypoint_accepted",
                    "position": {"x": msg.x, "y": msg.y, "z": msg.z},
                }
            )
        else:
            self.publish_status(
                {
                    "action": "waypoint_rejected",
                    "reason": "geofence_violation"
                    if self.drone.is_armed
                    else "not_armed",
                }
            )

    def arm_callback(self, msg):
        """Handle arm/disarm commands."""
        if msg.data:
            self.get_logger().info("Arming drone")
            success = self.drone.arm()
            if success:
                self.publish_status({"action": "armed", "success": True})
            else:
                self.publish_status(
                    {
                        "action": "arm_failed",
                        "reason": "low_battery"
                        if self.drone.battery_percent < 25
                        else "unknown",
                    }
                )
        else:
            self.get_logger().info("Disarming drone")
            success = self.drone.disarm()
            if success:
                self.publish_status({"action": "disarmed", "success": True})
            else:
                self.publish_status(
                    {
                        "action": "disarm_failed",
                        "reason": "in_flight"
                        if self.drone.position["alt"] > 0.5
                        else "unknown",
                    }
                )

    def takeoff_callback(self, msg):
        """Handle takeoff command."""
        altitude = msg.data
        self.get_logger().info(f"Taking off to {altitude}m")

        success = self.drone.takeoff(altitude)
        if success:
            self.publish_status(
                {"action": "takeoff_initiated", "target_altitude": altitude}
            )
        else:
            self.publish_status(
                {
                    "action": "takeoff_failed",
                    "reason": "not_armed" if not self.drone.is_armed else "unknown",
                }
            )

    def land_callback(self, msg):
        """Handle land command."""
        self.get_logger().info("Landing drone")
        success = self.drone.land()
        if success:
            self.publish_status({"action": "landing_initiated"})

    def rtl_callback(self, msg):
        """Handle return-to-launch command."""
        self.get_logger().info("Return to launch")
        success = self.drone.return_to_launch()
        if success:
            self.publish_status({"action": "rtl_initiated"})

    def publish_telemetry(self):
        """Publish current telemetry data."""
        telemetry = self.drone.get_telemetry()

        # Publish position
        position_msg = Point()
        # Convert GPS to local coordinates (simplified)
        position_msg.x = (
            telemetry["position"]["lon"] - self.drone.home_position["lon"]
        ) * 111111.0
        position_msg.y = (
            telemetry["position"]["lat"] - self.drone.home_position["lat"]
        ) * 111111.0
        position_msg.z = telemetry["position"]["alt"]
        self.position_publisher.publish(position_msg)

        # Publish battery
        battery_msg = Float32()
        battery_msg.data = telemetry["battery_percent"]
        self.battery_publisher.publish(battery_msg)

        # Publish armed status
        armed_msg = Bool()
        armed_msg.data = telemetry["armed"]
        self.armed_publisher.publish(armed_msg)

        # Log important state changes
        if hasattr(self, "_last_mode") and self._last_mode != telemetry["mode"]:
            self.get_logger().info(
                f"Mode changed: {self._last_mode} → {telemetry['mode']}"
            )
            self.publish_status({"action": "mode_change", "mode": telemetry["mode"]})
        self._last_mode = telemetry["mode"]

        # Check for target reached
        if telemetry.get("target_reached", False) and hasattr(
            self, "_last_target_reached"
        ):
            if not self._last_target_reached:
                self.publish_status({"action": "waypoint_reached"})
        self._last_target_reached = telemetry.get("target_reached", False)

    def update_drone(self):
        """Update mock drone state."""
        self.drone.update()

    def publish_status(self, status_data):
        """Publish navigation status update."""
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_publisher.publish(status_msg)

        # Also log the status
        self.get_logger().debug(f"Status: {status_data}")


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

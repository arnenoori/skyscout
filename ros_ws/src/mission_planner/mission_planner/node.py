#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2DArray
import json
import enum
import math


class MissionState(enum.Enum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    PAUSED = 3
    COMPLETED = 4
    ABORTED = 5


class MissionPlannerNode(Node):
    """
    ROS2 node that executes structured mission plans received from the LLM agent.
    Coordinates with navigation_bridge and perception nodes to execute missions.
    """

    def __init__(self):
        super().__init__("mission_planner_node")

        # State management
        self.current_state = MissionState.IDLE
        self.current_mission = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.detected_objects = []

        # Create subscribers
        self.mission_subscriber = self.create_subscription(
            String, "mission_plan", self.mission_callback, 10
        )

        self.detection_subscriber = self.create_subscription(
            Detection2DArray, "detected_objects", self.detection_callback, 10
        )

        # Create publishers
        self.waypoint_publisher = self.create_publisher(
            Point, "navigation/waypoint", 10
        )

        self.mission_status_publisher = self.create_publisher(
            String, "mission_status", 10
        )

        # Navigation control publishers
        self.arm_publisher = self.create_publisher(Bool, "navigation/arm", 10)
        self.takeoff_publisher = self.create_publisher(
            Float32, "navigation/takeoff", 10
        )
        self.land_publisher = self.create_publisher(Bool, "navigation/land", 10)
        self.rtl_publisher = self.create_publisher(Bool, "navigation/rtl", 10)

        # Create timer for mission execution
        self.execution_timer = self.create_timer(1.0, self.execute_mission_step)

        self.get_logger().info("Mission Planner Node initialized")

    def mission_callback(self, msg):
        """Process new mission plan from LLM agent."""
        try:
            mission_data = json.loads(msg.data)
            self.get_logger().info(
                f"Received mission plan: {mission_data['mission_type']}"
            )

            # Validate mission plan
            if self.validate_mission_plan(mission_data):
                self.current_mission = mission_data
                self.current_state = MissionState.PLANNING
                self.publish_status("Mission accepted, beginning planning")
            else:
                self.publish_status("Invalid mission plan received")

        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse mission plan JSON")
            self.publish_status("Error: Invalid mission format")

    def detection_callback(self, msg):
        """Process detection results from perception node."""
        if self.current_state == MissionState.EXECUTING and self.current_mission:
            # Check if detected objects match mission targets
            for detection in msg.detections:
                for result in detection.results:
                    self.get_logger().info(
                        f"Detected: {result.id} with confidence {result.score}"
                    )
                    # TODO: Match against mission target_description

    def execute_mission_step(self):
        """Execute mission steps based on current state."""
        if self.current_state == MissionState.PLANNING:
            self.plan_mission()
        elif self.current_state == MissionState.ARMING:
            self.arm_drone()
        elif self.current_state == MissionState.TAKEOFF:
            self.takeoff()
        elif self.current_state == MissionState.EXECUTING:
            self.execute_mission()
        elif self.current_state == MissionState.LANDING:
            self.land_drone()

    def plan_mission(self):
        """Plan mission execution based on mission type and parameters."""
        if not self.current_mission:
            return

        self.get_logger().info(
            f"Planning {self.current_mission['mission_type']} mission"
        )

        # Generate waypoints based on flight pattern
        params = self.current_mission["parameters"]
        altitude = params["altitude"]
        coverage_area = params["coverage_area"]

        if self.current_mission["flight_pattern"] == "grid":
            self.waypoints = self._generate_grid_pattern(coverage_area, altitude)
        elif self.current_mission["flight_pattern"] == "spiral":
            self.waypoints = self._generate_spiral_pattern(coverage_area, altitude)
        elif self.current_mission["flight_pattern"] == "perimeter":
            self.waypoints = self._generate_perimeter_pattern(coverage_area, altitude)
        else:
            # Default to single point
            self.waypoints = [Point(x=0, y=0, z=altitude)]

        self.current_waypoint_index = 0
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints")

        # Start execution
        self.current_state = MissionState.ARMING
        self.publish_status(
            f"Mission planned with {len(self.waypoints)} waypoints, arming drone"
        )

    def arm_drone(self):
        """Arm the drone for flight."""
        self.get_logger().info("Arming drone")
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_publisher.publish(arm_msg)

        # Move to takeoff state after a delay
        # In real implementation, wait for arm confirmation
        self.current_state = MissionState.TAKEOFF
        self.publish_status("Drone armed, initiating takeoff")

    def takeoff(self):
        """Command drone to takeoff."""
        altitude = self.current_mission["parameters"]["altitude"]
        self.get_logger().info(f"Taking off to {altitude}m")

        takeoff_msg = Float32()
        takeoff_msg.data = altitude
        self.takeoff_publisher.publish(takeoff_msg)

        # Move to executing state after a delay
        # In real implementation, wait for altitude reached
        self.current_state = MissionState.EXECUTING
        self.publish_status(f"Takeoff initiated to {altitude}m")

    def execute_mission(self):
        """Execute the current mission step."""
        if self.current_waypoint_index >= len(self.waypoints):
            # Mission complete
            self.get_logger().info("All waypoints visited, mission complete")
            self.current_state = MissionState.LANDING
            self.publish_status("Mission complete, landing")
            return

        # Send current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        self.waypoint_publisher.publish(waypoint)
        self.get_logger().info(
            f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: "
            f"x={waypoint.x:.1f}, y={waypoint.y:.1f}, z={waypoint.z:.1f}"
        )

        # Move to next waypoint (in real implementation, wait for waypoint reached)
        self.current_waypoint_index += 1

    def land_drone(self):
        """Land the drone."""
        self.get_logger().info("Landing drone")
        land_msg = Bool()
        land_msg.data = True
        self.land_publisher.publish(land_msg)

        # Reset state
        self.current_state = MissionState.IDLE
        self.current_mission = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.publish_status("Drone landed, mission complete")

    def validate_mission_plan(self, mission_data):
        """Validate mission plan structure and safety parameters."""
        required_fields = [
            "mission_type",
            "target_description",
            "flight_pattern",
            "parameters",
            "safety",
        ]

        # Check required fields
        for field in required_fields:
            if field not in mission_data:
                self.get_logger().error(f"Missing required field: {field}")
                return False

        # Validate altitude limits
        altitude = mission_data["parameters"].get("altitude", 0)
        if altitude < 10 or altitude > 120:
            self.get_logger().error(f"Invalid altitude: {altitude}")
            return False

        return True

    def publish_status(self, status_message):
        """Publish mission status update."""
        status_msg = String()
        status_msg.data = json.dumps(
            {"state": self.current_state.name, "message": status_message}
        )
        self.mission_status_publisher.publish(status_msg)

    def _generate_grid_pattern(self, coverage_area: dict, altitude: float) -> list:
        """Generate grid pattern waypoints."""
        waypoints = []
        radius = coverage_area.get("radius", 100)

        # Simple grid with 20m spacing
        spacing = 20
        num_lines = int(radius * 2 / spacing)

        for i in range(num_lines):
            y = -radius + i * spacing
            if i % 2 == 0:
                # Left to right
                waypoints.append(Point(x=-radius, y=y, z=altitude))
                waypoints.append(Point(x=radius, y=y, z=altitude))
            else:
                # Right to left
                waypoints.append(Point(x=radius, y=y, z=altitude))
                waypoints.append(Point(x=-radius, y=y, z=altitude))

        return waypoints

    def _generate_spiral_pattern(self, coverage_area: dict, altitude: float) -> list:
        """Generate spiral pattern waypoints."""
        waypoints = []
        radius = coverage_area.get("radius", 100)

        # Archimedean spiral
        num_points = 20
        for i in range(num_points):
            angle = i * 2 * math.pi / 5  # 5 revolutions
            r = (i / num_points) * radius
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            waypoints.append(Point(x=x, y=y, z=altitude))

        return waypoints

    def _generate_perimeter_pattern(self, coverage_area: dict, altitude: float) -> list:
        """Generate perimeter pattern waypoints."""
        waypoints = []
        radius = coverage_area.get("radius", 100)

        # Square perimeter
        waypoints.append(Point(x=-radius, y=-radius, z=altitude))
        waypoints.append(Point(x=radius, y=-radius, z=altitude))
        waypoints.append(Point(x=radius, y=radius, z=altitude))
        waypoints.append(Point(x=-radius, y=radius, z=altitude))
        waypoints.append(Point(x=-radius, y=-radius, z=altitude))  # Close the loop

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2DArray
import json
import enum


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
        elif self.current_state == MissionState.EXECUTING:
            self.execute_mission()

    def plan_mission(self):
        """Plan mission execution based on mission type and parameters."""
        if not self.current_mission:
            return

        self.get_logger().info(
            f"Planning {self.current_mission['mission_type']} mission"
        )

        # Generate waypoints based on flight pattern
        if self.current_mission["flight_pattern"] == "grid":
            # TODO: Implement grid pattern generation
            pass
        elif self.current_mission["flight_pattern"] == "spiral":
            # TODO: Implement spiral pattern generation
            pass

        self.current_state = MissionState.EXECUTING
        self.publish_status("Mission planned, beginning execution")

    def execute_mission(self):
        """Execute the current mission step."""
        # TODO: Implement actual mission execution logic
        # For now, just publish a test waypoint
        waypoint = Point()
        waypoint.x = 10.0
        waypoint.y = 10.0
        waypoint.z = self.current_mission["parameters"]["altitude"]

        self.waypoint_publisher.publish(waypoint)
        self.get_logger().debug("Published waypoint")

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

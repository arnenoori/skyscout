#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


class PerceptionNode(Node):
    """
    ROS2 node for real-time object detection using YOLO or similar models.
    Processes camera images and publishes detected objects.
    """

    def __init__(self):
        super().__init__("perception_node")

        # Create subscriber for camera images
        self.image_subscriber = self.create_subscription(
            Image, "camera/image_raw", self.image_callback, 10
        )

        # Create publisher for detected objects
        self.detection_publisher = self.create_publisher(
            Detection2DArray, "detected_objects", 10
        )

        # Parameters for detection
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("model_path", "")

        self.confidence_threshold = self.get_parameter("confidence_threshold").value

        self.get_logger().info("Perception Node initialized")

    def image_callback(self, msg):
        """Process incoming camera image and detect objects."""
        self.get_logger().debug("Received image for processing")

        # TODO: Implement actual YOLO or other detection model
        # For now, publish mock detections
        detections = self.mock_detect_objects(msg)

        if detections.detections:
            self.detection_publisher.publish(detections)
            self.get_logger().info(f"Published {len(detections.detections)} detections")

    def mock_detect_objects(self, image_msg):
        """
        Mock object detection for testing.
        TODO: Replace with actual YOLO implementation
        """
        detections_msg = Detection2DArray()
        detections_msg.header = image_msg.header

        # Create a mock detection
        detection = Detection2D()
        detection.header = image_msg.header

        # Mock bounding box
        detection.bbox.center.x = 320.0
        detection.bbox.center.y = 240.0
        detection.bbox.size_x = 100.0
        detection.bbox.size_y = 100.0

        # Mock classification result
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = "vehicle"
        hypothesis.score = 0.85
        detection.results.append(hypothesis)

        detections_msg.detections.append(detection)

        return detections_msg


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# Perception

Real-time object detection using YOLO or similar models for identifying mission targets.

## Overview

Processes camera images to detect and track objects specified in the mission plan.

## Topics

**Subscribers:**
- `/camera/image_raw` (sensor_msgs/Image) - Camera feed

**Publishers:**
- `/detected_objects` (vision_msgs/Detection2DArray) - Detected objects with bounding boxes

## Parameters

- `confidence_threshold` - Minimum detection confidence (default: 0.5)
- `model_path` - Path to YOLO model weights

## Performance

Optimized for Raspberry Pi 5:
- Target: 10 FPS processing
- Uses model quantization
- Supports batch processing

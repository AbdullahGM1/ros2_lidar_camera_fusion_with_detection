# ROS2 Camera-Lidar Fusion with Object Detection

This ROS2 package integrates 360-degree Lidar point cloud data with camera imagery and performs real-time object detection using YOLOv8. It projects Lidar data onto the image plane and fuses it with the camera stream, providing enhanced object tracking and distance estimation for robotic applications.

---

## Table of Contents
1. [Features](#features)
2. [Installation](#installation)
3. [Usage](#usage)
4. [Nodes](#nodes)
5. [Topics](#topics)
6. [Launch Files](#launch-files)
7. [TODO / Future Improvements](#todo--future-improvements)
8. [Contributing](#contributing)

---

## Features
- **Lidar-Camera Sensor Fusion**: Projects 360-degree Lidar data onto the image plane.
- **Real-Time Object Detection**: Uses YOLOv8 for object detection and tracking.
- **Distance Estimation**: Publishes the average distance to detected objects from the Lidar data.
- **ROS2 Compatibility**: Designed for ROS2 Humble with full support for simulation environments.

---

## Installation

### Prerequisites
- **ROS2 Humble**: Ensure you have ROS2 Humble installed on your machine. [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **YOLOv8**: Follow the instructions to set up YOLOv8 in ROS2 for object detection.

### Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ros2_lidar_camera_fusion_with_detection.git

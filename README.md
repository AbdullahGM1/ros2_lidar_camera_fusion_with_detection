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
- **Lidar-Camera Sensor Fusion**: Transforms 360-degree Lidar data onto the image plane.
- **Point Cloud Overlay within Detected Objects**: Overlays Lidar point cloud data within the bounding boxes of detected objects in the camera image.
- **Position Estimation**: Publishes the estimate position of the detected objects with recpect to the camera frame.
- **ROS2 Compatibility**: Designed for ROS2 Humble with full support for simulation environments.

---

## Installation

### Prerequisites
- **ROS2 Humble**: Ensure you have ROS2 Humble installed on your machine. [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **YOLOv8**: Follow the instructions to set up YOLOv8 in ROS2 for object detection. [Installation Guide](https://github.com/mgonzs13/yolov8_ros) - You can use any Yolo Model.

### Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ros2_lidar_camera_fusion_with_detection.git
```
### Install Dependencies
Run `rosdep` to install any missing dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
### Build the Package
After cloning and installing dependencies, build your package:
```bash
colcon build --packages-select ros2_lidar_camera_fusion_with_detection
```
---

## Usage
### Prerequisites


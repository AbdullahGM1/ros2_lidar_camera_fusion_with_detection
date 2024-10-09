# ROS2 Camera-Lidar Fusion Package

## Description

This ROS2 package fuses 360-degree lidar and camera data for enhanced object tracking. It transforms lidar point clouds from the lidar frame to the camera frame and overlays the points within detected object bounding boxes onto the image. The system estimates 3D positions by averaging the point cloud (x, y, z) within each bounding box, enabling real-time tracking and position estimation for multiple objects simultaneously.

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

- **Lidar to Camera Frame Transformation**: Converts lidar point cloud data into the camera's reference frame.
- **Object Detection Overlay**: Overlays lidar points corresponding to detected objects (within bounding boxes) onto the camera image.
- **3D Position Estimation**: Calculates the average (x, y, z) of point clouds within object bounding boxes to estimate 3D positions.
- **Multi-Object Tracking**: Simultaneously tracks and estimates positions for multiple detected objects in real-time.
- **ROS2 Integration**: Fully compatible with ROS2 for seamless integration in robotics applications.
  
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
### Run the Node
To run the package with your custom launch file (make sure you specify it):
```bash
ros2 run ros2_lidar_camera_fusion_with_detection lidar_camera_fusion_with_detection
```
---
## Node
### lidar_camera_fusion_node
This node transforms Lidar point cloud data onto the camera image frame, and overlay

```bash
ros2 run ros2_lidar_camera_fusion_with_detection lidar_camera_fusion_with_detection
```

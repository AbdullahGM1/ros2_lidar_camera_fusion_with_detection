# ROS2 Camera-Lidar Fusion with Object Detection

This ROS2 package integrates data from a 360-degree lidar and a camera to achieve enhanced object tracking through sensor fusion. The package transforms all lidar point cloud data from the lidar frame to the camera frame. It then overlays the point cloud data corresponding to the detected objects (within their bounding boxes) onto the image.

  The fusion process enables the system to estimate the 3D position of detected objects by calculating the average point cloud (x, y, z) within each bounding box. This allows for real-time tracking and provides accurate position estimates for one or more objects simultaneously.
  
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
- **Transformation of lidar point cloud data to the camera frame**
- **Overlay of point cloud data within detected object bounding boxes onto the image**
- **3D position estimation based on the average point cloud of detected objects**
- **Simultaneous tracking and position estimation for multiple detected objects**
- **ROS2 Compatibility**

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

# ROS2 Camera-Lidar Fusion Package

## Description

This ROS2 package fuses 360-degree lidar and camera data for enhanced object tracking. It transforms lidar point clouds from the lidar frame to the camera frame and overlays the points within the detected object bounding boxes onto the image. The system estimates the 3D positions of the detected object by averaging the point cloud (x, y, z) within the bounding box. It also publishes the points inside these bounding boxes as point cloud data. This enables real-time tracking and position estimation for multiple objects.

---

## Table of Contents
1. [Demonstration](#demonstration)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Node](#node)
6. [Contributing](#contributing)

---
## Demonstration

### Lidar-Camera Fusion in Action


<p align="center">
- Overlays the detected object point cloud points onto the image:
</p>


<p align="center">
  <img src="images/1.png" alt="Lidar-Camera Fusion Example 1" width="500"/>
  <img src="images/2.png" alt="Lidar-Camera Fusion Example 2" width="500"/>
</p>

<p align="center">
  <img src="images/camera_lidar_detection_sensor_fusion.gif" alt="Lidar-Camera Fusion in Action gif" width="600"/>
</p>

<p align="center">
- Publish the points within the detected object bounding boxes as point cloud points.
</p>


<p align="center">
  <img src="images/Detected_Object_Point_Cloud(3).png" alt="Lidar-Camera Fusion Example 1" width="500"/>
  <img src="images/Detected_Object_Point_Cloud.gif" alt="Lidar-Camera Fusion Example 2" width="500"/>
</p>

---

## Features

- **Lidar to Camera Frame Transformation**: Fuse lidar point cloud data into the camera reference frame.
- **Object Detection Overlay**: Overlays lidar points corresponding to detected objects (within bounding boxes) onto the camera image.
- **3D Position Estimation**: Calculates the average (x, y, z) of point clouds within object bounding boxes to estimate 3D positions.
- **Multi-Object Tracking**: Simultaneously tracks and estimates positions for multiple detected objects in real-time.
- **Detected Object Point Cloud Publishing**: Publishes lidar points overlaid onto the image within detected object bounding boxes as a separate point cloud.
- **ROS2 Integration**: Fully compatible with ROS2 for seamless integration in robotics applications.
  
---

## Installation

### Prerequisites
- **ROS2 Humble**: Ensure you have ROS2 Humble installed on your machine. [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **YOLOvX**: Follow the instructions to set up YOLOvX in ROS2 for object detection. [Installation Guide](https://github.com/mgonzs13/yolov8_ros) 

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

### Modifying the Code for Your Setup
Before running the package, make sure to modify the following parts of the code in `lidar_camera_fusion_with_detection.py` to match your setup:

1. **Set the Transformation Matrix**: Update the transformation matrix between the Lidar and the camera based on your setup in the `/ros2_lidar_camera_fusion_with_detection/config` folder.
   Example:
   ```python
     ```yaml
transformation_matrix:
  - [0, -1, 0, 0.1]
  - [0, 0, -1, 0]
  - [1, 0, 0, 0]
  - [0, 0, 0, 1]


   ```
3. **Specify the Distance Range**: Set the distance range for points that should be transformed. In this example, only points between 0.5 and 10 meters are considered.
   Example:
   ```python
    if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and (0.1 <= x <= 10.0):
                # Convert point to PointStamped for transformation
                point_stamped = PointStamped()
                point_stamped.point.x = x
                point_stamped.point.y = y
                point_stamped.point.z = z
                point_stamped.header.frame_id = self.lidar_frame_
                points.append(point_stamped)
   ```
### Build the Package
After modifying the code, build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_lidar_camera_fusion_with_detection
```
### Source the Workspace
Before running the package, ensure you source the workspace to have access to the built packages:

For **Bash**:
```bash
source ~/ros2_ws/install/setup.bash
```

### Run the Node
To run the package with your custom launch file (make sure you specify it):
```bash
ros2 run ros2_lidar_camera_fusion_with_detection lidar_camera_fusion_with_detection
```

---
## Node

### `lidar_camera_fusion_node`

This node fuses lidar point cloud data onto the camera image frame and overlays the points within detected object bounding boxes onto the image. Also, publishing the points within the bounding box as point cloud points.

#### Subscribed Topics:

-**`/scan/points`**: Lidar point cloud data.

-**`/interceptor/gimbal_camera`**: Camera image output.

-**`/interceptor/gimbal_camera_info`**: Camera info for the Camera Intrinsic.

-**`/yolo/tracking`**: Detected objects with bounding boxes.


#### Published Topics:

-**`/image_lidar`**: Image with projected Lidar points.

-**`/detected_object_distance`**: Average distance from the detected objects to the camera frame.

-**`/detected_object_pointcloud`**: Points inside the bounding boxes are published as point cloud points.

---
## Contributing

Feel free to contribute to this project by creating pull requests or opening issues.

import rclpy
from rclpy.node import Node
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import PointCloud2, CameraInfo, Image, PointField
import sensor_msgs_py.point_cloud2 as pc2
import math
import cv2
from cv_bridge import CvBridge
from yolov8_msgs.msg import DetectionArray
from geometry_msgs.msg import Point
import struct

class LidarToImageProjection(Node):
    def __init__(self):
        super().__init__('lidar_to_image_projection')

        # Load transformation matrix from YAML file
        package_share_directory = get_package_share_directory('ros2_lidar_camera_fusion_with_detection')
        yaml_file_path = os.path.join(package_share_directory, 'config', 'setup_config.yaml')

        # Load transformation matrix and depth_range from the YAML file
        yaml_data = self.load_yaml_file(yaml_file_path)
        self.transformation_matrix = np.array(yaml_data['transformation_matrix'])
        self.depth_range_min = yaml_data['depth_range']['min']
        self.depth_range_max = yaml_data['depth_range']['max']

        self.get_logger().info(f'Loaded transformation matrix:\n{self.transformation_matrix}')
        self.get_logger().info(f'Loaded depth_range: min={self.depth_range_min}, max={self.depth_range_max}')

        # Image to hold the current frame
        self.current_image = None

        # OpenCV Bridge for converting ROS Image to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to Lidar Point Cloud
        self.subscription = self.create_subscription(PointCloud2, '/scan/points', self.pointcloud_callback, 10)

        # Subscribe to CameraInfo
        self.camera_info_subscription = self.create_subscription(CameraInfo, '/interceptor/gimbal_camera_info', self.camera_info_callback, 10)

        # Subscribe to Camera Image
        self.image_subscription = self.create_subscription(Image, '/interceptor/gimbal_camera', self.image_callback, 10)

        # Subscribe to Detected Object Bounding Box
        self.detection_subscription = self.create_subscription(DetectionArray, '/yolo/tracking', self.detection_callback, 10)

        # Publisher for Image with projected Lidar points
        self.image_publisher = self.create_publisher(Image, '/image_lidar', 10)

        # Publisher for publishing the detected objects' positions
        self.distance_publisher = self.create_publisher(Point, '/detected_object_position', 10)

        # Publisher for detected object's point cloud
        self.object_pointcloud_publisher = self.create_publisher(PointCloud2, '/detected_object_pointcloud', 10)


    def load_yaml_file(self, yaml_file):
        """
        Loads data from a YAML file.
        """
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file)

    def camera_info_callback(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_width, self.image_height = msg.width, msg.height

    def detection_callback(self, msg):
        self.bounding_boxes = [
            (
                detection.bbox.center.position.x - detection.bbox.size.x / 2,
                detection.bbox.center.position.y - detection.bbox.size.y / 2,
                detection.bbox.center.position.x + detection.bbox.size.x / 2,
                detection.bbox.center.position.y + detection.bbox.size.y / 2
            )
            for detection in msg.detections
        ]

    def pointcloud_callback(self, msg):

        if self.current_image is None or any(val is None for val in [self.fx, self.fy, self.cx, self.cy]):
            self.get_logger().warning("Camera intrinsics or current image not available.")
            return

        # Step 1: Extract points
        points = self.point_cloud_data_extraction(msg)

        # Process the points
        image_with_points = self.current_image.copy()
        bbox_values = {i: {'x': [], 'y': [], 'z': [], 'points': []} for i in range(len(self.bounding_boxes))}

        for point in points:
            x, y, z = point
            transformed_point, u, v = self.transformation(x, y, z, self.transformation_matrix)
            self.point_cloud_within_bounding_box(u, v, transformed_point, bbox_values, image_with_points, point, msg.header)

        # Step 4: Detected object position
        self.detected_object_position(bbox_values, msg.header)

        image_msg = self.bridge.cv2_to_imgmsg(image_with_points, encoding='bgr8')
        self.image_publisher.publish(image_msg)


    def point_cloud_data_extraction(self, msg):
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[0], point[1], point[2]

            # Check if x, y, or z are finite values and if x (depth) is within the loaded depth range from YAML
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and self.depth_range_min < x < self.depth_range_max:
                points.append([x, y, z])
        return points

    def transformation(self, x, y, z, transformation_matrix):
        # Convert the 3D points to homogeneous coordinates
        homogeneous_point = np.array([x, y, z, 1.0])
        transformed_point = np.dot(transformation_matrix, homogeneous_point)

        # Project the point onto the image plane using camera intrinsics from the camera frame
        u = (self.fx * transformed_point[0] / transformed_point[2]) + self.cx
        v = (self.fy * transformed_point[1] / transformed_point[2]) + self.cy

        return transformed_point, u, v

    def point_cloud_within_bounding_box(self, u, v, transformed_point, bbox_values, image_with_points, point, header):
        # Check if the projected point is within image bounds
        if 0 <= u < self.image_width and 0 <= v < self.image_height:
            for i, bbox in enumerate(self.bounding_boxes):
                top_left_x, top_left_y, bottom_right_x, bottom_right_y = bbox
                if top_left_x <= u <= bottom_right_x and top_left_y <= v <= bottom_right_y:
                    bbox_values[i]['x'].append(transformed_point[1])
                    bbox_values[i]['y'].append(transformed_point[2])
                    bbox_values[i]['z'].append(transformed_point[0])
                    bbox_values[i]['points'].append(point)
                    cv2.circle(image_with_points, (int(u), int(v)), 3, (0, 0, 255), -1)

        # After checking all points, publish the point cloud for the detected objects point cloud
        for i, values in bbox_values.items():
            if values['points']:  # Avoid empty lists
                # Create PointCloud2 message from the points within the bounding box
                object_pointcloud = self.create_pointcloud2_msg(values['points'], header)
                self.object_pointcloud_publisher.publish(object_pointcloud)

    def detected_object_position(self, bbox_values, header):
        for i, values in bbox_values.items():
            if values['points']:  # Avoid empty lists
                avg_x = sum(values['x']) / len(values['x'])
                avg_y = sum(values['y']) / len(values['y'])
                avg_z = sum(values['z']) / len(values['z'])

                self.get_logger().info(
                    f"Bounding Box {i + 1}: Detected Object Position (x, y, z) = "
                    f"({avg_z:.2f}, {avg_x:.2f}, {avg_y:.2f}) meters"
                )

                distance_msg = Point()
                distance_msg.x = avg_z  # transformed x is now z
                distance_msg.y = avg_x  # transformed y is now x
                distance_msg.z = avg_y  # transformed z is now y
                self.distance_publisher.publish(distance_msg)

    def create_pointcloud2_msg(self, points, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud_data = b''.join([struct.pack('fff', point[0], point[1], point[2]) for point in points])

        return PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 3 floats * 4 bytes each
            row_step=12 * len(points),
            data=point_cloud_data
        )

def main(args=None):
    rclpy.init(args=args)
    node = LidarToImageProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

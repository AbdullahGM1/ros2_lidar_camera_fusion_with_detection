import rclpy
from rclpy.node import Node
import numpy as np
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

        # Initialize camera intrinsic values
        self.fx, self.fy, self.cx, self.cy = None, None, None, None  # Camera intrinsics

        # Image to hold the current frame
        self.current_image = None

        # List to hold bounding boxes
        self.bounding_boxes = []

        # OpenCV Bridge for converting ROS Image to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to Lidar Point Cloud
        self.subscription = self.create_subscription(PointCloud2,'/scan/points',self.pointcloud_callback,10)

        # Subscribe to CameraInfo
        self.camera_info_subscription = self.create_subscription(CameraInfo,'/interceptor/gimbal_camera_info',self.camera_info_callback,10)

        # Subscribe to Camera Image
        self.image_subscription = self.create_subscription(Image,'/interceptor/gimbal_camera',self.image_callback,10)

        # Subscribe to Detected Object Bounding Box
        self.detection_subscription = self.create_subscription(DetectionArray,'/yolo/tracking',self.detection_callback,10)

        # Publisher for Image with projected Lidar points
        self.image_publisher = self.create_publisher(Image,'/image_lidar',10)

        # Publisher for publishing the detected objects' positions
        self.distance_publisher = self.create_publisher(Point,'/detected_object_position',10)

        # Publisher for detected object's point cloud
        self.object_pointcloud_publisher = self.create_publisher(PointCloud2,'/detected_object_pointcloud',10)

    def camera_info_callback(self, msg):
        # Extract the camera intrinsic parameters from CameraInfo
        self.fx = msg.k[0]  # Focal length in x (fx)
        self.fy = msg.k[4]  # Focal length in y (fy)
        self.cx = msg.k[2]  # Principal point in x (cx)
        self.cy = msg.k[5]  # Principal point in y (cy)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Store the image width and height
        self.image_width = msg.width
        self.image_height = msg.height

    def detection_callback(self, msg):
        # Extract bounding boxes from the YOLO detections
        self.bounding_boxes = []

        for detection in msg.detections:
            bbox_center_x = detection.bbox.center.position.x
            bbox_center_y = detection.bbox.center.position.y
            bbox_size_x = detection.bbox.size.x
            bbox_size_y = detection.bbox.size.y

            # Calculate the bounding box corners
            top_left_x = bbox_center_x - bbox_size_x / 2
            top_left_y = bbox_center_y - bbox_size_y / 2
            bottom_right_x = bbox_center_x + bbox_size_x / 2
            bottom_right_y = bbox_center_y + bbox_size_y / 2

            self.bounding_boxes.append((top_left_x, top_left_y, bottom_right_x, bottom_right_y))

    def pointcloud_callback(self, msg):
        
        # Check if camera intrinsics and image are received
        if self.current_image is None:
            self.get_logger().warning("Current image is not available yet.")
            return
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().warning("Camera intrinsics are not available yet.")
            return

        # Transformation Matrix Between the Lidar and the camera
        T_lidar_to_camera = np.array([
            [0, -1, 0, 0.1],
            [0,  0, -1, 0],
            [1,  0,  0, 0],
            [0,  0,  0, 1]
        ])

        # Create a copy of the image to draw the points
        image_with_points = self.current_image.copy()

        # Create a dictionary to accumulate points' (x, y, z) values for each bounding box
        bbox_values = {
            i: {'x': [], 'y': [], 'z': [], 'points': []}
            for i in range(len(self.bounding_boxes))
        }

        # Step 1: Point cloud data extraction
        points = self.point_cloud_data_extraction(msg)

        # Step 2 and 3: Process each point
        for point in points:
            x, y, z = point[0], point[1], point[2]

            # Step 2: Transformation
            transformed_point, u, v = self.transformation(x, y, z, T_lidar_to_camera)

            # Step 3: Point cloud within the bounding box
            self.point_cloud_within_bounding_box(
                u, v, transformed_point, bbox_values, image_with_points, point, msg.header
            )

        # Step 4: Detected object position
        self.detected_object_position(bbox_values, msg.header)

        # Convert the OpenCV image back to ROS Image message and publish
        image_msg = self.bridge.cv2_to_imgmsg(image_with_points, encoding='bgr8')
        self.image_publisher.publish(image_msg)

    def point_cloud_data_extraction(self, msg):
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[0], point[1], point[2]

            # Check if x, y, or z are finite values (not inf) and if x (Depth) is within the range
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and 0.5 < x < 10:
                points.append([x, y, z])
        return points

    def transformation(self, x, y, z, T_lidar_to_camera):

        # Convert the 3D point to homogeneous coordinates
        homogeneous_point = [x, y, z, 1.0]
        points_np = np.array(homogeneous_point)

        # Transform the point from lidar to camera frame
        transformed_point = np.dot(T_lidar_to_camera, points_np)

        # Project the point onto the image plane using camera intrinsics from the camera frame
        u = (self.fx * transformed_point[0] / transformed_point[2]) + self.cx
        v = (self.fy * transformed_point[1] / transformed_point[2]) + self.cy

        return transformed_point, u, v

    def point_cloud_within_bounding_box(self, u, v, transformed_point, bbox_values, image_with_points, point, header):

        # Check if the projected point is within image bounds
        if 0 <= u < self.image_width and 0 <= v < self.image_height:
            # Check if the projected point is within any bounding box
            for i, bbox in enumerate(self.bounding_boxes):
                top_left_x, top_left_y, bottom_right_x, bottom_right_y = bbox
                if (
                    top_left_x <= u <= bottom_right_x
                    and top_left_y <= v <= bottom_right_y
                ):
                    # Add the (x, y, z) values of the point to the corresponding bounding box
                    bbox_values[i]['x'].append(transformed_point[1])  # lidar y becomes camera x
                    bbox_values[i]['y'].append(transformed_point[2])  # lidar z becomes camera y
                    bbox_values[i]['z'].append(transformed_point[0])  # lidar x becomes camera z
                    bbox_values[i]['points'].append(point)  # Store original point

                    # Draw the point on the image (as a small circle)
                    cv2.circle(
                        image_with_points, (int(u), int(v)), 3, (0, 0, 255), -1
                    )  # Red dot
                    break

        # After checking all points, publish the point cloud for the detected objects point cloud
        for i, values in bbox_values.items():
            if values['points']:  # Avoid empty lists
                # Create PointCloud2 message from the points within the bounding box
                object_pointcloud = self.create_pointcloud2_msg(values['points'], header)
                self.object_pointcloud_publisher.publish(object_pointcloud)

    def detected_object_position(self, bbox_values, header):

        # Publish the point cloud for each detected object and print its position
        for i, values in bbox_values.items():
            if values['points']:  # Avoid empty lists
                # Calculate the average position
                avg_x = sum(values['x']) / len(values['x'])
                avg_y = sum(values['y']) / len(values['y'])
                avg_z = sum(values['z']) / len(values['z'])

                # Print the position of the detected object
                self.get_logger().info(
                    f"Bounding Box {i + 1}: Detected Object Position (x, y, z) = "
                    f"({avg_z:.2f}, {avg_x:.2f}, {avg_y:.2f}) meters"
                )

                # Create and publish the detected object point cloud (x, y, z) with respect to the camera frame
                distance_msg = Point()
                distance_msg.x = avg_z  # transformed x is now z
                distance_msg.y = avg_x  # transformed y is now x
                distance_msg.z = avg_y  # transformed z is now y
                self.distance_publisher.publish(distance_msg)

    def create_pointcloud2_msg(self, points, header):
        # Publishing the (x,y,z) point cloud with recpect to the lidar frame. 
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        # Pack point cloud data into a PointCloud2 message
        point_cloud_data = []
        for point in points:
            packed = struct.pack('fff', point[0], point[1], point[2])
            point_cloud_data.append(packed)
        point_cloud_data = b''.join(point_cloud_data)

        # Create and return the PointCloud2 message
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

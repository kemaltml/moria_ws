#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class GlassDetectionLiDARFilter(Node):
    def __init__(self):
        super().__init__('glass_lidar_filter_node')
        
        # Subscriptions
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera_rgb/image_raw', self.camera_callback, 10)
        
        # Publisher
        self.filtered_lidar_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        
        # Bridge for OpenCV and ROS
        self.bridge = CvBridge()
        self.latest_image = None  # To store the latest camera frame

    def camera_callback(self, msg):
        """
        Process the incoming camera frame to detect glass regions.
        """
        # Convert ROS Image message to OpenCV format
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detect_glass_regions(self, image):
        """
        Detect glass regions in the camera frame using edge and texture analysis.
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours of potential glass regions
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours based on texture analysis
        glass_regions = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 5000:  # Threshold for large reflective regions
                # Crop the region
                x, y, w, h = cv2.boundingRect(contour)
                cropped = gray[y:y+h, x:x+w]
                
                # Analyze texture using Laplacian variance
                laplacian_var = cv2.Laplacian(cropped, cv2.CV_64F).var()
                if laplacian_var < 15:  # Low texture indicates potential glass
                    glass_regions.append((x, y, w, h))
        
        return glass_regions

    def map_lidar_to_camera(self, angle, range_val, camera_width, camera_height, fov):
        """
        Map LiDAR point (angle, range) to pixel coordinates in the camera frame.
        """
        # Convert polar to cartesian coordinates
        x = range_val * np.cos(angle)
        y = range_val * np.sin(angle)
        
        # Convert to camera frame (assuming camera is aligned with LiDAR)
        fx = fy = camera_width / (2 * np.tan(fov / 2))
        cx = camera_width / 2
        cy = camera_height / 2
        
        # Project to pixel coordinates
        u = int(fx * x / range_val + cx)
        v = int(fy * y / range_val + cy)
        
        return u, v

    def lidar_callback(self, msg):
        """
        Process the LiDAR data and filter out points falling in detected glass regions.
        """
        if self.latest_image is None:
            self.get_logger().warning("No camera frame received yet!")
            return
        
        # Detect glass regions in the latest camera frame
        glass_regions = self.detect_glass_regions(self.latest_image)
        
        # Get camera dimensions
        camera_height, camera_width, _ = self.latest_image.shape
        
        # Field of view of the camera (assumed horizontal FOV of 90 degrees)
        fov = np.pi / 2
        
        filtered_ranges = []
        for i, range_val in enumerate(msg.ranges):
            if range_val < msg.range_min or range_val > msg.range_max:
                filtered_ranges.append(range_val)
                continue
            
            # Map LiDAR point to camera pixel
            angle = msg.angle_min + i * msg.angle_increment
            u, v = self.map_lidar_to_camera(angle, range_val, camera_width, camera_height, fov)
            
            # Check if the point falls within any glass region
            is_glass = any(x <= u <= x+w and y <= v <= y+h for (x, y, w, h) in glass_regions)
            
            # Filter the range if it falls in a glass region
            if is_glass:
                filtered_ranges.append(float('inf'))
            else:
                filtered_ranges.append(range_val)

        # Publish the filtered LiDAR data
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = filtered_ranges
        self.filtered_lidar_pub.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = GlassDetectionLiDARFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

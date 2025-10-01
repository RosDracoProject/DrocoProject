#!/usr/bin/env python3
# Copyright (c) 2024, Gazebo LiDAR Simulation

"""
LiDAR Point Cloud Subscriber for testing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from point_cloud_transport_py import PointCloudTransport
import time
import numpy as np


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        
        # Create point cloud transport
        self.transport = PointCloudTransport(self)
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscriber with zlib compression
        self.subscriber = self.transport.subscribe(
            "/lidar/points",
            self.point_cloud_callback,
            qos_profile=qos_profile,
            transport='zlib'
        )
        
        # Statistics
        self.message_count = 0
        self.start_time = time.time()
        self.total_points = 0
        
        self.get_logger().info("LiDAR Subscriber started")
        
    def point_cloud_callback(self, point_cloud: PointCloud2):
        """Callback for received point cloud data"""
        self.message_count += 1
        
        # Calculate statistics
        num_points = point_cloud.width * point_cloud.height
        self.total_points += num_points
        
        # Log point cloud info
        self.get_logger().info(f"Received point cloud #{self.message_count}:")
        self.get_logger().info(f"  Frame ID: {point_cloud.header.frame_id}")
        self.get_logger().info(f"  Width: {point_cloud.width}")
        self.get_logger().info(f"  Height: {point_cloud.height}")
        self.get_logger().info(f"  Points: {num_points}")
        self.get_logger().info(f"  Data size: {len(point_cloud.data)} bytes")
        self.get_logger().info(f"  Is dense: {point_cloud.is_dense}")
        
        # Process point cloud data
        self.process_point_cloud(point_cloud)
        
        # Log statistics every 10 messages
        if self.message_count % 10 == 0:
            elapsed_time = time.time() - self.start_time
            rate = self.message_count / elapsed_time
            avg_points = self.total_points / self.message_count
            
            self.get_logger().info(f"Received {self.message_count} messages at {rate:.2f} Hz")
            self.get_logger().info(f"Average points per message: {avg_points:.0f}")
            
            # Get subscriber statistics
            if hasattr(self.subscriber, 'get_statistics'):
                stats = self.subscriber.get_statistics()
                self.get_logger().info(f"Subscriber stats: {stats}")
        
    def process_point_cloud(self, point_cloud: PointCloud2):
        """Process received point cloud data"""
        # This is where you would implement your point cloud processing logic
        # For example: filtering, segmentation, feature extraction, etc.
        
        # Example: Calculate basic statistics
        num_points = point_cloud.width * point_cloud.height
        
        if num_points > 0:
            # Calculate point density
            point_density = num_points / (point_cloud.width * point_cloud.height)
            self.get_logger().debug(f"Point density: {point_density:.2f} points per pixel")
            
            # You can convert to numpy arrays for processing:
            # import sensor_msgs_py.point_cloud2 as pc2
            # points = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)
            # points_array = np.array(list(points))
            
            # Example processing: count points in different height ranges
            # This is a simplified example - in practice you'd use proper point cloud processing
            self.analyze_point_cloud_structure(point_cloud)
    
    def analyze_point_cloud_structure(self, point_cloud: PointCloud2):
        """Analyze the structure of the point cloud"""
        # This is a placeholder for more sophisticated analysis
        # In practice, you would use libraries like Open3D, PCL, or similar
        
        num_points = point_cloud.width * point_cloud.height
        
        if num_points > 0:
            # Log some basic analysis
            self.get_logger().debug(f"Analyzing {num_points} points...")
            
            # You could implement:
            # - Ground plane detection
            # - Object segmentation
            # - Feature extraction
            # - SLAM integration
            # - etc.
            
            self.get_logger().debug("Point cloud analysis completed")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LidarSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.get_logger().info("LiDAR Subscriber shutdown")
        rclpy.shutdown()


if __name__ == '__main__':
    main()

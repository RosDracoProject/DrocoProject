#!/usr/bin/env python3
# Copyright (c) 2024, Point Cloud Transport Python Package

"""
Example Point Cloud Subscriber using pure Python implementation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import time

from point_cloud_transport_py import PointCloudTransport


class PointCloudSubscriberExample(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber_example')
        
        # Create PointCloudTransport
        self.transport = PointCloudTransport(self)
        
        # Create subscriber with zlib compression
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.subscriber = self.transport.subscribe(
            "/point_cloud",
            self.point_cloud_callback,
            qos_profile=qos_profile,
            transport='zlib'
        )
        
        self.get_logger().info("Point Cloud Subscriber Example started")
        self.get_logger().info(f"Available transports: {self.transport.get_available_transports()}")
        
    def point_cloud_callback(self, point_cloud: PointCloud2):
        """Callback for received point cloud data"""
        self.get_logger().info(f"Received point cloud:")
        self.get_logger().info(f"  Frame ID: {point_cloud.header.frame_id}")
        self.get_logger().info(f"  Width: {point_cloud.width}")
        self.get_logger().info(f"  Height: {point_cloud.height}")
        self.get_logger().info(f"  Data size: {len(point_cloud.data)} bytes")
        self.get_logger().info(f"  Is dense: {point_cloud.is_dense}")
        
        # Process the point cloud data here
        self.process_point_cloud(point_cloud)
        
        # Log subscriber statistics
        stats = self.subscriber.get_statistics()
        self.get_logger().info(f"Subscriber stats: {stats}")
        
    def process_point_cloud(self, point_cloud: PointCloud2):
        """Process received point cloud data"""
        # This is where you would implement your point cloud processing logic
        # For example: filtering, segmentation, feature extraction, etc.
        
        # Example: Calculate basic statistics
        num_points = point_cloud.width * point_cloud.height
        self.get_logger().info(f"Processing {num_points} points")
        
        # You can convert to numpy arrays for processing:
        # import sensor_msgs_py.point_cloud2 as pc2
        # points = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)
        # points_array = np.array(list(points))
        
    def shutdown(self):
        """Shutdown the subscriber"""
        self.subscriber.shutdown()
        self.get_logger().info("Point Cloud Subscriber Example shutdown")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PointCloudSubscriberExample()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

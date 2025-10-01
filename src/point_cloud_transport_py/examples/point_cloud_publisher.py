#!/usr/bin/env python3
# Copyright (c) 2024, Point Cloud Transport Python Package

"""
Example Point Cloud Publisher using pure Python implementation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import time

from point_cloud_transport_py import PointCloudTransport


class PointCloudPublisherExample(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher_example')
        
        # Create PointCloudTransport
        self.transport = PointCloudTransport(self)
        
        # Set compression settings
        self.transport.set_compression_settings('zlib', level=6)
        self.transport.set_compression_settings('draco', compression_level=6)
        
        # Create publisher with zlib compression
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.publisher = self.transport.advertise(
            "/point_cloud", 
            queue_size=1, 
            qos_profile=qos_profile,
            transport='zlib'
        )
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0, self.publish_point_cloud)
        
        self.get_logger().info("Point Cloud Publisher Example started")
        self.get_logger().info(f"Available transports: {self.transport.get_available_transports()}")
        
    def publish_point_cloud(self):
        """Generate and publish a sample point cloud"""
        # Generate sample point cloud data
        points = self.generate_sample_point_cloud()
        
        # Create PointCloud2 message
        point_cloud = point_cloud2.create_cloud_xyz32(
            header=self.get_clock().now().to_msg(),
            points=points
        )
        
        # Publish via transport
        self.publisher.publish(point_cloud)
        
        # Log statistics
        stats = self.publisher.get_statistics()
        self.get_logger().info(f"Published point cloud with {len(points)} points")
        self.get_logger().info(f"Publisher stats: {stats}")
        
    def generate_sample_point_cloud(self):
        """Generate a sample point cloud"""
        # Create a simple point cloud (cube)
        points = []
        size = 1.0
        resolution = 0.1
        
        for x in np.arange(-size/2, size/2, resolution):
            for y in np.arange(-size/2, size/2, resolution):
                for z in np.arange(-size/2, size/2, resolution):
                    points.append([x, y, z])
                    
        return points


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PointCloudPublisherExample()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.publisher.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

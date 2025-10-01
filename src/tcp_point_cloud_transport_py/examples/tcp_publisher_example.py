#!/usr/bin/env python3
# Copyright (c) 2024, TCP Point Cloud Transport Python Package

"""
Example TCP Point Cloud Publisher

This example demonstrates how to publish point cloud data via TCP with Draco compression.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import time

from tcp_point_cloud_transport_py import TcpPointCloudTransport


class TcpPublisherExample(Node):
    def __init__(self):
        super().__init__('tcp_publisher_example')
        
        # Create TCP transport
        self.tcp_transport = TcpPointCloudTransport(
            self, 
            server_address="0.0.0.0", 
            server_port=8080
        )
        
        # Set compression settings
        self.tcp_transport.set_compression_settings(use_draco=True, compression_level=6)
        
        # Create publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.tcp_transport.create_publisher("/point_cloud", qos_profile)
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0, self.publish_point_cloud)
        
        self.get_logger().info("TCP Publisher Example started")
        self.get_logger().info(f"Server running on port {self.tcp_transport.server_port}")
        self.get_logger().info(f"Connected clients: {self.tcp_transport.get_connected_clients_count()}")
        
    def publish_point_cloud(self):
        """Generate and publish a sample point cloud"""
        # Generate sample point cloud data
        points = self.generate_sample_point_cloud()
        
        # Create PointCloud2 message
        point_cloud = point_cloud2.create_cloud_xyz32(
            header=self.get_clock().now().to_msg(),
            points=points
        )
        
        # Publish via TCP
        self.tcp_transport.publish(point_cloud)
        
        # Log status
        client_count = self.tcp_transport.get_connected_clients_count()
        self.get_logger().info(f"Published point cloud with {len(points)} points to {client_count} clients")
        
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
        node = TcpPublisherExample()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.tcp_transport.stop_server()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

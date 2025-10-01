#!/usr/bin/env python3
# Copyright (c) 2024, Gazebo LiDAR Simulation

"""
LiDAR Point Cloud Publisher with TCP Transport integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from point_cloud_transport_py import PointCloudTransport
import time


class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        
        # Get parameters
        self.declare_parameter('use_tcp_transport', True)
        self.declare_parameter('tcp_server_port', 8080)
        
        self.use_tcp_transport = self.get_parameter('use_tcp_transport').value
        self.tcp_server_port = self.get_parameter('tcp_server_port').value
        
        # Create point cloud transport
        self.transport = PointCloudTransport(self)
        
        # Set compression settings
        self.transport.set_compression_settings('zlib', level=6)
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create publisher
        if self.use_tcp_transport:
            self.publisher = self.transport.advertise(
                "/lidar/points", 
                queue_size=1, 
                qos_profile=qos_profile,
                transport='zlib'
            )
            self.get_logger().info(f"TCP transport enabled on port {self.tcp_server_port}")
        else:
            # Standard ROS2 publisher
            self.publisher = self.create_publisher(
                PointCloud2, 
                "/lidar/points", 
                qos_profile
            )
            self.get_logger().info("Standard ROS2 publisher enabled")
        
        # Subscribe to Velodyne LiDAR topic
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/velodyne2/velodyne_points2',
            self.lidar_callback,
            qos_profile
        )
        
        # Statistics
        self.message_count = 0
        self.start_time = time.time()
        
        self.get_logger().info("LiDAR Publisher started")
        
    def lidar_callback(self, point_cloud: PointCloud2):
        """Callback for LiDAR point cloud data"""
        self.message_count += 1
        
        # Log point cloud info
        num_points = point_cloud.width * point_cloud.height
        self.get_logger().info(f"Received LiDAR data: {num_points} points, {len(point_cloud.data)} bytes")
        
        # Publish via transport
        if self.use_tcp_transport:
            self.publisher.publish(point_cloud)
        else:
            self.publisher.publish(point_cloud)
            
        # Log statistics every 10 messages
        if self.message_count % 10 == 0:
            elapsed_time = time.time() - self.start_time
            rate = self.message_count / elapsed_time
            self.get_logger().info(f"Published {self.message_count} messages at {rate:.2f} Hz")
            
            # Get transport statistics
            if hasattr(self.publisher, 'get_statistics'):
                stats = self.publisher.get_statistics()
                self.get_logger().info(f"Transport stats: {stats}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LidarPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.get_logger().info("LiDAR Publisher shutdown")
        rclpy.shutdown()


if __name__ == '__main__':
    main()

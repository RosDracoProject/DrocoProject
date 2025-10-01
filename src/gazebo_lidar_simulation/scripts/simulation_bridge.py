#!/usr/bin/env python3
# Copyright (c) 2024, Gazebo LiDAR Simulation

"""
Simulation Bridge Node for integrating mobile-3d-lidar-sim with point_cloud_transport
This node bridges data between the two simulation systems.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from point_cloud_transport_py import PointCloudTransport
import time
import numpy as np


class SimulationBridge(Node):
    def __init__(self):
        super().__init__('simulation_bridge')
        
        # Get parameters
        self.declare_parameter('enable_velodyne_to_transport', True)
        self.declare_parameter('enable_transport_to_velodyne', False)
        self.declare_parameter('use_tcp_transport', True)
        self.declare_parameter('tcp_server_port', 8080)
        self.declare_parameter('compression_level', 6)
        
        self.enable_velodyne_to_transport = self.get_parameter('enable_velodyne_to_transport').value
        self.enable_transport_to_velodyne = self.get_parameter('enable_transport_to_velodyne').value
        self.use_tcp_transport = self.get_parameter('use_tcp_transport').value
        self.tcp_server_port = self.get_parameter('tcp_server_port').value
        self.compression_level = self.get_parameter('compression_level').value
        
        # Create point cloud transport
        self.transport = PointCloudTransport(self)
        
        # Set compression settings
        self.transport.set_compression_settings('zlib', level=self.compression_level)
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Statistics
        self.velodyne_to_transport_count = 0
        self.transport_to_velodyne_count = 0
        self.start_time = time.time()
        
        # Setup publishers and subscribers based on configuration
        self.setup_velodyne_to_transport()
        self.setup_transport_to_velodyne()
        
        self.get_logger().info("Simulation Bridge started")
        self.get_logger().info(f"Velodyne to Transport: {self.enable_velodyne_to_transport}")
        self.get_logger().info(f"Transport to Velodyne: {self.enable_transport_to_velodyne}")
        self.get_logger().info(f"TCP Transport: {self.use_tcp_transport}")
        
    def setup_velodyne_to_transport(self):
        """Setup bridge from Velodyne LiDAR to point_cloud_transport"""
        if not self.enable_velodyne_to_transport:
            return
            
        # Create transport publisher
        if self.use_tcp_transport:
            self.velodyne_publisher = self.transport.advertise(
                "/lidar/points", 
                queue_size=1, 
                qos_profile=QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                ),
                transport='zlib'
            )
        else:
            # Standard ROS2 publisher
            self.velodyne_publisher = self.create_publisher(
                PointCloud2, 
                "/lidar/points", 
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                )
            )
        
        # Subscribe to Velodyne LiDAR topic from mobile-3d-lidar-sim
        self.velodyne_subscription = self.create_subscription(
            PointCloud2,
            '/velodyne2/velodyne_points2',
            self.velodyne_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
        )
        
        self.get_logger().info("Velodyne to Transport bridge configured")
        
    def setup_transport_to_velodyne(self):
        """Setup bridge from point_cloud_transport to Velodyne LiDAR"""
        if not self.enable_transport_to_velodyne:
            return
            
        # Create standard ROS2 publisher for Velodyne format
        self.velodyne_output_publisher = self.create_publisher(
            PointCloud2,
            '/velodyne2/velodyne_points2',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
        )
        
        # Subscribe to transport topic
        if self.use_tcp_transport:
            self.transport_subscription = self.transport.subscribe(
                "/lidar/points",
                self.transport_callback,
                qos_profile=QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                ),
                transport='zlib'
            )
        else:
            self.transport_subscription = self.create_subscription(
                PointCloud2,
                "/lidar/points",
                self.transport_callback,
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                )
            )
        
        self.get_logger().info("Transport to Velodyne bridge configured")
        
    def velodyne_callback(self, point_cloud: PointCloud2):
        """Callback for Velodyne LiDAR data - forward to transport"""
        self.velodyne_to_transport_count += 1
        
        # Log point cloud info
        num_points = point_cloud.width * point_cloud.height
        self.get_logger().info(f"Velodyne -> Transport: {num_points} points, {len(point_cloud.data)} bytes")
        
        # Forward to transport
        self.velodyne_publisher.publish(point_cloud)
        
        # Log statistics every 10 messages
        if self.velodyne_to_transport_count % 10 == 0:
            elapsed_time = time.time() - self.start_time
            rate = self.velodyne_to_transport_count / elapsed_time
            self.get_logger().info(f"Velodyne -> Transport: {self.velodyne_to_transport_count} messages at {rate:.2f} Hz")
            
    def transport_callback(self, point_cloud: PointCloud2):
        """Callback for transport data - forward to Velodyne format"""
        self.transport_to_velodyne_count += 1
        
        # Log point cloud info
        num_points = point_cloud.width * point_cloud.height
        self.get_logger().info(f"Transport -> Velodyne: {num_points} points, {len(point_cloud.data)} bytes")
        
        # Convert frame_id to match Velodyne format if needed
        if point_cloud.header.frame_id != 'top':
            point_cloud.header.frame_id = 'top'
        
        # Forward to Velodyne format
        self.velodyne_output_publisher.publish(point_cloud)
        
        # Log statistics every 10 messages
        if self.transport_to_velodyne_count % 10 == 0:
            elapsed_time = time.time() - self.start_time
            rate = self.transport_to_velodyne_count / elapsed_time
            self.get_logger().info(f"Transport -> Velodyne: {self.transport_to_velodyne_count} messages at {rate:.2f} Hz")
            
    def get_bridge_statistics(self):
        """Get bridge statistics"""
        elapsed_time = time.time() - self.start_time
        
        stats = {
            'velodyne_to_transport_count': self.velodyne_to_transport_count,
            'transport_to_velodyne_count': self.transport_to_velodyne_count,
            'velodyne_to_transport_rate': self.velodyne_to_transport_count / elapsed_time if elapsed_time > 0 else 0,
            'transport_to_velodyne_rate': self.transport_to_velodyne_count / elapsed_time if elapsed_time > 0 else 0,
            'elapsed_time': elapsed_time
        }
        
        return stats


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimulationBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            stats = node.get_bridge_statistics()
            node.get_logger().info("Simulation Bridge shutdown")
            node.get_logger().info(f"Final statistics: {stats}")
        rclpy.shutdown()


if __name__ == '__main__':
    main()

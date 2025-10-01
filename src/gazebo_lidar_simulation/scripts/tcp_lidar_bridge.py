#!/usr/bin/env python3
# Copyright (c) 2024, Gazebo LiDAR Simulation

"""
TCP LiDAR Bridge for point cloud transmission over TCP/IP
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import socket
import threading
import time
import json
import struct


class TcpLidarBridge(Node):
    def __init__(self):
        super().__init__('tcp_lidar_bridge')
        
        # Get parameters
        self.declare_parameter('tcp_server_port', 8080)
        self.declare_parameter('max_clients', 10)
        self.declare_parameter('compression_enabled', True)
        
        self.tcp_port = self.get_parameter('tcp_server_port').value
        self.max_clients = self.get_parameter('max_clients').value
        self.compression_enabled = self.get_parameter('compression_enabled').value
        
        # TCP server setup
        self.server_socket = None
        self.clients = []
        self.server_running = False
        
        # Point cloud data
        self.latest_point_cloud = None
        self.point_cloud_lock = threading.Lock()
        
        # Statistics
        self.message_count = 0
        self.bytes_sent = 0
        self.start_time = time.time()
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribe to point cloud data
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.point_cloud_callback,
            qos_profile
        )
        
        # Start TCP server
        self.start_tcp_server()
        
        self.get_logger().info(f"TCP LiDAR Bridge started on port {self.tcp_port}")
        
    def start_tcp_server(self):
        """Start TCP server for point cloud transmission"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.tcp_port))
            self.server_socket.listen(self.max_clients)
            
            self.server_running = True
            
            # Start server thread
            server_thread = threading.Thread(target=self.tcp_server_loop)
            server_thread.daemon = True
            server_thread.start()
            
            self.get_logger().info(f"TCP server listening on port {self.tcp_port}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start TCP server: {e}")
            
    def tcp_server_loop(self):
        """Main TCP server loop"""
        while self.server_running:
            try:
                client_socket, client_address = self.server_socket.accept()
                self.get_logger().info(f"New client connected: {client_address}")
                
                # Add client to list
                self.clients.append(client_socket)
                
                # Start client handler thread
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, client_address)
                )
                client_thread.daemon = True
                client_thread.start()
                
            except Exception as e:
                if self.server_running:
                    self.get_logger().error(f"TCP server error: {e}")
                    
    def handle_client(self, client_socket, client_address):
        """Handle individual client connection"""
        try:
            while self.server_running:
                # Send latest point cloud data
                with self.point_cloud_lock:
                    if self.latest_point_cloud is not None:
                        self.send_point_cloud_to_client(client_socket, self.latest_point_cloud)
                
                time.sleep(0.1)  # 10 Hz update rate
                
        except Exception as e:
            self.get_logger().warn(f"Client {client_address} disconnected: {e}")
        finally:
            client_socket.close()
            if client_socket in self.clients:
                self.clients.remove(client_socket)
                
    def point_cloud_callback(self, point_cloud: PointCloud2):
        """Callback for point cloud data"""
        with self.point_cloud_lock:
            self.latest_point_cloud = point_cloud
            
        self.message_count += 1
        
        # Log statistics
        if self.message_count % 10 == 0:
            elapsed_time = time.time() - self.start_time
            rate = self.message_count / elapsed_time
            self.get_logger().info(f"Processed {self.message_count} point clouds at {rate:.2f} Hz")
            self.get_logger().info(f"Active clients: {len(self.clients)}")
            
    def send_point_cloud_to_client(self, client_socket, point_cloud: PointCloud2):
        """Send point cloud data to a specific client"""
        try:
            # Create message header
            header = {
                'frame_id': point_cloud.header.frame_id,
                'timestamp': {
                    'sec': point_cloud.header.stamp.sec,
                    'nanosec': point_cloud.header.stamp.nanosec
                },
                'width': point_cloud.width,
                'height': point_cloud.height,
                'point_step': point_cloud.point_step,
                'row_step': point_cloud.row_step,
                'is_dense': point_cloud.is_dense,
                'compression_enabled': self.compression_enabled
            }
            
            # Serialize header
            header_json = json.dumps(header).encode('utf-8')
            header_size = len(header_json)
            
            # Send header size and header
            client_socket.send(struct.pack('I', header_size))
            client_socket.send(header_json)
            
            # Send point cloud data
            data_size = len(point_cloud.data)
            client_socket.send(struct.pack('I', data_size))
            client_socket.send(point_cloud.data)
            
            # Update statistics
            self.bytes_sent += header_size + data_size + 8  # +8 for size fields
            
        except Exception as e:
            self.get_logger().error(f"Failed to send point cloud to client: {e}")
            raise
            
    def get_statistics(self):
        """Get transmission statistics"""
        elapsed_time = time.time() - self.start_time
        return {
            'messages_sent': self.message_count,
            'bytes_sent': self.bytes_sent,
            'active_clients': len(self.clients),
            'transmission_rate': self.message_count / elapsed_time if elapsed_time > 0 else 0,
            'data_rate_mbps': (self.bytes_sent * 8) / (elapsed_time * 1e6) if elapsed_time > 0 else 0
        }
        
    def shutdown(self):
        """Shutdown TCP server"""
        self.server_running = False
        
        # Close all client connections
        for client in self.clients:
            try:
                client.close()
            except:
                pass
        self.clients.clear()
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
                
        self.get_logger().info("TCP LiDAR Bridge shutdown")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TcpLidarBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

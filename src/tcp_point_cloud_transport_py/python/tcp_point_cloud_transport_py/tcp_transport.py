# Copyright (c) 2024, TCP Point Cloud Transport Python Package

"""
Python interface for TCP Point Cloud Transport
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from point_cloud_transport import PointCloudTransport
import threading
import socket
import struct
import time
from typing import Optional, Callable, List
import numpy as np


class TcpPointCloudTransport:
    """
    Python wrapper for TCP Point Cloud Transport with Draco compression support
    """
    
    def __init__(self, node: Node, server_address: str = "0.0.0.0", server_port: int = 8080):
        """
        Initialize TCP Point Cloud Transport
        
        Args:
            node: ROS2 node
            server_address: Server IP address
            server_port: Server port number
        """
        self.node = node
        self.server_address = server_address
        self.server_port = server_port
        self.pct = PointCloudTransport(node)
        
        # TCP server components
        self.server_socket = None
        self.server_running = False
        self.server_thread = None
        self.connected_clients = []
        self.clients_lock = threading.Lock()
        
        # Publishers and subscribers
        self.publisher = None
        self.subscriber = None
        self.user_callback = None
        
        # Compression settings
        self.use_draco = True
        self.compression_level = 6
        
    def start_server(self):
        """Start TCP server"""
        if self.server_running:
            return
            
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.server_address, self.server_port))
            self.server_socket.listen(5)
            
            self.server_running = True
            self.server_thread = threading.Thread(target=self._server_loop)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            self.node.get_logger().info(f"TCP server started on {self.server_address}:{self.server_port}")
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to start TCP server: {e}")
            
    def stop_server(self):
        """Stop TCP server"""
        self.server_running = False
        
        if self.server_socket:
            self.server_socket.close()
            self.server_socket = None
            
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join()
            
        with self.clients_lock:
            for client_socket in self.connected_clients:
                try:
                    client_socket.close()
                except:
                    pass
            self.connected_clients.clear()
            
        self.node.get_logger().info("TCP server stopped")
        
    def _server_loop(self):
        """Main server loop"""
        while self.server_running:
            try:
                client_socket, client_address = self.server_socket.accept()
                self.node.get_logger().info(f"New client connected from {client_address}")
                
                with self.clients_lock:
                    self.connected_clients.append(client_socket)
                    
                # Handle client in separate thread
                client_thread = threading.Thread(
                    target=self._handle_client, 
                    args=(client_socket, client_address)
                )
                client_thread.daemon = True
                client_thread.start()
                
            except Exception as e:
                if self.server_running:
                    self.node.get_logger().warn(f"Error accepting client: {e}")
                    
    def _handle_client(self, client_socket: socket.socket, client_address):
        """Handle individual client connection"""
        try:
            while self.server_running:
                # Keep connection alive and handle any client messages
                data = client_socket.recv(1024)
                if not data:
                    break
                # Handle client messages if needed
                
        except Exception as e:
            self.node.get_logger().warn(f"Error handling client {client_address}: {e}")
        finally:
            client_socket.close()
            with self.clients_lock:
                if client_socket in self.connected_clients:
                    self.connected_clients.remove(client_socket)
                    
    def create_publisher(self, topic: str, qos_profile: Optional[QoSProfile] = None):
        """
        Create TCP publisher for point cloud data
        
        Args:
            topic: Topic name
            qos_profile: QoS profile for the publisher
        """
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            
        self.publisher = self.pct.advertise(topic, 1, qos_profile)
        self.start_server()
        
    def create_subscriber(self, topic: str, callback: Callable, qos_profile: Optional[QoSProfile] = None):
        """
        Create TCP subscriber for point cloud data
        
        Args:
            topic: Topic name
            callback: Callback function for received messages
            qos_profile: QoS profile for the subscriber
        """
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            
        self.user_callback = callback
        self.subscriber = self.pct.subscribe(topic, callback, qos_profile)
        
    def publish(self, point_cloud: PointCloud2):
        """
        Publish point cloud data via TCP
        
        Args:
            point_cloud: PointCloud2 message to publish
        """
        if self.publisher is None:
            self.node.get_logger().error("Publisher not created. Call create_publisher first.")
            return
            
        # Compress and send via TCP
        compressed_data = self._compress_point_cloud(point_cloud)
        self._broadcast_to_clients(compressed_data)
        
        # Also publish via ROS2
        self.publisher.publish(point_cloud)
        
    def _compress_point_cloud(self, point_cloud: PointCloud2) -> bytes:
        """
        Compress point cloud data using Draco
        
        Args:
            point_cloud: PointCloud2 message
            
        Returns:
            Compressed data as bytes
        """
        if self.use_draco:
            # Use Draco compression (placeholder implementation)
            return self._draco_compress(point_cloud)
        else:
            # Use raw data
            return bytes(point_cloud.data)
            
    def _draco_compress(self, point_cloud: PointCloud2) -> bytes:
        """
        Draco compression implementation (placeholder)
        
        Args:
            point_cloud: PointCloud2 message
            
        Returns:
            Compressed data as bytes
        """
        # This is a placeholder implementation
        # In a real implementation, you would use the Draco library
        self.node.get_logger().warn("Draco compression not implemented - using raw data")
        return bytes(point_cloud.data)
        
    def _broadcast_to_clients(self, data: bytes):
        """
        Broadcast data to all connected clients
        
        Args:
            data: Data to broadcast
        """
        with self.clients_lock:
            clients_to_remove = []
            for client_socket in self.connected_clients:
                try:
                    # Send data size first
                    data_size = len(data)
                    client_socket.send(struct.pack('I', data_size))
                    # Send data
                    client_socket.send(data)
                except Exception as e:
                    self.node.get_logger().warn(f"Error sending to client: {e}")
                    clients_to_remove.append(client_socket)
                    
            # Remove failed clients
            for client_socket in clients_to_remove:
                self.connected_clients.remove(client_socket)
                try:
                    client_socket.close()
                except:
                    pass
                    
    def set_compression_settings(self, use_draco: bool = True, compression_level: int = 6):
        """
        Set compression settings
        
        Args:
            use_draco: Whether to use Draco compression
            compression_level: Compression level (0-10)
        """
        self.use_draco = use_draco
        self.compression_level = max(0, min(10, compression_level))
        
    def get_connected_clients_count(self) -> int:
        """
        Get number of connected clients
        
        Returns:
            Number of connected clients
        """
        with self.clients_lock:
            return len(self.connected_clients)
            
    def is_server_running(self) -> bool:
        """
        Check if TCP server is running
        
        Returns:
            True if server is running
        """
        return self.server_running


class TcpPointCloudClient:
    """
    TCP client for receiving point cloud data
    """
    
    def __init__(self, node: Node, server_address: str, server_port: int = 8080):
        """
        Initialize TCP client
        
        Args:
            node: ROS2 node
            server_address: Server IP address
            server_port: Server port number
        """
        self.node = node
        self.server_address = server_address
        self.server_port = server_port
        self.client_socket = None
        self.client_running = False
        self.client_thread = None
        self.callback = None
        
    def connect(self, callback: Callable):
        """
        Connect to TCP server
        
        Args:
            callback: Callback function for received point clouds
        """
        self.callback = callback
        
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_address, self.server_port))
            
            self.client_running = True
            self.client_thread = threading.Thread(target=self._client_loop)
            self.client_thread.daemon = True
            self.client_thread.start()
            
            self.node.get_logger().info(f"Connected to TCP server at {self.server_address}:{self.server_port}")
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to connect to TCP server: {e}")
            
    def disconnect(self):
        """Disconnect from TCP server"""
        self.client_running = False
        
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
            
        if self.client_thread and self.client_thread.is_alive():
            self.client_thread.join()
            
        self.node.get_logger().info("Disconnected from TCP server")
        
    def _client_loop(self):
        """Main client loop"""
        while self.client_running:
            try:
                # Receive data size
                data_size_bytes = self.client_socket.recv(4)
                if not data_size_bytes:
                    break
                    
                data_size = struct.unpack('I', data_size_bytes)[0]
                
                # Receive compressed data
                compressed_data = b''
                while len(compressed_data) < data_size:
                    chunk = self.client_socket.recv(min(data_size - len(compressed_data), 4096))
                    if not chunk:
                        break
                    compressed_data += chunk
                    
                if len(compressed_data) == data_size and self.callback:
                    # Decompress and call callback
                    point_cloud = self._decompress_point_cloud(compressed_data)
                    self.callback(point_cloud)
                    
            except Exception as e:
                if self.client_running:
                    self.node.get_logger().warn(f"Error receiving data: {e}")
                    
    def _decompress_point_cloud(self, compressed_data: bytes) -> PointCloud2:
        """
        Decompress point cloud data
        
        Args:
            compressed_data: Compressed data
            
        Returns:
            Decompressed PointCloud2 message
        """
        # This is a placeholder implementation
        # In a real implementation, you would use the Draco library
        point_cloud = PointCloud2()
        point_cloud.data = list(compressed_data)
        return point_cloud

# Copyright (c) 2024, Point Cloud Transport Python Package

"""
Pure Python implementation of PointCloudTransport
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from typing import Optional, Callable, List
import threading
import time

from .publisher import Publisher
from .subscriber import Subscriber
from .codec import PointCloudCodec


class PointCloudTransport:
    """
    Pure Python implementation of PointCloudTransport
    
    This class provides a Python interface for point cloud transport with
    compression support, similar to the C++ point_cloud_transport but
    implemented entirely in Python.
    """
    
    def __init__(self, node: Node):
        """
        Initialize PointCloudTransport
        
        Args:
            node: ROS2 node instance
        """
        self.node = node
        self.logger = node.get_logger()
        self.codec = PointCloudCodec()
        
        # Available transports
        self.available_transports = [
            'raw',      # No compression
            'zlib',     # Zlib compression
            'draco',    # Draco compression (if available)
            'custom'    # Custom compression
        ]
        
        self.logger.info("PointCloudTransport initialized with Python implementation")
        
    def advertise(self, topic: str, queue_size: int = 1, 
                  qos_profile: Optional[QoSProfile] = None,
                  transport: str = 'raw') -> Publisher:
        """
        Advertise a point cloud topic
        
        Args:
            topic: Topic name
            queue_size: Queue size for messages
            qos_profile: QoS profile
            transport: Transport type ('raw', 'zlib', 'draco', 'custom')
            
        Returns:
            Publisher instance
        """
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=queue_size
            )
            
        if transport not in self.available_transports:
            self.logger.warn(f"Unknown transport '{transport}', using 'raw'")
            transport = 'raw'
            
        publisher = Publisher(
            self.node, 
            topic, 
            qos_profile, 
            transport,
            self.codec
        )
        
        self.logger.info(f"Advertised topic '{topic}' with transport '{transport}'")
        return publisher
        
    def subscribe(self, topic: str, callback: Callable, 
                  qos_profile: Optional[QoSProfile] = None,
                  transport: str = 'raw') -> Subscriber:
        """
        Subscribe to a point cloud topic
        
        Args:
            topic: Topic name
            callback: Callback function
            qos_profile: QoS profile
            transport: Transport type
            
        Returns:
            Subscriber instance
        """
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            
        if transport not in self.available_transports:
            self.logger.warn(f"Unknown transport '{transport}', using 'raw'")
            transport = 'raw'
            
        subscriber = Subscriber(
            self.node,
            topic,
            callback,
            qos_profile,
            transport,
            self.codec
        )
        
        self.logger.info(f"Subscribed to topic '{topic}' with transport '{transport}'")
        return subscriber
        
    def get_available_transports(self) -> List[str]:
        """
        Get list of available transport types
        
        Returns:
            List of available transport names
        """
        return self.available_transports.copy()
        
    def set_compression_settings(self, transport: str, **kwargs):
        """
        Set compression settings for a specific transport
        
        Args:
            transport: Transport type
            **kwargs: Compression-specific parameters
        """
        self.codec.set_compression_settings(transport, **kwargs)
        
    def get_transport_info(self, transport: str) -> dict:
        """
        Get information about a transport
        
        Args:
            transport: Transport type
            
        Returns:
            Dictionary with transport information
        """
        return self.codec.get_transport_info(transport)

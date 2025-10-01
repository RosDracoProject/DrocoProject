# Copyright (c) 2024, Point Cloud Transport Python Package

"""
Pure Python implementation of PointCloud Subscriber
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from typing import Callable, Optional
import time

from .codec import PointCloudCodec


class Subscriber:
    """
    Pure Python implementation of PointCloud Subscriber
    
    This class provides a Python interface for subscribing to point cloud data
    with various decompression options.
    """
    
    def __init__(self, node: Node, topic: str, callback: Callable, 
                 qos_profile: QoSProfile, transport: str, codec: PointCloudCodec):
        """
        Initialize Subscriber
        
        Args:
            node: ROS2 node
            topic: Topic name
            callback: Callback function
            qos_profile: QoS profile
            transport: Transport type
            codec: Codec instance
        """
        self.node = node
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile
        self.transport = transport
        self.codec = codec
        self.logger = node.get_logger()
        
        # Statistics
        self.message_count = 0
        self.total_decompression_time = 0.0
        
        # Create ROS2 subscription
        self.ros_subscription = node.create_subscription(
            PointCloud2,
            topic,
            self._message_callback,
            qos_profile
        )
        
        self.logger.info(f"Subscriber created for topic '{topic}' with transport '{transport}'")
        
    def _message_callback(self, point_cloud: PointCloud2):
        """
        Internal callback for ROS2 messages
        
        Args:
            point_cloud: Received PointCloud2 message
        """
        start_time = time.time()
        
        try:
            # Apply decompression if needed
            if self.transport != 'raw':
                decompressed_cloud = self.codec.decompress(point_cloud, self.transport)
                if decompressed_cloud is not None:
                    point_cloud = decompressed_cloud
                    
            # Call user callback
            self.callback(point_cloud)
            
            # Update statistics
            self.message_count += 1
            decompression_time = (time.time() - start_time) * 1000  # ms
            self.total_decompression_time += decompression_time
            
            if self.message_count % 100 == 0:  # Log every 100 messages
                avg_time = self.total_decompression_time / self.message_count
                self.logger.debug(f"Received {self.message_count} messages, avg decompression time: {avg_time:.2f}ms")
                
        except Exception as e:
            self.logger.error(f"Failed to process point cloud message: {e}")
            
    def get_topic(self) -> str:
        """
        Get the topic name
        
        Returns:
            Topic name
        """
        return self.topic
        
    def get_num_publishers(self) -> int:
        """
        Get number of publishers
        
        Returns:
            Number of publishers
        """
        return self.ros_subscription.get_publisher_count()
        
    def shutdown(self):
        """
        Shutdown the subscriber
        """
        self.ros_subscription.destroy()
        self.logger.info(f"Subscriber for topic '{self.topic}' shutdown")
        
    def get_statistics(self) -> dict:
        """
        Get subscriber statistics
        
        Returns:
            Dictionary with statistics
        """
        avg_decompression_time = 0.0
        if self.message_count > 0:
            avg_decompression_time = self.total_decompression_time / self.message_count
            
        return {
            'topic': self.topic,
            'transport': self.transport,
            'message_count': self.message_count,
            'publisher_count': self.get_num_publishers(),
            'avg_decompression_time_ms': avg_decompression_time,
            'total_decompression_time_ms': self.total_decompression_time
        }

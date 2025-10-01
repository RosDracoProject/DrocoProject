# Copyright (c) 2024, Point Cloud Transport Python Package

"""
Pure Python implementation of PointCloud Publisher
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from typing import Optional
import time

from .codec import PointCloudCodec


class Publisher:
    """
    Pure Python implementation of PointCloud Publisher
    
    This class provides a Python interface for publishing point cloud data
    with various compression options.
    """
    
    def __init__(self, node: Node, topic: str, qos_profile: QoSProfile, 
                 transport: str, codec: PointCloudCodec):
        """
        Initialize Publisher
        
        Args:
            node: ROS2 node
            topic: Topic name
            qos_profile: QoS profile
            transport: Transport type
            codec: Codec instance
        """
        self.node = node
        self.topic = topic
        self.qos_profile = qos_profile
        self.transport = transport
        self.codec = codec
        self.logger = node.get_logger()
        
        # Create ROS2 publisher
        self.ros_publisher = node.create_publisher(
            PointCloud2, 
            topic, 
            qos_profile
        )
        
        # Statistics
        self.publish_count = 0
        self.total_compression_time = 0.0
        
        self.logger.info(f"Publisher created for topic '{topic}' with transport '{transport}'")
        
    def publish(self, point_cloud: PointCloud2):
        """
        Publish a point cloud message
        
        Args:
            point_cloud: PointCloud2 message to publish
        """
        start_time = time.time()
        
        try:
            # Apply compression if needed
            if self.transport != 'raw':
                compressed_cloud = self.codec.compress(point_cloud, self.transport)
                if compressed_cloud is not None:
                    point_cloud = compressed_cloud
                    
            # Publish via ROS2
            self.ros_publisher.publish(point_cloud)
            
            # Update statistics
            self.publish_count += 1
            compression_time = (time.time() - start_time) * 1000  # ms
            self.total_compression_time += compression_time
            
            if self.publish_count % 100 == 0:  # Log every 100 messages
                avg_time = self.total_compression_time / self.publish_count
                self.logger.debug(f"Published {self.publish_count} messages, avg compression time: {avg_time:.2f}ms")
                
        except Exception as e:
            self.logger.error(f"Failed to publish point cloud: {e}")
            
    def get_topic(self) -> str:
        """
        Get the topic name
        
        Returns:
            Topic name
        """
        return self.topic
        
    def get_num_subscribers(self) -> int:
        """
        Get number of subscribers
        
        Returns:
            Number of subscribers
        """
        return self.ros_publisher.get_subscription_count()
        
    def shutdown(self):
        """
        Shutdown the publisher
        """
        self.ros_publisher.destroy()
        self.logger.info(f"Publisher for topic '{self.topic}' shutdown")
        
    def get_statistics(self) -> dict:
        """
        Get publisher statistics
        
        Returns:
            Dictionary with statistics
        """
        avg_compression_time = 0.0
        if self.publish_count > 0:
            avg_compression_time = self.total_compression_time / self.publish_count
            
        return {
            'topic': self.topic,
            'transport': self.transport,
            'publish_count': self.publish_count,
            'subscriber_count': self.get_num_subscribers(),
            'avg_compression_time_ms': avg_compression_time,
            'total_compression_time_ms': self.total_compression_time
        }

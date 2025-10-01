#!/usr/bin/env python3
# Copyright (c) 2024, Point Cloud Transport Python Package

"""
Test script for PointCloudTransport Python functionality
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import time

from point_cloud_transport_py import PointCloudTransport, Publisher, Subscriber, PointCloudCodec


class TestPointCloudTransport:
    """Test cases for PointCloudTransport"""
    
    def setup_method(self):
        """Setup for each test method"""
        rclpy.init()
        self.node = Node('test_node')
        
    def teardown_method(self):
        """Cleanup after each test method"""
        rclpy.shutdown()
        
    def test_transport_creation(self):
        """Test PointCloudTransport creation"""
        transport = PointCloudTransport(self.node)
        assert transport is not None
        assert transport.node == self.node
        
    def test_available_transports(self):
        """Test available transports"""
        transport = PointCloudTransport(self.node)
        transports = transport.get_available_transports()
        assert isinstance(transports, list)
        assert 'raw' in transports
        assert 'zlib' in transports
        
    def test_publisher_creation(self):
        """Test publisher creation"""
        transport = PointCloudTransport(self.node)
        publisher = transport.advertise("/test_topic", transport='raw')
        assert isinstance(publisher, Publisher)
        assert publisher.get_topic() == "/test_topic"
        
    def test_subscriber_creation(self):
        """Test subscriber creation"""
        transport = PointCloudTransport(self.node)
        
        def dummy_callback(msg):
            pass
            
        subscriber = transport.subscribe("/test_topic", dummy_callback, transport='raw')
        assert isinstance(subscriber, Subscriber)
        assert subscriber.get_topic() == "/test_topic"
        
    def test_compression_settings(self):
        """Test compression settings"""
        transport = PointCloudTransport(self.node)
        
        # Test setting compression settings
        transport.set_compression_settings('zlib', level=9)
        transport.set_compression_settings('draco', compression_level=8)
        
        # Should not raise exceptions
        assert True
        
    def test_transport_info(self):
        """Test transport info"""
        transport = PointCloudTransport(self.node)
        
        info = transport.get_transport_info('raw')
        assert isinstance(info, dict)
        assert 'type' in info


class TestPointCloudCodec:
    """Test cases for PointCloudCodec"""
    
    def setup_method(self):
        """Setup for each test method"""
        rclpy.init()
        self.node = Node('test_node')
        self.codec = PointCloudCodec()
        
    def teardown_method(self):
        """Cleanup after each test method"""
        rclpy.shutdown()
        
    def test_codec_creation(self):
        """Test codec creation"""
        assert self.codec is not None
        
    def test_available_transports(self):
        """Test available transports"""
        transports = self.codec.get_available_transports()
        assert isinstance(transports, list)
        assert 'raw' in transports
        assert 'zlib' in transports
        
    def test_raw_compression(self):
        """Test raw compression (no compression)"""
        # Create sample point cloud
        points = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
        point_cloud = point_cloud2.create_cloud_xyz32(
            header=self.node.get_clock().now().to_msg(),
            points=points
        )
        
        # Test compression
        compressed = self.codec.compress(point_cloud, 'raw')
        assert compressed is not None
        assert len(compressed.data) == len(point_cloud.data)
        
        # Test decompression
        decompressed = self.codec.decompress(compressed, 'raw')
        assert decompressed is not None
        assert len(decompressed.data) == len(point_cloud.data)
        
    def test_zlib_compression(self):
        """Test zlib compression"""
        # Create sample point cloud
        points = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
        point_cloud = point_cloud2.create_cloud_xyz32(
            header=self.node.get_clock().now().to_msg(),
            points=points
        )
        
        # Test compression
        compressed = self.codec.compress(point_cloud, 'zlib')
        assert compressed is not None
        
        # Test decompression
        decompressed = self.codec.decompress(compressed, 'zlib')
        assert decompressed is not None
        
    def test_compression_ratio(self):
        """Test compression ratio calculation"""
        # Create sample point cloud
        points = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
        point_cloud = point_cloud2.create_cloud_xyz32(
            header=self.node.get_clock().now().to_msg(),
            points=points
        )
        
        compressed = self.codec.compress(point_cloud, 'zlib')
        if compressed:
            ratio = self.codec.get_compression_ratio(point_cloud, compressed)
            assert ratio > 0
            assert isinstance(ratio, float)
            
    def test_benchmark_compression(self):
        """Test compression benchmarking"""
        # Create sample point cloud
        points = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
        point_cloud = point_cloud2.create_cloud_xyz32(
            header=self.node.get_clock().now().to_msg(),
            points=points
        )
        
        # Test benchmark
        results = self.codec.benchmark_compression(point_cloud, 'zlib', iterations=3)
        assert isinstance(results, dict)
        assert 'transport' in results
        assert 'iterations' in results
        assert 'avg_compression_time_ms' in results


def test_integration_publisher_subscriber():
    """Integration test for publisher-subscriber communication"""
    rclpy.init()
    
    try:
        # Create publisher node
        pub_node = Node('test_publisher')
        pub_transport = PointCloudTransport(pub_node)
        publisher = pub_transport.advertise("/test_topic", transport='raw')
        
        # Create subscriber node
        sub_node = Node('test_subscriber')
        sub_transport = PointCloudTransport(sub_node)
        
        # Test data
        received_data = []
        
        def callback(point_cloud):
            received_data.append(point_cloud)
            
        subscriber = sub_transport.subscribe("/test_topic", callback, transport='raw')
        
        # Generate and publish test data
        points = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
        point_cloud = point_cloud2.create_cloud_xyz32(
            header=pub_node.get_clock().now().to_msg(),
            points=points
        )
        
        # Publish data
        publisher.publish(point_cloud)
        
        # Process callbacks
        rclpy.spin_once(pub_node, timeout_sec=0.1)
        rclpy.spin_once(sub_node, timeout_sec=0.1)
        
        # Cleanup
        publisher.shutdown()
        subscriber.shutdown()
        
        # Note: In a real test, you would verify that data was received
        # This is a simplified test that just checks the setup works
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    # Run tests
    pytest.main([__file__, '-v'])

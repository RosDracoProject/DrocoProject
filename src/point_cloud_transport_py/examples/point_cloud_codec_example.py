#!/usr/bin/env python3
# Copyright (c) 2024, Point Cloud Transport Python Package

"""
Example Point Cloud Codec usage with compression benchmarking
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import time

from point_cloud_transport_py import PointCloudCodec


class PointCloudCodecExample(Node):
    def __init__(self):
        super().__init__('point_cloud_codec_example')
        
        # Create codec
        self.codec = PointCloudCodec()
        
        # Generate sample point cloud
        self.sample_cloud = self.generate_sample_point_cloud()
        
        self.get_logger().info("Point Cloud Codec Example started")
        self.get_logger().info(f"Available transports: {self.codec.get_available_transports()}")
        
        # Run compression tests
        self.run_compression_tests()
        
    def generate_sample_point_cloud(self):
        """Generate a sample point cloud"""
        # Create a point cloud with many points
        points = []
        size = 2.0
        resolution = 0.05  # Higher resolution for more points
        
        for x in np.arange(-size/2, size/2, resolution):
            for y in np.arange(-size/2, size/2, resolution):
                for z in np.arange(-size/2, size/2, resolution):
                    points.append([x, y, z])
                    
        # Create PointCloud2 message
        point_cloud = point_cloud2.create_cloud_xyz32(
            header=self.get_clock().now().to_msg(),
            points=points
        )
        
        self.get_logger().info(f"Generated point cloud with {len(points)} points")
        self.get_logger().info(f"Original data size: {len(point_cloud.data)} bytes")
        
        return point_cloud
        
    def run_compression_tests(self):
        """Run compression tests for all available transports"""
        transports = self.codec.get_available_transports()
        
        for transport in transports:
            self.get_logger().info(f"\n--- Testing {transport} compression ---")
            
            # Test compression
            start_time = time.time()
            compressed_cloud = self.codec.compress(self.sample_cloud, transport)
            compression_time = (time.time() - start_time) * 1000  # ms
            
            if compressed_cloud:
                # Calculate compression ratio
                original_size = len(self.sample_cloud.data)
                compressed_size = len(compressed_cloud.data)
                compression_ratio = original_size / compressed_size if compressed_size > 0 else 1.0
                
                self.get_logger().info(f"Compression time: {compression_time:.2f}ms")
                self.get_logger().info(f"Original size: {original_size:,} bytes")
                self.get_logger().info(f"Compressed size: {compressed_size:,} bytes")
                self.get_logger().info(f"Compression ratio: {compression_ratio:.2f}:1")
                
                # Test decompression
                start_time = time.time()
                decompressed_cloud = self.codec.decompress(compressed_cloud, transport)
                decompression_time = (time.time() - start_time) * 1000  # ms
                
                if decompressed_cloud:
                    self.get_logger().info(f"Decompression time: {decompression_time:.2f}ms")
                    self.get_logger().info(f"Decompressed size: {len(decompressed_cloud.data):,} bytes")
                else:
                    self.get_logger().error("Decompression failed")
            else:
                self.get_logger().error("Compression failed")
                
            # Get transport info
            info = self.codec.get_transport_info(transport)
            self.get_logger().info(f"Transport info: {info}")
            
    def run_benchmark(self, transport: str, iterations: int = 10):
        """Run benchmark for specific transport"""
        self.get_logger().info(f"\n--- Benchmarking {transport} ---")
        
        benchmark_results = self.codec.benchmark_compression(
            self.sample_cloud, 
            transport, 
            iterations
        )
        
        self.get_logger().info(f"Benchmark results: {benchmark_results}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PointCloudCodecExample()
        
        # Run additional benchmarks
        node.run_benchmark('zlib', 5)
        node.run_benchmark('raw', 5)
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

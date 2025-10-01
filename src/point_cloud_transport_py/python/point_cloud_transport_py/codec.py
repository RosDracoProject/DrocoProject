# Copyright (c) 2024, Point Cloud Transport Python Package

"""
Pure Python implementation of PointCloud Codec
"""

from sensor_msgs.msg import PointCloud2
from typing import Optional, Dict, Any
import time

from .compression import CompressionManager


class PointCloudCodec:
    """
    Pure Python implementation of PointCloud Codec
    
    This class provides encoding and decoding functionality for point cloud data
    with various compression algorithms.
    """
    
    def __init__(self):
        """Initialize PointCloudCodec"""
        self.compression_manager = CompressionManager()
        self.available_transports = self.compression_manager.get_available_transports()
        
    def compress(self, point_cloud: PointCloud2, transport: str) -> Optional[PointCloud2]:
        """
        Compress a point cloud using the specified transport
        
        Args:
            point_cloud: PointCloud2 message to compress
            transport: Transport type ('raw', 'zlib', 'draco', 'custom')
            
        Returns:
            Compressed PointCloud2 message or None if compression fails
        """
        if transport == 'raw':
            return point_cloud
            
        try:
            return self.compression_manager.compress(point_cloud, transport)
        except Exception as e:
            print(f"Compression failed for transport '{transport}': {e}")
            return None
            
    def decompress(self, point_cloud: PointCloud2, transport: str) -> Optional[PointCloud2]:
        """
        Decompress a point cloud using the specified transport
        
        Args:
            point_cloud: PointCloud2 message to decompress
            transport: Transport type
            
        Returns:
            Decompressed PointCloud2 message or None if decompression fails
        """
        if transport == 'raw':
            return point_cloud
            
        try:
            return self.compression_manager.decompress(point_cloud, transport)
        except Exception as e:
            print(f"Decompression failed for transport '{transport}': {e}")
            return None
            
    def encode(self, point_cloud: PointCloud2, transport: str) -> Optional[bytes]:
        """
        Encode a point cloud to bytes using the specified transport
        
        Args:
            point_cloud: PointCloud2 message to encode
            transport: Transport type
            
        Returns:
            Encoded bytes or None if encoding fails
        """
        try:
            return self.compression_manager.encode(point_cloud, transport)
        except Exception as e:
            print(f"Encoding failed for transport '{transport}': {e}")
            return None
            
    def decode(self, data: bytes, transport: str) -> Optional[PointCloud2]:
        """
        Decode bytes to a point cloud using the specified transport
        
        Args:
            data: Encoded bytes
            transport: Transport type
            
        Returns:
            Decoded PointCloud2 message or None if decoding fails
        """
        try:
            return self.compression_manager.decode(data, transport)
        except Exception as e:
            print(f"Decoding failed for transport '{transport}': {e}")
            return None
            
    def get_available_transports(self) -> list:
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
        self.compression_manager.set_compression_settings(transport, **kwargs)
        
    def get_transport_info(self, transport: str) -> Dict[str, Any]:
        """
        Get information about a transport
        
        Args:
            transport: Transport type
            
        Returns:
            Dictionary with transport information
        """
        return self.compression_manager.get_transport_info(transport)
        
    def get_compression_ratio(self, original: PointCloud2, compressed: PointCloud2) -> float:
        """
        Calculate compression ratio
        
        Args:
            original: Original point cloud
            compressed: Compressed point cloud
            
        Returns:
            Compression ratio (original_size / compressed_size)
        """
        if len(compressed.data) == 0:
            return 1.0
        return len(original.data) / len(compressed.data)
        
    def benchmark_compression(self, point_cloud: PointCloud2, transport: str, iterations: int = 10) -> Dict[str, float]:
        """
        Benchmark compression performance
        
        Args:
            point_cloud: PointCloud2 message to test
            transport: Transport type
            iterations: Number of iterations for benchmarking
            
        Returns:
            Dictionary with benchmark results
        """
        if transport not in self.available_transports:
            return {'error': f'Transport {transport} not available'}
            
        # Compression benchmark
        compression_times = []
        compression_ratios = []
        
        for _ in range(iterations):
            start_time = time.time()
            compressed = self.compress(point_cloud, transport)
            compression_time = (time.time() - start_time) * 1000  # ms
            compression_times.append(compression_time)
            
            if compressed:
                ratio = self.get_compression_ratio(point_cloud, compressed)
                compression_ratios.append(ratio)
                
        # Decompression benchmark
        compressed = self.compress(point_cloud, transport)
        if not compressed:
            return {'error': 'Compression failed'}
            
        decompression_times = []
        for _ in range(iterations):
            start_time = time.time()
            decompressed = self.decompress(compressed, transport)
            decompression_time = (time.time() - start_time) * 1000  # ms
            decompression_times.append(decompression_time)
            
        return {
            'transport': transport,
            'iterations': iterations,
            'avg_compression_time_ms': sum(compression_times) / len(compression_times),
            'avg_decompression_time_ms': sum(decompression_times) / len(decompression_times),
            'avg_compression_ratio': sum(compression_ratios) / len(compression_ratios) if compression_ratios else 1.0,
            'min_compression_time_ms': min(compression_times),
            'max_compression_time_ms': max(compression_times)
        }

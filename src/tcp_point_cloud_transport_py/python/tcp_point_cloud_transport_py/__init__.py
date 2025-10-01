# Copyright (c) 2024, TCP Point Cloud Transport Python Package

"""
TCP Point Cloud Transport Python Package

This package provides Python interfaces for TCP/IP transport of point cloud data with Draco compression support.
It includes both pure Python implementations and C++ bindings for efficient 3D LiDAR data transmission.
"""

from .tcp_transport import TcpPointCloudTransport, TcpPointCloudClient
from .draco_compression import DracoCompression, PointCloudCompressor

__version__ = "1.0.0"
__all__ = ["TcpPointCloudTransport", "TcpPointCloudClient", "DracoCompression", "PointCloudCompressor"]

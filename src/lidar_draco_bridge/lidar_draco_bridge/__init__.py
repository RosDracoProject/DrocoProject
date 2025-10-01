"""
LiDAR-Draco Bridge Package

A ROS2 package for bridging LiDAR simulation data to Draco compression system.
Supports efficient point cloud compression and transmission using point_cloud_transport_py.
"""

__version__ = "1.0.0"
__author__ = "User"
__email__ = "user@example.com"

from .bridge import LidarDracoBridge
from .client import DracoClient

__all__ = [
    'LidarDracoBridge',
    'DracoClient',
]

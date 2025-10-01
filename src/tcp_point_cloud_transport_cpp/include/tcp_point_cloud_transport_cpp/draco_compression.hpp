// Copyright (c) 2024, TCP Point Cloud Transport C++ Plugin

#ifndef TCP_POINT_CLOUD_TRANSPORT_CPP__DRACO_COMPRESSION_HPP_
#define TCP_POINT_CLOUD_TRANSPORT_CPP__DRACO_COMPRESSION_HPP_

#include <vector>
#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace tcp_point_cloud_transport_cpp
{

/// Draco compression handler for point cloud data
class DracoCompression
{
public:
  DracoCompression();
  ~DracoCompression();

  /// Compress point cloud using Draco
  /// @param cloud Input point cloud
  /// @param compression_level Compression level (0-10)
  /// @return Compressed data
  std::vector<uint8_t> compress(
    const sensor_msgs::msg::PointCloud2 & cloud,
    int compression_level = 6) const;

  /// Decompress point cloud from Draco
  /// @param compressed_data Compressed data
  /// @return Decompressed point cloud
  sensor_msgs::msg::PointCloud2 decompress(
    const std::vector<uint8_t> & compressed_data) const;

  /// Check if Draco is available
  bool isAvailable() const;

  /// Get compression ratio
  float getCompressionRatio(
    const sensor_msgs::msg::PointCloud2 & original,
    const std::vector<uint8_t> & compressed) const;

private:
  bool draco_available_;
  
  /// Convert ROS2 PointCloud2 to Draco format
  std::vector<float> extractPoints(const sensor_msgs::msg::PointCloud2 & cloud) const;
  
  /// Convert Draco format back to ROS2 PointCloud2
  sensor_msgs::msg::PointCloud2 createPointCloud2(
    const std::vector<float> & points,
    const sensor_msgs::msg::PointCloud2 & original_cloud) const;
};

}  // namespace tcp_point_cloud_transport_cpp

#endif  // TCP_POINT_CLOUD_TRANSPORT_CPP__DRACO_COMPRESSION_HPP_

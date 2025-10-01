// Copyright (c) 2024, TCP Point Cloud Transport C++ Plugin

#include <tcp_point_cloud_transport_cpp/draco_compression.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstring>
#include <algorithm>

#ifdef HAVE_DRACO
// Draco includes would go here
// #include <draco/compression/encode.h>
// #include <draco/compression/decode.h>
// #include <draco/point_cloud/point_cloud.h>
// #include <draco/point_cloud/point_cloud_builder.h>
#endif

namespace tcp_point_cloud_transport_cpp
{

DracoCompression::DracoCompression()
{
#ifdef HAVE_DRACO
  draco_available_ = true;
#else
  draco_available_ = false;
#endif
}

DracoCompression::~DracoCompression()
{
}

std::vector<uint8_t> DracoCompression::compress(
  const sensor_msgs::msg::PointCloud2 & cloud,
  int compression_level) const
{
  if (!draco_available_) {
    // Fallback to raw data
    return std::vector<uint8_t>(cloud.data.begin(), cloud.data.end());
  }

#ifdef HAVE_DRACO
  // Real Draco compression would go here
  // For now, return raw data as placeholder
  return std::vector<uint8_t>(cloud.data.begin(), cloud.data.end());
#else
  // Fallback to raw data
  return std::vector<uint8_t>(cloud.data.begin(), cloud.data.end());
#endif
}

sensor_msgs::msg::PointCloud2 DracoCompression::decompress(
  const std::vector<uint8_t> & compressed_data) const
{
  if (!draco_available_) {
    // Fallback to raw data
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.data.assign(compressed_data.begin(), compressed_data.end());
    return cloud;
  }

#ifdef HAVE_DRACO
  // Real Draco decompression would go here
  // For now, return raw data as placeholder
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.data.assign(compressed_data.begin(), compressed_data.end());
  return cloud;
#else
  // Fallback to raw data
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.data.assign(compressed_data.begin(), compressed_data.end());
  return cloud;
#endif
}

bool DracoCompression::isAvailable() const
{
  return draco_available_;
}

float DracoCompression::getCompressionRatio(
  const sensor_msgs::msg::PointCloud2 & original,
  const std::vector<uint8_t> & compressed) const
{
  if (original.data.empty()) {
    return 1.0f;
  }
  
  return static_cast<float>(original.data.size()) / static_cast<float>(compressed.size());
}

std::vector<float> DracoCompression::extractPoints(const sensor_msgs::msg::PointCloud2 & cloud) const
{
  std::vector<float> points;
  
  if (cloud.data.empty() || cloud.point_step == 0) {
    return points;
  }
  
  // Extract XYZ coordinates from point cloud data
  // This is a simplified implementation
  size_t num_points = cloud.width * cloud.height;
  points.reserve(num_points * 3);
  
  for (size_t i = 0; i < num_points; ++i) {
    size_t offset = i * cloud.point_step;
    if (offset + 12 <= cloud.data.size()) {  // 3 floats * 4 bytes
      float x, y, z;
      std::memcpy(&x, cloud.data.data() + offset, sizeof(float));
      std::memcpy(&y, cloud.data.data() + offset + 4, sizeof(float));
      std::memcpy(&z, cloud.data.data() + offset + 8, sizeof(float));
      
      points.push_back(x);
      points.push_back(y);
      points.push_back(z);
    }
  }
  
  return points;
}

sensor_msgs::msg::PointCloud2 DracoCompression::createPointCloud2(
  const std::vector<float> & points,
  const sensor_msgs::msg::PointCloud2 & original_cloud) const
{
  sensor_msgs::msg::PointCloud2 cloud = original_cloud;
  
  // Reconstruct point cloud data from points
  cloud.data.clear();
  cloud.data.reserve(points.size() * sizeof(float));
  
  for (size_t i = 0; i < points.size(); i += 3) {
    if (i + 2 < points.size()) {
      cloud.data.insert(cloud.data.end(), 
                       reinterpret_cast<const uint8_t*>(&points[i]), 
                       reinterpret_cast<const uint8_t*>(&points[i]) + sizeof(float));
      cloud.data.insert(cloud.data.end(), 
                       reinterpret_cast<const uint8_t*>(&points[i + 1]), 
                       reinterpret_cast<const uint8_t*>(&points[i + 1]) + sizeof(float));
      cloud.data.insert(cloud.data.end(), 
                       reinterpret_cast<const uint8_t*>(&points[i + 2]), 
                       reinterpret_cast<const uint8_t*>(&points[i + 2]) + sizeof(float));
    }
  }
  
  return cloud;
}

}  // namespace tcp_point_cloud_transport_cpp

// Copyright (c) 2024, TCP Point Cloud Transport C++ Plugin

#include <tcp_point_cloud_transport_cpp/tcp_point_cloud_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstring>
#include <algorithm>

namespace tcp_point_cloud_transport_cpp
{

std::vector<uint8_t> TcpPointCloudMessage::serialize() const
{
  std::vector<uint8_t> data;
  
  // Serialize header
  uint32_t frame_id_size = static_cast<uint32_t>(frame_id.size());
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&frame_id_size), 
              reinterpret_cast<const uint8_t*>(&frame_id_size) + sizeof(frame_id_size));
  data.insert(data.end(), frame_id.begin(), frame_id.end());
  
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&timestamp_sec), 
              reinterpret_cast<const uint8_t*>(&timestamp_sec) + sizeof(timestamp_sec));
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&timestamp_nanosec), 
              reinterpret_cast<const uint8_t*>(&timestamp_nanosec) + sizeof(timestamp_nanosec));
  
  // Serialize compression info
  uint32_t compression_type_size = static_cast<uint32_t>(compression_type.size());
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&compression_type_size), 
              reinterpret_cast<const uint8_t*>(&compression_type_size) + sizeof(compression_type_size));
  data.insert(data.end(), compression_type.begin(), compression_type.end());
  
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&compression_level), 
              reinterpret_cast<const uint8_t*>(&compression_level) + sizeof(compression_level));
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&compression_ratio), 
              reinterpret_cast<const uint8_t*>(&compression_ratio) + sizeof(compression_ratio));
  
  // Serialize point cloud metadata
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&width), 
              reinterpret_cast<const uint8_t*>(&width) + sizeof(width));
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&height), 
              reinterpret_cast<const uint8_t*>(&height) + sizeof(height));
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&point_step), 
              reinterpret_cast<const uint8_t*>(&point_step) + sizeof(point_step));
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&row_step), 
              reinterpret_cast<const uint8_t*>(&row_step) + sizeof(row_step));
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&is_dense), 
              reinterpret_cast<const uint8_t*>(&is_dense) + sizeof(is_dense));
  
  // Serialize network info
  uint32_t server_address_size = static_cast<uint32_t>(server_address.size());
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&server_address_size), 
              reinterpret_cast<const uint8_t*>(&server_address_size) + sizeof(server_address_size));
  data.insert(data.end(), server_address.begin(), server_address.end());
  
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&server_port), 
              reinterpret_cast<const uint8_t*>(&server_port) + sizeof(server_port));
  
  // Serialize quality metrics
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&compression_time_ms), 
              reinterpret_cast<const uint8_t*>(&compression_time_ms) + sizeof(compression_time_ms));
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&transmission_time_ms), 
              reinterpret_cast<const uint8_t*>(&transmission_time_ms) + sizeof(transmission_time_ms));
  
  // Serialize compressed data
  uint32_t compressed_data_size = static_cast<uint32_t>(compressed_data.size());
  data.insert(data.end(), reinterpret_cast<const uint8_t*>(&compressed_data_size), 
              reinterpret_cast<const uint8_t*>(&compressed_data_size) + sizeof(compressed_data_size));
  data.insert(data.end(), compressed_data.begin(), compressed_data.end());
  
  return data;
}

TcpPointCloudMessage TcpPointCloudMessage::deserialize(const std::vector<uint8_t>& data)
{
  TcpPointCloudMessage msg;
  size_t offset = 0;
  
  // Deserialize header
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  uint32_t frame_id_size;
  std::memcpy(&frame_id_size, data.data() + offset, sizeof(frame_id_size));
  offset += sizeof(frame_id_size);
  
  if (offset + frame_id_size > data.size()) return msg;
  msg.frame_id.assign(data.begin() + offset, data.begin() + offset + frame_id_size);
  offset += frame_id_size;
  
  if (offset + sizeof(uint64_t) > data.size()) return msg;
  std::memcpy(&msg.timestamp_sec, data.data() + offset, sizeof(msg.timestamp_sec));
  offset += sizeof(msg.timestamp_sec);
  
  if (offset + sizeof(uint64_t) > data.size()) return msg;
  std::memcpy(&msg.timestamp_nanosec, data.data() + offset, sizeof(msg.timestamp_nanosec));
  offset += sizeof(msg.timestamp_nanosec);
  
  // Deserialize compression info
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  uint32_t compression_type_size;
  std::memcpy(&compression_type_size, data.data() + offset, sizeof(compression_type_size));
  offset += sizeof(compression_type_size);
  
  if (offset + compression_type_size > data.size()) return msg;
  msg.compression_type.assign(data.begin() + offset, data.begin() + offset + compression_type_size);
  offset += compression_type_size;
  
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  std::memcpy(&msg.compression_level, data.data() + offset, sizeof(msg.compression_level));
  offset += sizeof(msg.compression_level);
  
  if (offset + sizeof(float) > data.size()) return msg;
  std::memcpy(&msg.compression_ratio, data.data() + offset, sizeof(msg.compression_ratio));
  offset += sizeof(msg.compression_ratio);
  
  // Deserialize point cloud metadata
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  std::memcpy(&msg.width, data.data() + offset, sizeof(msg.width));
  offset += sizeof(msg.width);
  
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  std::memcpy(&msg.height, data.data() + offset, sizeof(msg.height));
  offset += sizeof(msg.height);
  
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  std::memcpy(&msg.point_step, data.data() + offset, sizeof(msg.point_step));
  offset += sizeof(msg.point_step);
  
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  std::memcpy(&msg.row_step, data.data() + offset, sizeof(msg.row_step));
  offset += sizeof(msg.row_step);
  
  if (offset + sizeof(bool) > data.size()) return msg;
  std::memcpy(&msg.is_dense, data.data() + offset, sizeof(msg.is_dense));
  offset += sizeof(msg.is_dense);
  
  // Deserialize network info
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  uint32_t server_address_size;
  std::memcpy(&server_address_size, data.data() + offset, sizeof(server_address_size));
  offset += sizeof(server_address_size);
  
  if (offset + server_address_size > data.size()) return msg;
  msg.server_address.assign(data.begin() + offset, data.begin() + offset + server_address_size);
  offset += server_address_size;
  
  if (offset + sizeof(uint16_t) > data.size()) return msg;
  std::memcpy(&msg.server_port, data.data() + offset, sizeof(msg.server_port));
  offset += sizeof(msg.server_port);
  
  // Deserialize quality metrics
  if (offset + sizeof(float) > data.size()) return msg;
  std::memcpy(&msg.compression_time_ms, data.data() + offset, sizeof(msg.compression_time_ms));
  offset += sizeof(msg.compression_time_ms);
  
  if (offset + sizeof(float) > data.size()) return msg;
  std::memcpy(&msg.transmission_time_ms, data.data() + offset, sizeof(msg.transmission_time_ms));
  offset += sizeof(msg.transmission_time_ms);
  
  // Deserialize compressed data
  if (offset + sizeof(uint32_t) > data.size()) return msg;
  uint32_t compressed_data_size;
  std::memcpy(&compressed_data_size, data.data() + offset, sizeof(compressed_data_size));
  offset += sizeof(compressed_data_size);
  
  if (offset + compressed_data_size > data.size()) return msg;
  msg.compressed_data.assign(data.begin() + offset, data.begin() + offset + compressed_data_size);
  
  return msg;
}

size_t TcpPointCloudMessage::getSize() const
{
  return sizeof(uint32_t) + frame_id.size() +  // frame_id_size + frame_id
         sizeof(uint64_t) + sizeof(uint64_t) +  // timestamp
         sizeof(uint32_t) + compression_type.size() +  // compression_type_size + compression_type
         sizeof(uint32_t) + sizeof(float) +  // compression_level + compression_ratio
         sizeof(uint32_t) * 4 + sizeof(bool) +  // point cloud metadata
         sizeof(uint32_t) + server_address.size() + sizeof(uint16_t) +  // network info
         sizeof(float) * 2 +  // quality metrics
         sizeof(uint32_t) + compressed_data.size();  // compressed_data_size + compressed_data
}

sensor_msgs::msg::PointCloud2 TcpPointCloudMessage::toPointCloud2() const
{
  sensor_msgs::msg::PointCloud2 cloud;
  
  // Set header
  cloud.header.frame_id = frame_id;
  cloud.header.stamp.sec = static_cast<int32_t>(timestamp_sec);
  cloud.header.stamp.nanosec = static_cast<uint32_t>(timestamp_nanosec);
  
  // Set point cloud metadata
  cloud.width = width;
  cloud.height = height;
  cloud.point_step = point_step;
  cloud.row_step = row_step;
  cloud.is_dense = is_dense;
  
  // Set data (this would need decompression in real implementation)
  cloud.data = compressed_data;
  
  return cloud;
}

TcpPointCloudMessage TcpPointCloudMessage::fromPointCloud2(const sensor_msgs::msg::PointCloud2& cloud)
{
  TcpPointCloudMessage msg;
  
  // Copy header information
  msg.frame_id = cloud.header.frame_id;
  msg.timestamp_sec = static_cast<uint64_t>(cloud.header.stamp.sec);
  msg.timestamp_nanosec = static_cast<uint64_t>(cloud.header.stamp.nanosec);
  
  // Copy point cloud metadata
  msg.width = cloud.width;
  msg.height = cloud.height;
  msg.point_step = cloud.point_step;
  msg.row_step = cloud.row_step;
  msg.is_dense = cloud.is_dense;
  
  // Set default compression settings
  msg.compression_type = "none";
  msg.compression_level = 6;
  msg.compression_ratio = 1.0f;
  
  // Copy data (this would need compression in real implementation)
  msg.compressed_data.assign(cloud.data.begin(), cloud.data.end());
  
  return msg;
}

}  // namespace tcp_point_cloud_transport_cpp

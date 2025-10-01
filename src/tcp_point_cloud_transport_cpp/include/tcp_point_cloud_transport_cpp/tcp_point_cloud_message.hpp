// Copyright (c) 2024, TCP Point Cloud Transport C++ Plugin
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef TCP_POINT_CLOUD_TRANSPORT_CPP__TCP_POINT_CLOUD_MESSAGE_HPP_
#define TCP_POINT_CLOUD_TRANSPORT_CPP__TCP_POINT_CLOUD_MESSAGE_HPP_

#include <stdint.h>
#include <vector>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace tcp_point_cloud_transport_cpp
{

/// TCP Point Cloud Message for compressed point cloud data transmission
struct TcpPointCloudMessage
{
  /// Header information
  std::string frame_id;
  uint64_t timestamp_sec;
  uint64_t timestamp_nanosec;
  
  /// Compression information
  std::string compression_type;  // "draco", "zlib", "none"
  uint32_t compression_level;
  float compression_ratio;
  
  /// Compressed data
  std::vector<uint8_t> compressed_data;
  
  /// Original point cloud metadata
  uint32_t width;
  uint32_t height;
  uint32_t point_step;
  uint32_t row_step;
  bool is_dense;
  
  /// Network information
  std::string server_address;
  uint16_t server_port;
  
  /// Quality metrics
  float compression_time_ms;
  float transmission_time_ms;
  
  TcpPointCloudMessage()
    : timestamp_sec(0)
    , timestamp_nanosec(0)
    , compression_level(6)
    , compression_ratio(1.0f)
    , width(0)
    , height(0)
    , point_step(0)
    , row_step(0)
    , is_dense(true)
    , server_port(0)
    , compression_time_ms(0.0f)
    , transmission_time_ms(0.0f)
  {
  }
  
  /// Serialize message to binary data
  std::vector<uint8_t> serialize() const;
  
  /// Deserialize message from binary data
  static TcpPointCloudMessage deserialize(const std::vector<uint8_t>& data);
  
  /// Get message size in bytes
  size_t getSize() const;
  
  /// Convert to ROS2 PointCloud2 message
  sensor_msgs::msg::PointCloud2 toPointCloud2() const;
  
  /// Create from ROS2 PointCloud2 message
  static TcpPointCloudMessage fromPointCloud2(const sensor_msgs::msg::PointCloud2& cloud);
};

}  // namespace tcp_point_cloud_transport_cpp

#endif  // TCP_POINT_CLOUD_TRANSPORT_CPP__TCP_POINT_CLOUD_MESSAGE_HPP_

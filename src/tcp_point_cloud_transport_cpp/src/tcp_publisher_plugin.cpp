// Copyright (c) 2024, TCP Point Cloud Transport C++ Plugin

#include <tcp_point_cloud_transport_cpp/tcp_publisher_plugin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace tcp_point_cloud_transport_cpp
{

TcpPublisherPlugin::TcpPublisherPlugin()
  : server_socket_(-1)
  , server_running_(false)
  , server_port_(8080)
  , compression_level_(6)
  , use_draco_(true)
  , use_compression_(true)
{
  server_address_ = "0.0.0.0";  // Listen on all interfaces
  compression_type_ = "draco";
  draco_compression_ = std::make_unique<DracoCompression>();
}

TcpPublisherPlugin::~TcpPublisherPlugin()
{
  stopTcpServer();
}

std::string TcpPublisherPlugin::getTransportName() const
{
  return "tcp";
}

std::string TcpPublisherPlugin::getDataType() const
{
  return "tcp_point_cloud_transport_cpp/TcpPointCloudMessage";
}

std::string TcpPublisherPlugin::getTopicToAdvertise(const std::string & base_topic) const
{
  return base_topic + "/tcp";
}

void TcpPublisherPlugin::declareParameters(const std::string & base_topic)
{
  if (parameters_) {
    parameters_->declare_parameter(base_topic + ".tcp.server_address", server_address_);
    parameters_->declare_parameter(base_topic + ".tcp.server_port", static_cast<int>(server_port_));
    parameters_->declare_parameter(base_topic + ".tcp.compression_type", compression_type_);
    parameters_->declare_parameter(base_topic + ".tcp.compression_level", compression_level_);
    parameters_->declare_parameter(base_topic + ".tcp.use_draco", use_draco_);
    parameters_->declare_parameter(base_topic + ".tcp.use_compression", use_compression_);
  }
}

void TcpPublisherPlugin::advertiseImpl(
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
  const std::string & base_topic,
  rclcpp::QoS custom_qos,
  const rclcpp::PublisherOptions & options)
{
  logger_ = node_interfaces.get_node_logging_interface();
  parameters_ = node_interfaces.get_node_parameters_interface();
  
  // Get parameters
  if (parameters_) {
    try {
      server_address_ = parameters_->get_parameter(base_topic + ".tcp.server_address").as_string();
      server_port_ = static_cast<uint16_t>(parameters_->get_parameter(base_topic + ".tcp.server_port").as_int());
      compression_type_ = parameters_->get_parameter(base_topic + ".tcp.compression_type").as_string();
      compression_level_ = parameters_->get_parameter(base_topic + ".tcp.compression_level").as_int();
      use_draco_ = parameters_->get_parameter(base_topic + ".tcp.use_draco").as_bool();
      use_compression_ = parameters_->get_parameter(base_topic + ".tcp.use_compression").as_bool();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_->get_logger(), "Failed to get TCP parameters: %s", e.what());
    }
  }
  
  // Start TCP server
  startTcpServer();
  
  // Call parent implementation
  SimplePublisherPlugin<TcpPointCloudMessage>::advertiseImpl(
    node_interfaces, base_topic, custom_qos, options);
}

rclcpp::expected<TcpPointCloudMessage, std::string> TcpPublisherPlugin::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  return compressPointCloud(raw);
}

TcpPointCloudMessage TcpPublisherPlugin::compressPointCloud(const sensor_msgs::msg::PointCloud2 & cloud) const
{
  TcpPointCloudMessage tcp_msg;
  
  // Copy header information
  tcp_msg.frame_id = cloud.header.frame_id;
  tcp_msg.timestamp_sec = cloud.header.stamp.sec;
  tcp_msg.timestamp_nanosec = cloud.header.stamp.nanosec;
  
  // Copy point cloud metadata
  tcp_msg.width = cloud.width;
  tcp_msg.height = cloud.height;
  tcp_msg.point_step = cloud.point_step;
  tcp_msg.row_step = cloud.row_step;
  tcp_msg.is_dense = cloud.is_dense;
  
  // Set compression parameters
  tcp_msg.compression_type = compression_type_;
  tcp_msg.compression_level = compression_level_;
  tcp_msg.server_address = server_address_;
  tcp_msg.server_port = server_port_;
  
  // Compress the data
  if (use_compression_ && use_draco_ && compression_type_ == "draco") {
    tcp_msg.compressed_data = draco_compression_->compress(cloud, compression_level_);
  } else {
    // Fallback to simple compression or no compression
    tcp_msg.compressed_data.assign(cloud.data.begin(), cloud.data.end());
  }
  
  // Calculate compression ratio
  if (!tcp_msg.compressed_data.empty() && !cloud.data.empty()) {
    tcp_msg.compression_ratio = static_cast<float>(cloud.data.size()) / 
                               static_cast<float>(tcp_msg.compressed_data.size());
  } else {
    tcp_msg.compression_ratio = 1.0f;
  }
  
  return tcp_msg;
}

void TcpPublisherPlugin::startTcpServer()
{
  if (server_running_) {
    return;
  }
  
  server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket_ < 0) {
    RCLCPP_ERROR(logger_->get_logger(), "Failed to create TCP socket");
    return;
  }
  
  // Set socket options
  int opt = 1;
  setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  
  // Bind socket
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = inet_addr(server_address_.c_str());
  server_addr.sin_port = htons(server_port_);
  
  if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(logger_->get_logger(), "Failed to bind TCP socket to %s:%d", 
                 server_address_.c_str(), server_port_);
    close(server_socket_);
    server_socket_ = -1;
    return;
  }
  
  // Start listening
  if (listen(server_socket_, 5) < 0) {
    RCLCPP_ERROR(logger_->get_logger(), "Failed to listen on TCP socket");
    close(server_socket_);
    server_socket_ = -1;
    return;
  }
  
  server_running_ = true;
  server_thread_ = std::thread(&TcpPublisherPlugin::tcpServerLoop, this);
  
  RCLCPP_INFO(logger_->get_logger(), "TCP server started on %s:%d", 
              server_address_.c_str(), server_port_);
}

void TcpPublisherPlugin::stopTcpServer()
{
  if (!server_running_) {
    return;
  }
  
  server_running_ = false;
  
  if (server_socket_ >= 0) {
    close(server_socket_);
    server_socket_ = -1;
  }
  
  if (server_thread_.joinable()) {
    server_thread_.join();
  }
  
  // Close all client connections
  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (int client_socket : connected_clients_) {
    close(client_socket);
  }
  connected_clients_.clear();
  
  RCLCPP_INFO(logger_->get_logger(), "TCP server stopped");
}

void TcpPublisherPlugin::tcpServerLoop()
{
  while (server_running_) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
    if (client_socket < 0) {
      if (server_running_) {
        RCLCPP_WARN(logger_->get_logger(), "Failed to accept client connection");
      }
      continue;
    }
    
    RCLCPP_INFO(logger_->get_logger(), "New client connected from %s:%d", 
                inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
    
    // Add client to list
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      connected_clients_.push_back(client_socket);
    }
    
    // Handle client in separate thread
    std::thread client_thread(&TcpPublisherPlugin::handleClientConnection, this, client_socket);
    client_thread.detach();
  }
}

void TcpPublisherPlugin::handleClientConnection(int client_socket)
{
  // This would be called when publishing data to connected clients
  // For now, just keep the connection alive
  char buffer[1024];
  while (server_running_) {
    ssize_t bytes_received = recv(client_socket, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (bytes_received <= 0) {
      break;
    }
    // Handle client messages if needed
  }
  
  close(client_socket);
  
  // Remove client from list
  std::lock_guard<std::mutex> lock(clients_mutex_);
  connected_clients_.erase(
    std::remove(connected_clients_.begin(), connected_clients_.end(), client_socket),
    connected_clients_.end());
}

void TcpPublisherPlugin::broadcastToClients(const TcpPointCloudMessage & message)
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  
  for (auto it = connected_clients_.begin(); it != connected_clients_.end();) {
    try {
      sendToClient(message, *it);
      ++it;
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_->get_logger(), "Error sending to client: %s", e.what());
      close(*it);
      it = connected_clients_.erase(it);
    }
  }
}

void TcpPublisherPlugin::sendToClient(const TcpPointCloudMessage & message, int client_socket)
{
  // Serialize the message
  std::vector<uint8_t> serialized_data = message.serialize();
  
  // Send data size first
  uint32_t data_size = static_cast<uint32_t>(serialized_data.size());
  send(client_socket, &data_size, sizeof(data_size), 0);
  
  // Send serialized data
  send(client_socket, serialized_data.data(), serialized_data.size(), 0);
}

}  // namespace tcp_point_cloud_transport_cpp

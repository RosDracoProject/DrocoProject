// Copyright (c) 2024, TCP Point Cloud Transport C++ Plugin

#ifndef TCP_POINT_CLOUD_TRANSPORT_CPP__TCP_PUBLISHER_PLUGIN_HPP_
#define TCP_POINT_CLOUD_TRANSPORT_CPP__TCP_PUBLISHER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <queue>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include <point_cloud_transport/simple_publisher_plugin.hpp>
#include <tcp_point_cloud_transport_cpp/tcp_point_cloud_message.hpp>
#include <tcp_point_cloud_transport_cpp/draco_compression.hpp>

namespace tcp_point_cloud_transport_cpp
{

/// TCP Publisher Plugin for point cloud transport
class TcpPublisherPlugin : public point_cloud_transport::SimplePublisherPlugin<TcpPointCloudMessage>
{
public:
  TcpPublisherPlugin();
  virtual ~TcpPublisherPlugin();

  std::string getTransportName() const override;
  std::string getDataType() const override;
  std::string getTopicToAdvertise(const std::string & base_topic) const override;

  void declareParameters(const std::string & base_topic) override;

protected:
  void advertiseImpl(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    rclcpp::QoS custom_qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions()) override;

  rclcpp::expected<TcpPointCloudMessage, std::string> encodeTyped(
    const sensor_msgs::msg::PointCloud2 & raw) const override;

private:
  /// TCP Server functionality
  void startTcpServer();
  void stopTcpServer();
  void tcpServerLoop();
  void handleClientConnection(int client_socket);
  void broadcastToClients(const TcpPointCloudMessage & message);
  void sendToClient(const TcpPointCloudMessage & message, int client_socket);
  
  /// Compression functionality
  TcpPointCloudMessage compressPointCloud(const sensor_msgs::msg::PointCloud2 & cloud) const;
  
  /// Network parameters
  std::string server_address_;
  uint16_t server_port_;
  int server_socket_;
  std::atomic<bool> server_running_;
  std::thread server_thread_;
  std::mutex clients_mutex_;
  std::vector<int> connected_clients_;
  
  /// Compression parameters
  std::string compression_type_;
  int compression_level_;
  bool use_draco_;
  bool use_compression_;
  
  /// ROS2 components
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  
  /// Draco compression handler
  std::unique_ptr<DracoCompression> draco_compression_;
};

}  // namespace tcp_point_cloud_transport_cpp

#endif  // TCP_POINT_CLOUD_TRANSPORT_CPP__TCP_PUBLISHER_PLUGIN_HPP_

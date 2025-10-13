#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <vector>
#include <memory>
#include <chrono>
#include <mutex>
#include <atomic>
#include <zlib.h>

class SimpleDracoBridge : public rclcpp::Node
{
public:
    SimpleDracoBridge() : Node("simple_draco_bridge")
    {
        // Parameters
        this->declare_parameter("server_port", 8888);
        this->declare_parameter("input_topic", "/sensing/lidar/top/pointcloud_raw_ex");
        this->declare_parameter("output_topic", "/lidar/compressed");
        this->declare_parameter("compression_level", 6);  // 0-9, 기본값 6
        
        server_port_ = this->get_parameter("server_port").as_int();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        compression_level_ = this->get_parameter("compression_level").as_int();
        
        // QoS profile for high frequency
        auto qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        // Publishers and subscribers
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, qos);
        
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&SimpleDracoBridge::pointCloudCallback, this, std::placeholders::_1));
        
        // Statistics
        message_count_ = 0;
        total_original_size_ = 0;
        total_compressed_size_ = 0;
        
        // Start TCP server
        startServer();
        
        RCLCPP_INFO(this->get_logger(), "Simple Draco Bridge Server started");
        RCLCPP_INFO(this->get_logger(), "Listening on port: %d", server_port_);
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Compression level: %d (0=none, 9=max)", compression_level_);
    }
    
    ~SimpleDracoBridge()
    {
        stopServer();
    }

private:
    void startServer()
    {
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }
        
        // Set socket options
        int opt = 1;
        setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // Increase buffer sizes for high throughput
        int buffer_size = 4 * 1024 * 1024; // 4MB
        setsockopt(server_socket_, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
        setsockopt(server_socket_, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
        
        // Bind socket
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(server_port_);
        
        if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            close(server_socket_);
            return;
        }
        
        // Listen for connections
        if (listen(server_socket_, 10) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to listen on socket");
            close(server_socket_);
            return;
        }
        
        // Start server thread
        server_thread_ = std::thread(&SimpleDracoBridge::serverLoop, this);
        server_running_ = true;
        
        RCLCPP_INFO(this->get_logger(), "TCP server started on port %d", server_port_);
    }
    
    void stopServer()
    {
        server_running_ = false;
        if (server_socket_ >= 0) {
            close(server_socket_);
        }
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
        
        // Close all client connections
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (int client_socket : client_sockets_) {
            close(client_socket);
        }
        client_sockets_.clear();
    }
    
    void serverLoop()
    {
        while (server_running_) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
            if (client_socket < 0) {
                if (server_running_) {
                    RCLCPP_WARN(this->get_logger(), "Failed to accept client connection");
                }
                continue;
            }
            
            // Set TCP_NODELAY for low latency
            int flag = 1;
            setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
            
            // Add client to list
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                client_sockets_.push_back(client_socket);
            }
            
            RCLCPP_INFO(this->get_logger(), "New client connected: %s", 
                       inet_ntoa(client_addr.sin_addr));
        }
    }
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // Compress with zlib
            std::vector<uint8_t> compressed_data = compressData(msg->data);
            if (compressed_data.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to compress point cloud");
                return;
            }
            
            // Create compressed PointCloud2 message
            auto compressed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            compressed_msg->header = msg->header;
            compressed_msg->height = msg->height;
            compressed_msg->width = msg->width;
            compressed_msg->fields = msg->fields;
            compressed_msg->is_bigendian = msg->is_bigendian;
            compressed_msg->point_step = msg->point_step;
            compressed_msg->row_step = msg->row_step;
            compressed_msg->data = compressed_data;
            compressed_msg->is_dense = msg->is_dense;
            
            // Publish compressed data
            compressed_publisher_->publish(*compressed_msg);
            
            // Send to TCP clients
            sendToClients(compressed_data);
            
            // Update statistics
            message_count_++;
            total_original_size_ += msg->data.size();
            total_compressed_size_ += compressed_data.size();
            
            if (message_count_ % 10 == 0) {
                double compression_ratio = static_cast<double>(total_original_size_) / total_compressed_size_;
                double compression_percentage = (1.0 - static_cast<double>(total_compressed_size_) / total_original_size_) * 100.0;
                
                RCLCPP_INFO(this->get_logger(), 
                           "=== 압축 통계 ===");
                RCLCPP_INFO(this->get_logger(), 
                           "처리된 메시지: %zu개", message_count_.load());
                RCLCPP_INFO(this->get_logger(), 
                           "원본 크기: %.2f MB", total_original_size_ / (1024.0 * 1024.0));
                RCLCPP_INFO(this->get_logger(), 
                           "압축 크기: %.2f MB", total_compressed_size_ / (1024.0 * 1024.0));
                RCLCPP_INFO(this->get_logger(), 
                           "압축률: %.2f:1", compression_ratio);
                RCLCPP_INFO(this->get_logger(), 
                           "압축 비율: %.1f%%", compression_percentage);
                RCLCPP_INFO(this->get_logger(), 
                           "현재 메시지 크기: %zu bytes -> %zu bytes", 
                           msg->data.size(), compressed_data.size());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in point cloud callback: %s", e.what());
        }
    }
    
    std::vector<uint8_t> compressData(const std::vector<uint8_t>& data)
    {
        try {
            uLongf compressed_size = compressBound(data.size());
            std::vector<uint8_t> compressed_data(compressed_size);
            
            int result = compress2(compressed_data.data(), &compressed_size,
                                 data.data(), data.size(), compression_level_);
            
            if (result == Z_OK) {
                compressed_data.resize(compressed_size);
                return compressed_data;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Compression failed with code: %d", result);
                return {};
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error compressing data: %s", e.what());
            return {};
        }
    }
    
    std::vector<uint8_t> decompressData(const std::vector<uint8_t>& compressed_data)
    {
        try {
            // Estimate decompressed size (this is a simple approach)
            uLongf decompressed_size = compressed_data.size() * 4; // Rough estimate
            std::vector<uint8_t> decompressed_data(decompressed_size);
            
            int result = uncompress(decompressed_data.data(), &decompressed_size,
                                  compressed_data.data(), compressed_data.size());
            
            if (result == Z_OK) {
                decompressed_data.resize(decompressed_size);
                return decompressed_data;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Decompression failed with code: %d", result);
                return {};
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error decompressing data: %s", e.what());
            return {};
        }
    }
    
    void sendToClients(const std::vector<uint8_t>& data)
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        
        // Create message header
        std::string header = "POINT_CLOUD:" + std::to_string(data.size()) + "\n";
        
        for (auto it = client_sockets_.begin(); it != client_sockets_.end();) {
            int client_socket = *it;
            
            try {
                // Send header
                ssize_t sent = send(client_socket, header.c_str(), header.size(), MSG_NOSIGNAL);
                if (sent < 0) {
                    RCLCPP_WARN(this->get_logger(), "Failed to send header to client");
                    close(client_socket);
                    it = client_sockets_.erase(it);
                    continue;
                }
                
                // Send data in chunks
                size_t total_sent = 0;
                const size_t chunk_size = 65536; // 64KB chunks for better TCP performance
                
                while (total_sent < data.size()) {
                    size_t to_send = std::min(chunk_size, data.size() - total_sent);
                    ssize_t sent = send(client_socket, data.data() + total_sent, to_send, MSG_NOSIGNAL);
                    
                    if (sent < 0) {
                        RCLCPP_WARN(this->get_logger(), "Failed to send data to client");
                        close(client_socket);
                        it = client_sockets_.erase(it);
                        break;
                    }
                    
                    total_sent += sent;
                }
                
                if (total_sent == data.size()) {
                    ++it;
                }
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Error sending to client: %s", e.what());
                close(client_socket);
                it = client_sockets_.erase(it);
            }
        }
    }
    
    // Parameters
    int server_port_;
    std::string input_topic_;
    std::string output_topic_;
    int compression_level_;  // 0-9, 0=no compression, 9=max compression
    
    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr compressed_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    
    // TCP Server
    int server_socket_ = -1;
    std::thread server_thread_;
    std::atomic<bool> server_running_{false};
    std::vector<int> client_sockets_;
    std::mutex clients_mutex_;
    
    // Statistics
    std::atomic<size_t> message_count_{0};
    std::atomic<size_t> total_original_size_{0};
    std::atomic<size_t> total_compressed_size_{0};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleDracoBridge>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

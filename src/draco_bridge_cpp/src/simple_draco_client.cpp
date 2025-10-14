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

class SimpleDracoClient : public rclcpp::Node
{
public:
    SimpleDracoClient() : Node("simple_draco_client")
    {
        // Parameters
        this->declare_parameter("server_host", "192.168.3.22");
        this->declare_parameter("server_port", 8888);
        this->declare_parameter("output_topic", "/lidar/decompressed");
        
        server_host_ = this->get_parameter("server_host").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        output_topic_ = this->get_parameter("output_topic").as_string();
        
        // QoS profile
        auto qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        // Publisher
        decompressed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, qos);
        
        // Statistics
        message_count_ = 0;
        total_bytes_received_ = 0;
        
        // Connect to server
        if (connectToServer()) {
            // Start receiving thread
            receive_thread_ = std::thread(&SimpleDracoClient::receiveLoop, this);
            receiving_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Connected to Draco server at %s:%d", 
                       server_host_.c_str(), server_port_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Draco server");
        }
    }
    
    ~SimpleDracoClient()
    {
        disconnect();
    }

private:
    bool connectToServer()
    {
        client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (client_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return false;
        }
        
        // Set TCP_NODELAY for low latency
        int flag = 1;
        setsockopt(client_socket_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        
        // Increase buffer sizes
        int buffer_size = 4 * 1024 * 1024; // 4MB
        setsockopt(client_socket_, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
        setsockopt(client_socket_, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
        
        // Connect to server
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_);
        
        if (inet_pton(AF_INET, server_host_.c_str(), &server_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid server address: %s", server_host_.c_str());
            close(client_socket_);
            return false;
        }
        
        if (connect(client_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to server");
            close(client_socket_);
            return false;
        }
        
        return true;
    }
    
    void disconnect()
    {
        receiving_ = false;
        
        if (client_socket_ >= 0) {
            close(client_socket_);
            client_socket_ = -1;
        }
        
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        
        RCLCPP_INFO(this->get_logger(), "Disconnected from Draco server");
    }
    
    void receiveLoop()
    {
        while (receiving_ && client_socket_ >= 0) {
            try {
                // Read header
                std::string header = readLine();
                if (header.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Empty header received, disconnecting");
                    break;
                }
                
                // Parse header
                if (header.find("POINT_CLOUD:") != 0) {
                    RCLCPP_WARN(this->get_logger(), "Invalid header format: %s", header.c_str());
                    continue;
                }
                
                size_t colon_pos = header.find(':');
                if (colon_pos == std::string::npos) {
                    RCLCPP_WARN(this->get_logger(), "Invalid header format: %s", header.c_str());
                    continue;
                }
                
                size_t data_size = std::stoul(header.substr(colon_pos + 1));
                
                // Read compressed data
                std::vector<uint8_t> compressed_data(data_size);
                if (!readData(compressed_data.data(), data_size)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to read compressed data");
                    break;
                }
                
                // Decompress and publish
                processCompressedData(compressed_data);
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error in receive loop: %s", e.what());
                break;
            }
        }
        
        receiving_ = false;
        RCLCPP_INFO(this->get_logger(), "Receive loop ended");
    }
    
    std::string readLine()
    {
        std::string line;
        char c;
        
        while (receiving_ && client_socket_ >= 0) {
            ssize_t received = recv(client_socket_, &c, 1, 0);
            if (received <= 0) {
                return "";
            }
            
            if (c == '\n') {
                break;
            }
            
            line += c;
        }
        
        return line;
    }
    
    bool readData(void* buffer, size_t size)
    {
        size_t total_received = 0;
        uint8_t* data_ptr = static_cast<uint8_t*>(buffer);
        
        while (total_received < size && receiving_ && client_socket_ >= 0) {
            ssize_t received = recv(client_socket_, data_ptr + total_received, 
                                  size - total_received, 0);
            if (received <= 0) {
                return false;
            }
            
            total_received += received;
        }
        
        return total_received == size;
    }
    
    void processCompressedData(const std::vector<uint8_t>& compressed_data)
    {
        try {
            // Decompress with zlib
            std::vector<uint8_t> decompressed_data = decompressData(compressed_data);
            if (decompressed_data.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decompress point cloud");
                return;
            }
            
            // Create PointCloud2 message
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            msg->header.stamp = this->now();
            msg->header.frame_id = "rslidar_top";
            
            // Estimate point count from decompressed data
            int point_step = 16; // 4 floats per point
            int num_points = decompressed_data.size() / point_step;
            
            msg->width = num_points;
            msg->height = 1;
            msg->point_step = point_step;
            msg->row_step = msg->width * msg->point_step;
            msg->is_dense = true;
            
            // Set fields
            sensor_msgs::msg::PointField field;
            field.name = "x";
            field.offset = 0;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            msg->fields.push_back(field);
            
            field.name = "y";
            field.offset = 4;
            msg->fields.push_back(field);
            
            field.name = "z";
            field.offset = 8;
            msg->fields.push_back(field);
            
            field.name = "intensity";
            field.offset = 12;
            msg->fields.push_back(field);
            
            msg->data = decompressed_data;
            
            // Publish
            decompressed_publisher_->publish(*msg);
            
            // Update statistics
            message_count_++;
            total_bytes_received_ += compressed_data.size();
            
            if (message_count_ % 10 == 0) {
                double compression_ratio = static_cast<double>(decompressed_data.size()) / compressed_data.size();
                RCLCPP_INFO(this->get_logger(), 
                           "=== TCP/IP 수신 통계 ===");
                RCLCPP_INFO(this->get_logger(), 
                           "수신된 메시지: %zu개", message_count_.load());
                RCLCPP_INFO(this->get_logger(), 
                           "총 수신 바이트: %.2f MB", total_bytes_received_ / (1024.0 * 1024.0));
                RCLCPP_INFO(this->get_logger(), 
                           "압축률: %.2f:1", compression_ratio);
                RCLCPP_INFO(this->get_logger(), 
                           "현재 메시지: %zu bytes -> %zu bytes", 
                           compressed_data.size(), decompressed_data.size());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing compressed data: %s", e.what());
        }
    }
    
    std::vector<uint8_t> decompressData(const std::vector<uint8_t>& compressed_data)
    {
        try {
            // Estimate decompressed size
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
    
    // Parameters
    std::string server_host_;
    int server_port_;
    std::string output_topic_;
    
    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr decompressed_publisher_;
    
    // TCP Client
    int client_socket_ = -1;
    std::thread receive_thread_;
    std::atomic<bool> receiving_{false};
    
    // Statistics
    std::atomic<size_t> message_count_{0};
    std::atomic<size_t> total_bytes_received_{0};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleDracoClient>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

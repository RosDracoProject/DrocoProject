#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <draco/compression/encode.h>
#include <draco/compression/decode.h>
#include <draco/point_cloud/point_cloud.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <vector>
#include <memory>
#include <chrono>
#include <sstream>
#include <mutex>
#include <atomic>

class DracoBridgeClient : public rclcpp::Node
{
public:
    DracoBridgeClient() : Node("draco_bridge_client")
    {
        // Parameters
        this->declare_parameter("server_host", "localhost");
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
            receive_thread_ = std::thread(&DracoBridgeClient::receiveLoop, this);
            receiving_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Connected to Draco server at %s:%d", 
                       server_host_.c_str(), server_port_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Draco server");
        }
    }
    
    ~DracoBridgeClient()
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
        int buffer_size = 1024 * 1024; // 1MB
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
            // Decompress with Draco
            auto point_cloud = decompressPointCloud(compressed_data);
            if (!point_cloud) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decompress point cloud");
                return;
            }
            
            // Convert to ROS PointCloud2
            auto msg = convertFromDracoPointCloud(point_cloud.get());
            if (!msg) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert from Draco point cloud");
                return;
            }
            
            // Publish
            decompressed_publisher_->publish(*msg);
            
            // Update statistics
            message_count_++;
            total_bytes_received_ += compressed_data.size();
            
            if (message_count_ % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "Received %zu messages, Total bytes: %zu", 
                           message_count_.load(), total_bytes_received_.load());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing compressed data: %s", e.what());
        }
    }
    
    std::unique_ptr<draco::PointCloud> decompressPointCloud(const std::vector<uint8_t>& compressed_data)
    {
        try {
            draco::Decoder decoder;
            draco::DecoderBuffer buffer;
            
            buffer.Init(reinterpret_cast<const char*>(compressed_data.data()), compressed_data.size());
            
            auto geometry_type = decoder.GetEncodedGeometryType(&buffer);
            if (!geometry_type.ok()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get geometry type");
                return nullptr;
            }
            
            if (geometry_type.value() == draco::TRIANGULAR_MESH) {
                // Try as point cloud
                auto point_cloud = decoder.DecodePointCloudFromBuffer(&buffer);
                if (point_cloud.ok()) {
                    return std::move(point_cloud.value());
                }
            } else if (geometry_type.value() == draco::POINT_CLOUD) {
                auto point_cloud = decoder.DecodePointCloudFromBuffer(&buffer);
                if (point_cloud.ok()) {
                    return std::move(point_cloud.value());
                }
            }
            
            RCLCPP_ERROR(this->get_logger(), "Failed to decode point cloud");
            return nullptr;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error decompressing point cloud: %s", e.what());
            return nullptr;
        }
    }
    
    sensor_msgs::msg::PointCloud2::SharedPtr convertFromDracoPointCloud(draco::PointCloud* point_cloud)
    {
        try {
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            
            // Set header
            msg->header.stamp = this->now();
            msg->header.frame_id = "rslidar_top";
            
            int num_points = point_cloud->num_points();
            if (num_points == 0) {
                return nullptr;
            }
            
            // Set dimensions
            msg->width = num_points;
            msg->height = 1;
            msg->point_step = 16; // 3 * float32 (position) + 1 * float32 (intensity)
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
            
            // Fill data
            msg->data.resize(num_points * msg->point_step);
            
            auto pos_att = point_cloud->attribute(0); // Position attribute
            auto intensity_att = point_cloud->attribute(1); // Intensity attribute
            
            for (int i = 0; i < num_points; ++i) {
                float* data_ptr = reinterpret_cast<float*>(msg->data.data() + i * msg->point_step);
                
                // Position
                draco::Vector3f pos;
                pos_att->GetValue(draco::AttributeValueIndex(i), &pos[0]);
                data_ptr[0] = pos[0];
                data_ptr[1] = pos[1];
                data_ptr[2] = pos[2];
                
                // Intensity
                float intensity;
                intensity_att->GetValue(draco::AttributeValueIndex(i), &intensity);
                data_ptr[3] = intensity;
            }
            
            return msg;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error converting from Draco point cloud: %s", e.what());
            return nullptr;
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
    
    auto node = std::make_shared<DracoBridgeClient>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

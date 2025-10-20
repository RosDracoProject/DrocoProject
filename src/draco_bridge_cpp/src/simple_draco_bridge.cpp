#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <draco/compression/encode.h>
#include <draco/compression/decode.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/attributes/point_attribute.h>

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

class SimpleDracoBridge : public rclcpp::Node
{
public:
    SimpleDracoBridge() : Node("simple_draco_bridge")
    {
        // Parameters
        this->declare_parameter("server_port", 8888);
        this->declare_parameter("input_topic", "/sensing/lidar/top/pointcloud_raw_ex");
        this->declare_parameter("output_topic", "/lidar/compressed");
        this->declare_parameter("quantization_bits", 11);  // Draco quantization bits (default: 11)
        this->declare_parameter("compression_speed", 5);   // Draco speed (0-10, default: 5)
        
        server_port_ = this->get_parameter("server_port").as_int();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        quantization_bits_ = this->get_parameter("quantization_bits").as_int();
        compression_speed_ = this->get_parameter("compression_speed").as_int();
        
        // QoS profile for sensor data (LiDAR typically uses BEST_EFFORT)
        auto qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
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
        RCLCPP_INFO(this->get_logger(), "Quantization bits: %d", quantization_bits_);
        RCLCPP_INFO(this->get_logger(), "Compression speed: %d (0=slowest, 9=fastest)", compression_speed_);
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
            // Convert ROS PointCloud2 to Draco PointCloud
            auto draco_point_cloud = convertToDracoPointCloud(msg);
            if (!draco_point_cloud) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert to Draco point cloud");
                return;
            }
            
            // Compress with Draco
            std::vector<uint8_t> compressed_data = compressPointCloud(draco_point_cloud.get());
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
            
            // Send to TCP clients (with header info)
            sendToClients(compressed_data, msg);
            
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
    
    std::unique_ptr<draco::PointCloud> convertToDracoPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            auto point_cloud = std::make_unique<draco::PointCloud>();
            
            // Get point count
            int num_points = msg->width * msg->height;
            if (num_points == 0) {
                RCLCPP_WARN(this->get_logger(), "Point cloud is empty (num_points=0)");
                return nullptr;
            }
            
            // Check data size
            if (msg->data.empty()) {
                RCLCPP_WARN(this->get_logger(), "Point cloud data is empty");
                return nullptr;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Converting point cloud: %d points, data size: %zu bytes, point_step: %u", 
                        num_points, msg->data.size(), msg->point_step);
            
            // Log field information for debugging
            RCLCPP_DEBUG(this->get_logger(), "Point cloud fields:");
            for (const auto& field : msg->fields) {
                RCLCPP_DEBUG(this->get_logger(), "  - %s: offset=%u, datatype=%u, count=%u", 
                           field.name.c_str(), field.offset, field.datatype, field.count);
            }
            
            // Set number of points first
            point_cloud->set_num_points(num_points);
            
            // Prepare position and intensity data
            std::vector<float> positions(num_points * 3);
            std::vector<float> intensities(num_points);
            
            const uint8_t* data_ptr = msg->data.data();
            int point_step = msg->point_step;
            
            // Extract data from ROS message
            int valid_point_count = 0;
            for (int i = 0; i < num_points; ++i) {
                // Check if we have enough data
                if (static_cast<size_t>((i + 1) * point_step) > msg->data.size()) {
                    RCLCPP_WARN(this->get_logger(), "Insufficient data at point %d", i);
                    break;
                }
                
                // Position (x, y, z) - assuming they are at offset 0, 4, 8
                float x = *reinterpret_cast<const float*>(data_ptr + i * point_step + 0);
                float y = *reinterpret_cast<const float*>(data_ptr + i * point_step + 4);
                float z = *reinterpret_cast<const float*>(data_ptr + i * point_step + 8);
                
                // Check for invalid values
                if (std::isnan(x) || std::isnan(y) || std::isnan(z) ||
                    std::isinf(x) || std::isinf(y) || std::isinf(z)) {
                    // Skip invalid points
                    x = y = z = 0.0f;
                } else if (x != 0.0f || y != 0.0f || z != 0.0f) {
                    // Count valid non-zero points
                    valid_point_count++;
                }
                
                positions[i * 3 + 0] = x;
                positions[i * 3 + 1] = y;
                positions[i * 3 + 2] = z;
                
                // Intensity - assuming it's at offset 12
                float intensity = 0.0f;
                if (point_step >= 16) {
                    intensity = *reinterpret_cast<const float*>(data_ptr + i * point_step + 12);
                    if (std::isnan(intensity) || std::isinf(intensity)) {
                        intensity = 0.0f;
                    }
                }
                intensities[i] = intensity;
            }
            
            // Only warn if most points are invalid (less than 1% valid)
            if (valid_point_count < num_points / 100) {
                RCLCPP_WARN(this->get_logger(), "Point cloud has very few valid points: %d/%d (%.1f%%)", 
                           valid_point_count, num_points, 
                           100.0f * valid_point_count / num_points);
            }
            
            // Create position attribute with pre-allocated data
            draco::GeometryAttribute pos_att;
            pos_att.Init(draco::GeometryAttribute::POSITION, nullptr, 3, draco::DT_FLOAT32, false, 
                        sizeof(float) * 3, 0);
            int pos_att_id = point_cloud->AddAttribute(pos_att, true, num_points);
            
            // Create intensity attribute with pre-allocated data
            draco::GeometryAttribute intensity_att;
            intensity_att.Init(draco::GeometryAttribute::GENERIC, nullptr, 1, draco::DT_FLOAT32, false, 
                             sizeof(float), 0);
            int intensity_att_id = point_cloud->AddAttribute(intensity_att, true, num_points);
            
            // Get attribute pointers and set values
            auto pos_att_ptr = point_cloud->attribute(pos_att_id);
            auto intensity_att_ptr = point_cloud->attribute(intensity_att_id);
            
            // Set attribute values
            for (int i = 0; i < num_points; ++i) {
                pos_att_ptr->SetAttributeValue(draco::AttributeValueIndex(i), &positions[i * 3]);
                intensity_att_ptr->SetAttributeValue(draco::AttributeValueIndex(i), &intensities[i]);
            }
            
            return point_cloud;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error converting to Draco point cloud: %s", e.what());
            return nullptr;
        }
    }
    
    std::vector<uint8_t> compressPointCloud(draco::PointCloud* point_cloud)
    {
        try {
            if (!point_cloud) {
                RCLCPP_ERROR(this->get_logger(), "Point cloud is null");
                return {};
            }
            
            if (point_cloud->num_points() == 0) {
                RCLCPP_ERROR(this->get_logger(), "Point cloud has 0 points");
                return {};
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Compressing point cloud with %d points, %d attributes", 
                        point_cloud->num_points(), point_cloud->num_attributes());
            
            draco::Encoder encoder;
            
            // Set compression options
            encoder.SetSpeedOptions(compression_speed_, compression_speed_);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, quantization_bits_);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 8);
            
            // Encode
            draco::EncoderBuffer buffer;
            draco::Status status = encoder.EncodePointCloudToBuffer(*point_cloud, &buffer);
            if (!status.ok()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to encode point cloud: %s (error code: %d)", 
                           status.error_msg_string().c_str(), status.code());
                return {};
            }
            
            // Copy to vector
            std::vector<uint8_t> compressed_data(buffer.size());
            std::memcpy(compressed_data.data(), buffer.data(), buffer.size());
            
            return compressed_data;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error compressing point cloud: %s", e.what());
            return {};
        }
    }
    
    void sendToClients(const std::vector<uint8_t>& data, 
                       const sensor_msgs::msg::PointCloud2::SharedPtr& original_msg)
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        
        // Create message header with metadata
        // Format: POINT_CLOUD:data_size:frame_id:timestamp_sec:timestamp_nanosec:height:width:point_step:row_step:is_bigendian:is_dense
        std::string header = "POINT_CLOUD:" + 
                            std::to_string(data.size()) + ":" +
                            original_msg->header.frame_id + ":" +
                            std::to_string(original_msg->header.stamp.sec) + ":" +
                            std::to_string(original_msg->header.stamp.nanosec) + ":" +
                            std::to_string(original_msg->height) + ":" +
                            std::to_string(original_msg->width) + ":" +
                            std::to_string(original_msg->point_step) + ":" +
                            std::to_string(original_msg->row_step) + ":" +
                            std::to_string(original_msg->is_bigendian) + ":" +
                            std::to_string(original_msg->is_dense) + "\n";
        
        // Send field information
        std::string fields_header = "FIELDS:" + std::to_string(original_msg->fields.size()) + "\n";
        
        for (auto it = client_sockets_.begin(); it != client_sockets_.end();) {
            int client_socket = *it;
            
            try {
                // Send main header
                ssize_t sent = send(client_socket, header.c_str(), header.size(), MSG_NOSIGNAL);
                if (sent < 0) {
                    RCLCPP_WARN(this->get_logger(), "Failed to send header to client");
                    close(client_socket);
                    it = client_sockets_.erase(it);
                    continue;
                }
                
                // Send fields header
                sent = send(client_socket, fields_header.c_str(), fields_header.size(), MSG_NOSIGNAL);
                if (sent < 0) {
                    RCLCPP_WARN(this->get_logger(), "Failed to send fields header to client");
                    close(client_socket);
                    it = client_sockets_.erase(it);
                    continue;
                }
                
                // Send each field
                for (const auto& field : original_msg->fields) {
                    std::string field_str = "FIELD:" + field.name + ":" +
                                          std::to_string(field.offset) + ":" +
                                          std::to_string(field.datatype) + ":" +
                                          std::to_string(field.count) + "\n";
                    sent = send(client_socket, field_str.c_str(), field_str.size(), MSG_NOSIGNAL);
                    if (sent < 0) {
                        RCLCPP_WARN(this->get_logger(), "Failed to send field to client");
                        close(client_socket);
                        it = client_sockets_.erase(it);
                        break;
                    }
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
    int quantization_bits_;  // Draco quantization bits (default: 11)
    int compression_speed_;  // Draco compression speed 0-10 (default: 5)
    
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

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

class DracoBridgeServer : public rclcpp::Node
{
public:
    DracoBridgeServer() : Node("draco_bridge_server")
    {
        // Parameters
        this->declare_parameter("server_port", 8888);
        this->declare_parameter("input_topic", "/sensing/lidar/top/pointcloud_raw_ex");
        this->declare_parameter("output_topic", "/lidar/compressed");
        this->declare_parameter("decompressed_topic", "/lidar/decompressed");
        
        server_port_ = this->get_parameter("server_port").as_int();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        decompressed_topic_ = this->get_parameter("decompressed_topic").as_string();
        
        // QoS profile
        auto qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        // Publishers and subscribers
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, qos);
        decompressed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            decompressed_topic_, qos);
        
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&DracoBridgeServer::pointCloudCallback, this, std::placeholders::_1));
        
        // Statistics
        message_count_ = 0;
        total_original_size_ = 0;
        total_compressed_size_ = 0;
        
        // Start TCP server
        startServer();
        
        RCLCPP_INFO(this->get_logger(), "Draco Bridge Server started");
        RCLCPP_INFO(this->get_logger(), "Listening on port: %d", server_port_);
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Decompressed topic: %s", decompressed_topic_.c_str());
    }
    
    ~DracoBridgeServer()
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
        
        // Increase buffer sizes
        int buffer_size = 1024 * 1024; // 1MB
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
        server_thread_ = std::thread(&DracoBridgeServer::serverLoop, this);
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
            auto compressed_data = compressPointCloud(draco_point_cloud.get());
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
            
            // Decompress for visualization
            auto decompressed_draco = decompressPointCloud(compressed_data);
            if (decompressed_draco) {
                auto decompressed_msg = convertFromDracoPointCloud(decompressed_draco.get(), msg->header);
                if (decompressed_msg) {
                    decompressed_publisher_->publish(*decompressed_msg);
                }
            }
            
            // Send to TCP clients
            sendToClients(compressed_data, msg);
            
            // Update statistics
            message_count_++;
            total_original_size_ += msg->data.size();
            total_compressed_size_ += compressed_data.size();
            
            if (message_count_ % 100 == 0) {
                double compression_ratio = static_cast<double>(total_original_size_) / total_compressed_size_;
                RCLCPP_INFO(this->get_logger(), 
                           "Processed %zu messages, Compression ratio: %.2f:1", 
                           message_count_.load(), compression_ratio);
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
                return nullptr;
            }
            
            // Create position attribute
            draco::GeometryAttribute pos_att;
            pos_att.Init(draco::GeometryAttribute::POSITION, nullptr, 3, draco::DT_FLOAT32, false, 12, 0);
            int pos_att_id = point_cloud->AddAttribute(pos_att, false, num_points);
            
            // Create intensity attribute
            draco::GeometryAttribute intensity_att;
            intensity_att.Init(draco::GeometryAttribute::GENERIC, nullptr, 1, draco::DT_FLOAT32, false, 4, 0);
            int intensity_att_id = point_cloud->AddAttribute(intensity_att, false, num_points);
            
            // Fill data
            auto pos_att_ptr = point_cloud->attribute(pos_att_id);
            auto intensity_att_ptr = point_cloud->attribute(intensity_att_id);
            
            const uint8_t* data_ptr = msg->data.data();
            int point_step = msg->point_step;
            
            for (int i = 0; i < num_points; ++i) {
                // Position (x, y, z)
                float x = *reinterpret_cast<const float*>(data_ptr + i * point_step + 0);
                float y = *reinterpret_cast<const float*>(data_ptr + i * point_step + 4);
                float z = *reinterpret_cast<const float*>(data_ptr + i * point_step + 8);
                
                pos_att_ptr->SetAttributeValue(draco::AttributeValueIndex(i), 
                                             draco::Vector3f(x, y, z).data());
                
                // Intensity
                float intensity = *reinterpret_cast<const float*>(data_ptr + i * point_step + 12);
                intensity_att_ptr->SetAttributeValue(draco::AttributeValueIndex(i), &intensity);
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
            draco::Encoder encoder;
            
            // Set compression options
            encoder.SetSpeedOptions(5, 5); // Balance between speed and compression
            encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 11);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 8);
            
            // Encode
            draco::EncoderBuffer buffer;
            if (!encoder.EncodePointCloudToBuffer(*point_cloud, &buffer).ok()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to encode point cloud");
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
    
    std::unique_ptr<draco::PointCloud> decompressPointCloud(const std::vector<uint8_t>& compressed_data)
    {
        try {
            draco::Decoder decoder;
            draco::DecoderBuffer buffer;
            
            buffer.Init(reinterpret_cast<const char*>(compressed_data.data()), compressed_data.size());
            
            auto geometry_type = decoder.GetEncodedGeometryType(&buffer);
            if (!geometry_type.ok() || geometry_type.value() != draco::TRIANGULAR_MESH) {
                // Try as point cloud
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
    
    sensor_msgs::msg::PointCloud2::SharedPtr convertFromDracoPointCloud(
        draco::PointCloud* point_cloud, const std_msgs::msg::Header& header)
    {
        try {
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            msg->header = header;
            
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
    
    void sendToClients(const std::vector<uint8_t>& data, const sensor_msgs::msg::PointCloud2::SharedPtr /* msg */)
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
                const size_t chunk_size = 8192;
                
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
    std::string decompressed_topic_;
    
    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr compressed_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr decompressed_publisher_;
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
    
    auto node = std::make_shared<DracoBridgeServer>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <draco/compression/decode.h>
#include <draco/point_cloud/point_cloud.h>
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
                // Read main header
                std::string header = readLine();
                if (header.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Empty header received, disconnecting");
                    break;
                }
                
                // Parse header: POINT_CLOUD:data_size:frame_id:timestamp_sec:timestamp_nanosec:height:width:point_step:row_step:is_bigendian:is_dense
                if (header.find("POINT_CLOUD:") != 0) {
                    RCLCPP_WARN(this->get_logger(), "Invalid header format: %s", header.c_str());
                    continue;
                }
                
                // Split header by ':'
                std::vector<std::string> header_parts;
                size_t pos = 0;
                std::string token;
                while ((pos = header.find(':')) != std::string::npos) {
                    token = header.substr(0, pos);
                    header_parts.push_back(token);
                    header.erase(0, pos + 1);
                }
                header_parts.push_back(header); // Last part
                
                if (header_parts.size() < 11) {
                    RCLCPP_WARN(this->get_logger(), "Invalid header parts count: %zu", header_parts.size());
                    continue;
                }
                
                // Extract metadata
                size_t data_size = std::stoul(header_parts[1]);
                std::string frame_id = header_parts[2];
                int32_t stamp_sec = std::stoi(header_parts[3]);
                uint32_t stamp_nanosec = std::stoul(header_parts[4]);
                uint32_t height = std::stoul(header_parts[5]);
                uint32_t width = std::stoul(header_parts[6]);
                uint32_t point_step = std::stoul(header_parts[7]);
                uint32_t row_step = std::stoul(header_parts[8]);
                bool is_bigendian = std::stoi(header_parts[9]) != 0;
                bool is_dense = std::stoi(header_parts[10]) != 0;
                
                // Read fields header
                std::string fields_header = readLine();
                if (fields_header.find("FIELDS:") != 0) {
                    RCLCPP_WARN(this->get_logger(), "Invalid fields header: %s", fields_header.c_str());
                    continue;
                }
                
                size_t num_fields = std::stoul(fields_header.substr(7));
                std::vector<sensor_msgs::msg::PointField> fields;
                
                // Read each field
                for (size_t i = 0; i < num_fields; i++) {
                    std::string field_line = readLine();
                    if (field_line.find("FIELD:") != 0) {
                        RCLCPP_WARN(this->get_logger(), "Invalid field format: %s", field_line.c_str());
                        break;
                    }
                    
                    // Parse field: FIELD:name:offset:datatype:count
                    std::vector<std::string> field_parts;
                    pos = 0;
                    while ((pos = field_line.find(':')) != std::string::npos) {
                        token = field_line.substr(0, pos);
                        field_parts.push_back(token);
                        field_line.erase(0, pos + 1);
                    }
                    field_parts.push_back(field_line);
                    
                    if (field_parts.size() >= 5) {
                        sensor_msgs::msg::PointField field;
                        field.name = field_parts[1];
                        field.offset = std::stoul(field_parts[2]);
                        field.datatype = std::stoul(field_parts[3]);
                        field.count = std::stoul(field_parts[4]);
                        fields.push_back(field);
                    }
                }
                
                // Read compressed data
                std::vector<uint8_t> compressed_data(data_size);
                if (!readData(compressed_data.data(), data_size)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to read compressed data");
                    break;
                }
                
                // Decompress and publish
                processCompressedData(compressed_data, frame_id, stamp_sec, stamp_nanosec, 
                                    height, width, point_step, row_step, 
                                    is_bigendian, is_dense, fields);
                
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
    
    void processCompressedData(const std::vector<uint8_t>& compressed_data,
                               const std::string& frame_id,
                               int32_t stamp_sec,
                               uint32_t stamp_nanosec,
                               uint32_t height,
                               uint32_t width,
                               uint32_t point_step,
                               uint32_t row_step,
                               bool is_bigendian,
                               bool is_dense,
                               const std::vector<sensor_msgs::msg::PointField>& fields)
    {
        try {
            // Decompress with Draco
            auto draco_point_cloud = decompressPointCloud(compressed_data);
            if (!draco_point_cloud) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decompress point cloud");
                return;
            }
            
            // Convert Draco PointCloud to ROS PointCloud2
            auto msg = convertFromDracoPointCloud(draco_point_cloud.get(), frame_id, 
                                                  stamp_sec, stamp_nanosec);
            if (!msg) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert from Draco point cloud");
                return;
            }
            
            // Publish
            decompressed_publisher_->publish(*msg);
            
            // Update statistics
            message_count_++;
            total_bytes_received_ += compressed_data.size();
            
            if (message_count_ % 10 == 0) {
                size_t decompressed_size = msg->data.size();
                double compression_ratio = static_cast<double>(decompressed_size) / compressed_data.size();
                RCLCPP_INFO(this->get_logger(), 
                           "=== TCP/IP 수신 통계 (Draco) ===");
                RCLCPP_INFO(this->get_logger(), 
                           "수신된 메시지: %zu개", message_count_.load());
                RCLCPP_INFO(this->get_logger(), 
                           "Frame ID: %s", frame_id.c_str());
                RCLCPP_INFO(this->get_logger(), 
                           "총 수신 바이트: %.2f MB", total_bytes_received_ / (1024.0 * 1024.0));
                RCLCPP_INFO(this->get_logger(), 
                           "압축률: %.2f:1", compression_ratio);
                RCLCPP_INFO(this->get_logger(), 
                           "현재 메시지: %zu bytes -> %zu bytes", 
                           compressed_data.size(), decompressed_size);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing compressed data: %s", e.what());
        }
    }
    
    std::unique_ptr<draco::PointCloud> decompressPointCloud(const std::vector<uint8_t>& compressed_data)
    {
        try {
            draco::DecoderBuffer buffer;
            buffer.Init(reinterpret_cast<const char*>(compressed_data.data()), compressed_data.size());
            
            draco::Decoder decoder;
            auto geom_type = draco::Decoder::GetEncodedGeometryType(&buffer).value();
            
            if (geom_type != draco::POINT_CLOUD) {
                RCLCPP_ERROR(this->get_logger(), "Invalid geometry type");
                return nullptr;
            }
            
            auto status_or = decoder.DecodePointCloudFromBuffer(&buffer);
            if (!status_or.ok()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode point cloud: %s", 
                           status_or.status().error_msg());
                return nullptr;
            }
            
            return std::move(status_or).value();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error decompressing point cloud: %s", e.what());
            return nullptr;
        }
    }
    
    std::shared_ptr<sensor_msgs::msg::PointCloud2> convertFromDracoPointCloud(
        draco::PointCloud* point_cloud,
        const std::string& frame_id,
        int32_t stamp_sec,
        uint32_t stamp_nanosec)
    {
        try {
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            
            msg->header.frame_id = frame_id;
            msg->header.stamp.sec = stamp_sec;
            msg->header.stamp.nanosec = stamp_nanosec;
            
            int num_points = point_cloud->num_points();
            if (num_points == 0) {
                return nullptr;
            }
            
            msg->width = num_points;
            msg->height = 1;
            msg->is_dense = true;
            msg->is_bigendian = false;
            
            // Setup fields
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
            
            msg->point_step = 16;
            msg->row_step = msg->point_step * msg->width;
            msg->data.resize(msg->row_step);
            
            // Get attributes
            const auto* pos_att = point_cloud->GetNamedAttribute(draco::GeometryAttribute::POSITION);
            const auto* intensity_att = point_cloud->GetNamedAttribute(draco::GeometryAttribute::GENERIC);
            
            if (!pos_att) {
                RCLCPP_ERROR(this->get_logger(), "No position attribute found");
                return nullptr;
            }
            
            // Fill data
            uint8_t* data_ptr = msg->data.data();
            for (int i = 0; i < num_points; ++i) {
                // Position
                std::array<float, 3> pos;
                pos_att->GetValue(draco::AttributeValueIndex(i), pos.data());
                
                *reinterpret_cast<float*>(data_ptr + i * 16 + 0) = pos[0];
                *reinterpret_cast<float*>(data_ptr + i * 16 + 4) = pos[1];
                *reinterpret_cast<float*>(data_ptr + i * 16 + 8) = pos[2];
                
                // Intensity
                if (intensity_att) {
                    float intensity;
                    intensity_att->GetValue(draco::AttributeValueIndex(i), &intensity);
                    *reinterpret_cast<float*>(data_ptr + i * 16 + 12) = intensity;
                } else {
                    *reinterpret_cast<float*>(data_ptr + i * 16 + 12) = 0.0f;
                }
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
    
    auto node = std::make_shared<SimpleDracoClient>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

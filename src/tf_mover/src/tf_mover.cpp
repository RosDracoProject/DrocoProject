#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

class TfMover : public rclcpp::Node
{
public:
    TfMover() : Node("tf_mover")
    {
        // Create transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // Timer for publishing transforms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&TfMover::publishTransform, this));
        
        // Parameters
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("radius", 2.0);
        this->declare_parameter("speed", 0.5);
        
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        radius_ = this->get_parameter("radius").as_double();
        speed_ = this->get_parameter("speed").as_double();
        
        RCLCPP_INFO(this->get_logger(), "TF Mover started");
        RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Map frame: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Radius: %.2f", radius_);
        RCLCPP_INFO(this->get_logger(), "Speed: %.2f", speed_);
    }

private:
    void publishTransform()
    {
        auto transform = geometry_msgs::msg::TransformStamped();
        
        // Set header
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = base_frame_;
        
        // Calculate circular motion
        double time = this->get_clock()->now().seconds();
        double angle = speed_ * time;
        
        // Circular path
        double x = radius_ * std::cos(angle);
        double y = radius_ * std::sin(angle);
        double z = 0.0;
        
        // Set translation
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        
        // Set rotation (robot always faces forward)
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = std::sin(angle / 2.0);
        transform.transform.rotation.w = std::cos(angle / 2.0);
        
        // Broadcast transform
        tf_broadcaster_->sendTransform(transform);
        
        // Log position every 2 seconds
        static auto last_log = this->get_clock()->now();
        if ((this->get_clock()->now() - last_log).seconds() > 2.0) {
            RCLCPP_INFO(this->get_logger(), 
                       "Robot position: (%.2f, %.2f, %.2f), angle: %.2f", 
                       x, y, z, angle);
            last_log = this->get_clock()->now();
        }
    }
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string base_frame_;
    std::string map_frame_;
    double radius_;
    double speed_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfMover>());
    rclcpp::shutdown();
    return 0;
}

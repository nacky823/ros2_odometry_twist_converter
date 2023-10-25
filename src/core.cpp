#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "geometry_msgs/msg/TwistWithCovarianceStamped.hpp"

class OdometryToTwistConverterNode : public rclcpp::Node
{
public:
    OdometryToTwistConverterNode() : Node("odometry_to_twist_converter") {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry_topic", 10, std::bind(&OdometryToTwistConverterNode::odometryCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "twist_with_covariance_stamped_topic", 10);
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = msg->header;
        
        // Convert Odometry data to TwistWithCovarianceStamped
        twist_msg.twist.twist = msg->twist.twist;
        
        publisher_->publish(twist_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryToTwistConverterNode>());
    rclcpp::shutdown();
    return 0;
}

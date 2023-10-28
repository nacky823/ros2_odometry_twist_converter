#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

class OdometryTwistConverter : public rclcpp::Node
{
public:
  OdometryTwistConverter()
  : Node("odometry_twist_converter")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "mugimaru_odom", 10, std::bind(&OdometryTwistConverter::callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("mugimaru_twist", 10);
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header = msg->header;
    twist_msg.header.frame_id = "base_link";
    twist_msg.twist.twist.linear = msg->twist.twist.linear;
    twist_msg.twist.twist.angular = msg->twist.twist.angular;
    publisher_->publish(twist_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryTwistConverter>());
  rclcpp::shutdown();
  return 0;
}

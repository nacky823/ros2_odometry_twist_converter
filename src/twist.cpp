#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("twist_to_twist_with_cov")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MinimalSubscriber::listener_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/mugimaru_twist", 10);
  }

private:
  void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    auto twist_with_covariance_stamped = geometry_msgs::msg::TwistWithCovarianceStamped();
    twist_with_covariance_stamped.header.frame_id = "base_link";
    twist_with_covariance_stamped.twist.twist = *msg;
    publisher_->publish(twist_with_covariance_stamped);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

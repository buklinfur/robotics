#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CircleMovement : public rclcpp::Node
{
public:
  CircleMovement()
  : Node("circle_movement")
  {
    this->declare_parameter<double>("linear_x", 0.4);
    this->declare_parameter<double>("angular_z", 0.6);
    this->declare_parameter<double>("rate_hz", 20.0);

    linear_x_  = this->get_parameter("linear_x").as_double();
    angular_z_ = this->get_parameter("angular_z").as_double();
    rate_hz_   = this->get_parameter("rate_hz").as_double();
    if (rate_hz_ <= 0.0) rate_hz_ = 20.0;

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CircleMovement::onTimer, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing /cmd_vel: linear.x=%.3f m/s, angular.z=%.3f rad/s (%.1f Hz)",
      linear_x_, angular_z_, rate_hz_
    );
  }

  ~CircleMovement() override
  {
    // стоп при выходе
    geometry_msgs::msg::Twist msg;
    pub_->publish(msg);
  }

private:
  void onTimer()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x_;
    msg.angular.z = angular_z_;
    pub_->publish(msg);
  }

  double linear_x_{0.4};
  double angular_z_{0.6};
  double rate_hz_{20.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CircleMovement>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

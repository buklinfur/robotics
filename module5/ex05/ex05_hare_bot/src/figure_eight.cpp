#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/*
  Figure-eight pattern via differential drive:
  We alternate the sign of angular velocity every half-period.
  That produces two circles with opposite curvature -> "8".

  Parameters:
    linear_x: forward speed (m/s)
    angular_z: magnitude of yaw rate (rad/s)
    segment_sec: duration of each lobe (seconds)  (one circle-ish)
    rate_hz: publish rate
*/

class FigureEight : public rclcpp::Node
{
public:
  FigureEight() : Node("figure_eight")
  {
    this->declare_parameter<double>("linear_x", 0.35);
    this->declare_parameter<double>("angular_z", 0.7);
    this->declare_parameter<double>("segment_sec", 6.0);
    this->declare_parameter<double>("rate_hz", 20.0);

    linear_x_ = this->get_parameter("linear_x").as_double();
    angular_z_mag_ = std::fabs(this->get_parameter("angular_z").as_double());
    segment_sec_ = this->get_parameter("segment_sec").as_double();
    rate_hz_ = this->get_parameter("rate_hz").as_double();

    if (rate_hz_ <= 0.0) rate_hz_ = 20.0;
    if (segment_sec_ <= 0.1) segment_sec_ = 6.0;
    if (angular_z_mag_ < 1e-3) angular_z_mag_ = 0.7;

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    start_time_ = this->now();
    lobe_sign_ = +1;

    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FigureEight::on_timer, this)
    );

    RCLCPP_INFO(get_logger(),
      "Figure-8 started: linear_x=%.3f, angular_z=±%.3f, segment_sec=%.2f, rate=%.1f",
      linear_x_, angular_z_mag_, segment_sec_, rate_hz_);  }

  ~FigureEight() override
  {
    stop();
  }

private:
  void stop()
  {
    geometry_msgs::msg::Twist msg;
    pub_->publish(msg);
  }

  void on_timer()
  {
    // Switch direction every segment_sec_ seconds
    const double t = (this->now() - start_time_).seconds();
    const int segment_idx = static_cast<int>(std::floor(t / segment_sec_));
    lobe_sign_ = (segment_idx % 2 == 0) ? +1 : -1;

    geometry_msgs::msg::Twist msg;
    msg.linear.x  = linear_x_;
    msg.angular.z = lobe_sign_ * angular_z_mag_;
    pub_->publish(msg);
  }

  double linear_x_{0.35};
  double angular_z_mag_{0.7};
  double segment_sec_{6.0};
  double rate_hz_{20.0};

  int lobe_sign_{+1};
  rclcpp::Time start_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FigureEight>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

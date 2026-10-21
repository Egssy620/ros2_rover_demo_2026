#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class AutoCmdVelNode : public rclcpp::Node {
public:
  AutoCmdVelNode()
  : Node("auto_cmd_vel_node"), phase_(0), timer_count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AutoCmdVelNode::control_loop, this)
    );
    RCLCPP_INFO(this->get_logger(), "Auto movement node started.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int phase_;         // 0: 斜め移動, 1: stop
  int timer_count_;   // 100msごとにインクリメント
  double speed_ = 0.6; // 移動速度（合成速度）
  double angle_deg_ = 35.0; // 進行方向（度）
  double distance_ = 50.0; // 斜め移動距離(30m)

  void control_loop() {
    geometry_msgs::msg::Twist msg;

    switch (phase_) {
      case 0: {  // 斜め移動（姿勢維持）
        const double angle_rad = angle_deg_ * M_PI / 180.0;
        msg.linear.x = speed_ * std::cos(angle_rad);
        msg.linear.y = speed_ * std::sin(angle_rad);
        msg.angular.z = 0.0;

        const double travel_time_sec = (speed_ > 0.0) ? (distance_ / speed_) : 0.0;
        if (++timer_count_ >= static_cast<int>(travel_time_sec / 0.1)) {
          phase_ = 1;
          timer_count_ = 0;
        }
        break;
      }

      case 1:  // 停止
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.angular.z = 0.0;
        RCLCPP_INFO_ONCE(this->get_logger(), "Movement complete.");
        break;
    }

    publisher_->publish(msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoCmdVelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

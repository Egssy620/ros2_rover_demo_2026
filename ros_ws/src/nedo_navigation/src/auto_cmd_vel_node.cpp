#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
  int phase_;         // 0: y+, 1: x+, 2: x-, 3: stop
  int timer_count_;   // 100msごとにインクリメント

  void control_loop() {
    geometry_msgs::msg::Twist msg;

    switch (phase_) {
      case 0:  // +y方向に5m前進
        msg.linear.y = 0.2;
        if (++timer_count_ >= int(5.0 / 0.5 / 0.1)) {
          phase_ = 1;
          timer_count_ = 0;
        }
        break;

      case 1:  // -y方向に2.5m移動
        msg.linear.y = -0.2;
        if (++timer_count_ >= int(2.5 / 0.5 / 0.1)) {
          phase_ = 2;
          timer_count_ = 0;
        }
        break;

      case 2:  // 停止
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

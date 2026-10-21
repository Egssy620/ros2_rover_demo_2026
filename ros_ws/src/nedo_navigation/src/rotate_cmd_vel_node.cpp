#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RotateCmdVelNode : public rclcpp::Node {
public:
  RotateCmdVelNode()
  : Node("rotate_cmd_vel_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RotateCmdVelNode::control_loop, this)
    );
    RCLCPP_INFO(this->get_logger(), "Rotate movement node started. Rotating at 0.1 rad/s.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void control_loop() {
    geometry_msgs::msg::Twist msg;
    
    // 0.1rad/sで回転
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.06;

    publisher_->publish(msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RotateCmdVelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

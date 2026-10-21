#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

class ManualCmdVelNode : public rclcpp::Node {
public:
  ManualCmdVelNode() : Node("manual_cmd_vel_node") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Manual cmd_vel publisher ready!");
    run();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void run() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char c;
    geometry_msgs::msg::Twist msg;

    std::cout << "Use keys: w/a/s/d/q/e (wasd = move, q/e = rotate)" << std::endl;

    while (rclcpp::ok()) {
      c = getchar();

      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.angular.z = 0.0;

      switch (c) {
        case 'w': msg.linear.x = 0.9; break;
        case 's': msg.linear.x = -0.9; break;
        case 'a': msg.linear.y = 0.9; break;
        case 'd': msg.linear.y = -0.9; break;
        case 'q': msg.angular.z = 0.9; break;
        case 'e': msg.angular.z = -0.9; break;
        case 'x': std::cout << "Stop\n"; break;
        default: continue;
      }

      publisher_->publish(msg);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualCmdVelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

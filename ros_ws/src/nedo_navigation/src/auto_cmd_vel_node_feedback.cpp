#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class AutoCmdVelNode : public rclcpp::Node {
public:
  AutoCmdVelNode()
  : Node("auto_cmd_vel_node"), phase_(0), odom_received_(false)
  {
    // Publisher (速度指令)
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscriber (オドメトリ受信)
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AutoCmdVelNode::odom_callback, this, std::placeholders::_1)
    );

    // 制御ループ (100ms周期)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AutoCmdVelNode::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Auto movement node (Closed Loop) started.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  int phase_;            // 0: y+, 1: y-, 2: stop
  bool odom_received_;   // オドメトリを一度でも受信したか
  float velocity_ = 0.35; // 移動速度
  float distance_ = 20;      // 移動距離(2.5m) considering odometry bias

  // 位置情報管理用
  double current_x_, current_y_; // 現在のオドメトリ位置
  double start_x_, start_y_;     // 現在のフェーズを開始したときの位置

  // オドメトリのコールバック関数
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // 最初の1回だけ、開始位置を初期化する
    if (!odom_received_) {
      reset_start_pose();
      odom_received_ = true;
    }
  }

  // フェーズ切り替え時に基準位置をリセットする関数
  void reset_start_pose() {
    start_x_ = current_x_;
    start_y_ = current_y_;
    RCLCPP_INFO(this->get_logger(), "Phase changed. Reset start pose to (%.2f, %.2f)", start_x_, start_y_);
  }

  // 現在のフェーズでの移動距離を計算
  double get_distance_traveled() {
    return std::hypot(current_x_ - start_x_, current_y_ - start_y_);
  }

  void control_loop() {
    // オドメトリを受け取るまでは何もしない
    if (!odom_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for /odom...");
      return;
    }

    geometry_msgs::msg::Twist msg;
    double dist = get_distance_traveled();

    switch (phase_) {
      case 0:  // +y方向に2.5m移動
        msg.linear.x = velocity_;
        // 距離判定 (2.5m)
        if (dist >= distance_) {
          phase_ = 1;
          reset_start_pose();
        }
        break;

      case 1:  // -y方向に2.5m移動
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.angular.z = 0.0;
        break;

      case 2:  // 停止
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.angular.z = 0.0;
        RCLCPP_INFO_ONCE(this->get_logger(), "Movement complete.");
        break;
    }

    // 停止フェーズ以外ではログを出す（デバッグ用）
    if (phase_ < 2) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Phase: %d, Distance: %.2f / Target", phase_, dist);
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
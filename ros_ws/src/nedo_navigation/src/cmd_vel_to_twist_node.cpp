// cmd_vel_to_twist_node.cpp
//
// 目的
// - 車輪オドメトリが無い構成で、LM が 2〜3 秒途切れた間のデッドレコニング源として
//   指令速度 /cmd_vel (geometry_msgs/Twist) を「疑似速度計測」として EKF に渡す。
// - robot_localization は生の Twist を購読できない (twistN 入力は
//   geometry_msgs/TwistWithCovarianceStamped が必要) ため、ここで型変換し
//   共分散とヘッダ (stamp, frame_id) を付与する。
//
// 注意
// - これは「指令値」であって実測ではない。スリップ・応答遅れ・加減速の追従誤差が
//   そのまま乗るので、共分散は車輪/LIO オドメより緩めに設定する (信頼しすぎない)。
// - stamp は受信時の now() を使う (/cmd_vel は通常スタンプ無しの Twist のため)。
// - frame_id は base_link (本体座標系での前進/旋回速度)。
//
// パラメータ
//   input_topic        : 購読する Twist トピック (default /cmd_vel)
//   output_topic       : 出力する TwistWithCovarianceStamped (default /cmd_vel_cov)
//   frame_id           : 出力ヘッダの frame (default base_link)
//   linear_x_variance  : vx の分散 [(m/s)^2]
//   linear_y_variance  : vy の分散 [(m/s)^2] (差動なら実質 0 拘束に使える)
//   angular_z_variance : vyaw の分散 [(rad/s)^2]

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

class CmdVelToTwistNode : public rclcpp::Node
{
public:
  CmdVelToTwistNode() : Node("cmd_vel_to_twist_node")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/cmd_vel_cov");
    frame_id_     = this->declare_parameter<std::string>("frame_id", "base_link");
    // 指令値ベースなので緩め (σ=0.2 相当)。実機の追従誤差を見て調整。
    linear_x_variance_  = this->declare_parameter<double>("linear_x_variance", 0.04);   // [(m/s)^2]
    linear_y_variance_  = this->declare_parameter<double>("linear_y_variance", 0.04);   // [(m/s)^2]
    angular_z_variance_ = this->declare_parameter<double>("angular_z_variance", 0.04);  // [(rad/s)^2]

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      output_topic_, rclcpp::SystemDefaultsQoS());

    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&CmdVelToTwistNode::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "cmd_vel_to_twist: %s -> %s (frame=%s, var vx=%.4f vy=%.4f vyaw=%.4f)",
                input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
                linear_x_variance_, linear_y_variance_, angular_z_variance_);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped out;
    out.header.stamp = this->now();
    out.header.frame_id = frame_id_;
    out.twist.twist = *msg;

    // 6x6 (vx,vy,vz,vroll,vpitch,vyaw)。対角のみ設定。
    std::fill(out.twist.covariance.begin(), out.twist.covariance.end(), 0.0);
    out.twist.covariance[0]  = linear_x_variance_;   // vx
    out.twist.covariance[7]  = linear_y_variance_;   // vy
    out.twist.covariance[35] = angular_z_variance_;  // vyaw
    // 使わない vz/vroll/vpitch は EKF 側 config で false にするため値は無視される。

    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double linear_x_variance_;
  double linear_y_variance_;
  double angular_z_variance_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToTwistNode>());
  rclcpp::shutdown();
  return 0;
}

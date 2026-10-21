// pose_converter.cpp
//
// LM 自己位置 (localizer_node の /robot_pose, geometry_msgs/PoseStamped) を
// robot_localization (EKF) が購読できる geometry_msgs/PoseWithCovarianceStamped
// に変換し、対角共分散を付与して再 publish する。
//
// 共分散は「固定値」をパラメータでそのまま対角に入れるだけの静的方式。
// 計測の実品質 (ランドマーク数/配置/fit残差) は反映しないので、値は検出器の
// 実精度を見て手調整する。
//   position_variance     : x, y, z の分散 [m^2]
//   orientation_variance  : roll, pitch, yaw の分散 [rad^2]
// 平面走行 (two_d_mode) では z/roll/pitch は EKF 側で 0 に拘束されるため、
// 実質的に効くのは x, y, yaw の分散。
//
// 入出力トピックもパラメータで差し替え可能。

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class PoseToCovarianceNode : public rclcpp::Node
{
public:
  PoseToCovarianceNode() : Node("pose_to_covariance_node")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "robot_pose");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "robot_pose_cov");
    // 検出器精度に合わせて調整するノブ。LM を信頼させたいほど小さく。
    position_variance_    = this->declare_parameter<double>("position_variance", 0.02);    // [m^2]
    orientation_variance_ = this->declare_parameter<double>("orientation_variance", 0.02); // [rad^2]

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, rclcpp::SystemDefaultsQoS());

    subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&PoseToCovarianceNode::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "pose_to_covariance: %s -> %s (pos_var=%.4f, ori_var=%.4f)",
                input_topic_.c_str(), output_topic_.c_str(),
                position_variance_, orientation_variance_);
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped cov_msg;

    // localizer_node は header.stamp=now(), frame_id=map で publish 済みなので
    // ヘッダはそのまま流用する (EKF の時系列順序付けのため再スタンプしない)。
    cov_msg.header = msg->header;
    cov_msg.pose.pose = msg->pose;

    // 6x6 を 36 要素一次元で表現。対角のみ設定し残りは 0。
    std::fill(cov_msg.pose.covariance.begin(), cov_msg.pose.covariance.end(), 0.0);
    cov_msg.pose.covariance[0]  = position_variance_;    // x
    cov_msg.pose.covariance[7]  = position_variance_;    // y
    cov_msg.pose.covariance[14] = position_variance_;    // z
    cov_msg.pose.covariance[21] = orientation_variance_; // roll
    cov_msg.pose.covariance[28] = orientation_variance_; // pitch
    cov_msg.pose.covariance[35] = orientation_variance_; // yaw

    publisher_->publish(cov_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  double position_variance_;
  double orientation_variance_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToCovarianceNode>());
  rclcpp::shutdown();
  return 0;
}

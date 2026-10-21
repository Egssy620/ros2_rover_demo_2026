// imu_restamp_node.cpp
//
// 目的
// - Livox 内蔵 IMU (/livox/imu_<serial>) は enable_timesync:=false のため
//   header.stamp が「デバイス時計 (起動からの経過時間)」で、ホスト ROS 時刻
//   (epoch) とはクロック領域が異なる。
// - robot_localization (EKF) の他入力 (/odom, /robot_pose_cov, /cmd_vel_cov) は
//   ホスト ROS 時刻なので、生 IMU をそのまま渡すと ~10億秒ずれて EKF が IMU を
//   棄却 (or フィルタ時刻破壊) する。
// - 本ノードは IMU を受信時の now() で再スタンプして別トピックへ流し、EKF 入力の
//   クロック領域をホスト時刻に揃える (mapping の odom_imu_fuser と同趣旨)。
//
// 注意
// - これは「定常オフセット (領域違い)」の解消であり、転送/スケジューリング由来の
//   ジッタは消えない。低速走行の EKF 用途では十分 (PTP が要るのは LIO デスキュー
//   や高速走行の場合)。
// - EKF はヨーレート (angular_velocity.z) のみ使うため、angular_velocity と
//   linear_acceleration はそのまま透過する。
//   ※ Livox の linear_acceleration は g 単位なので、もし将来 EKF で加速度を
//      使うなら別途 g→m/s² 変換が必要 (本ノードでは変換しない)。
// - Livox ドライバは orientation を {0,0,0,0} で出すことがあるため、不正な
//   クォータニオンを下流に流さないよう identity + covariance[0]=-1 (姿勢不明)
//   に正規化する (EKF は config で絶対姿勢を使わないので実害はないが防御的に)。
//
// パラメータ
//   input_topic  : 購読する生 IMU トピック (default /livox/imu)
//   output_topic : 再スタンプ後の出力トピック (default /livox/imu_restamped)

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuRestampNode : public rclcpp::Node
{
public:
  ImuRestampNode() : Node("imu_restamp_node")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/livox/imu");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/imu_restamped");

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
      output_topic_, rclcpp::SensorDataQoS());

    subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuRestampNode::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "imu_restamp: %s -> %s (stamp を now() に打ち直し)",
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;

    // 唯一の本質的な変更: デバイス時計 → ホスト ROS 時刻 に再スタンプ。
    out.header.stamp = this->now();

    // 不正クォータニオン ({0,0,0,0} 等) を防御的に正規化。
    // EKF は絶対姿勢を使わない (covariance[0]=-1 で「姿勢不明」を明示)。
    const auto & q = out.orientation;
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < 0.1) {
      out.orientation.w = 1.0;
      out.orientation.x = 0.0;
      out.orientation.y = 0.0;
      out.orientation.z = 0.0;
      out.orientation_covariance[0] = -1.0;
    }

    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRestampNode>());
  rclcpp::shutdown();
  return 0;
}

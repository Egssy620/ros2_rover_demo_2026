#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class LidarTfPublisherNode : public rclcpp::Node
{
public:
    LidarTfPublisherNode() : Node("lidar_tf_publisher_node")
    {
        // Static TF broadcasterを作成
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // QT128の設置位置を考慮したTF変換を設定
        //
        // フレーム名はパラメータ化してある。QT128/OT128 を同時運用する際に
        // 両者が同じ frame_id を名乗るとTFが衝突するため、ドライバ側の
        // ros_frame_id (config_hesai_qt128.yaml=hesai_qt128 /
        // config_hesai_ot128.yaml=hesai_ot128) と必ず一致させること。
        const std::string parent_frame =
            this->declare_parameter<std::string>("parent_frame", "base_link");
        const std::string child_frame =
            this->declare_parameter<std::string>("child_frame", "hesai_qt128");

        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = parent_frame; // ロボット中心
        transform_stamped.child_frame_id = child_frame;   // Hesaiドライバの ros_frame_id に合わせる

        // QT128の設置位置 (base_link → QT128)
        //   z / roll / pitch は 2026-09-24 実測 (bag 20260924 ②③④ の床平面フィット)
        //     床からセンサまで 1.939 m → base_link (床+0.3 m) から 1.639 m
        //     roll 50.37° / pitch 5.06° (旧値 roll 45° / pitch 0° から床が 7.4° 傾いて見えていた)
        //   yaw: 機体は壁と平行に設置 → bag 4本の壁方向 (旧yaw 180° で +7.0〜7.1°) を打ち消して 172.95°
        //   x / y は床からは決まらないので旧値のまま (機体の左上: 前方0.36m、左0.25m)
        const double x = this->declare_parameter<double>("x", 0.36);
        const double y = this->declare_parameter<double>("y", 0.25);
        const double z = this->declare_parameter<double>("z", 1.639);
        const double roll = this->declare_parameter<double>("roll", 0.8792);
        const double pitch = this->declare_parameter<double>("pitch", 0.0883);
        const double yaw = this->declare_parameter<double>("yaw", 3.0185);

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // Static TFを送信
        tf_broadcaster_->sendTransform(transform_stamped);

        RCLCPP_INFO(this->get_logger(), "LiDAR TF published: %s -> %s",
                    parent_frame.c_str(), child_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "  Position: x=%.3f, y=%.3f, z=%.3f", x, y, z);
        RCLCPP_INFO(this->get_logger(), "  Rotation: roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg",
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarTfPublisherNode>();

    RCLCPP_INFO(node->get_logger(), "LiDAR TF Publisher Node started");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

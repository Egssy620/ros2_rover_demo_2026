#ifndef NEDO_LOCALIZATION__LOCALIZER_HPP
#define NEDO_LOCALIZATION__LOCALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vehicle_interface/msg/pose_stamped_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
namespace nedo_localization
{
    class Localizer : public rclcpp::Node
    {
    public:
        explicit Localizer(const rclcpp::NodeOptions &options);

    private:
        void marker_pose_callback(const vehicle_interface::msg::PoseStampedArray::SharedPtr marker_poses);
        void load_map();
        void subscribe_map_topic();
        void handle_map_msg(const vehicle_interface::msg::PoseStampedArray::SharedPtr msg);
        void localization();
        void localize_by_svd(geometry_msgs::msg::PoseStamped &robot_pose);
        void lpf_pose(geometry_msgs::msg::PoseStamped &current_pose);
        void best_fit_transform(
            const std::vector<Eigen::Vector2d> &A,
            const std::vector<Eigen::Vector2d> &B,
            geometry_msgs::msg::PoseStamped &return_transform);

        bool calculate_success_;
        bool has_previous_pose_;
        // map->base_link の動的TFを localizer 自身が配信するか (default true)。
        // EKF(robot_localization) を併用する構成では EKF が TF を所有するため
        // false にして二重親 (map/odom 両方が base_link の親) を避ける。
        bool publish_tf_;
        double lpf_rate_;
        std::string map_frame_;
        geometry_msgs::msg::PoseStamped previous_pose_;

        rclcpp::Subscription<vehicle_interface::msg::PoseStampedArray>::SharedPtr marker_suber_;
        rclcpp::Subscription<vehicle_interface::msg::PoseStampedArray>::SharedPtr map_suber_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_puber_;

        tf2_ros::TransformBroadcaster tf_broadcaster_;

        // 現在のフレームで検出されたマーカー (id -> base_link系pose)。
        // callback先頭でclearするので過去フレームの蓄積はしない。
        std::map<std::string, geometry_msgs::msg::PoseStamped> current_detections_;

        // 地図上のランドマーク真値 (id -> map系pose)。yamlロード/topic更新の両対応。
        std::map<std::string, geometry_msgs::msg::PoseStamped> world_map_;
    };
}

#endif // NEDO_LOCALIZATION__LOCALIZER_HPP
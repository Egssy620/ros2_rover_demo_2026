#ifndef NEDO_LOCALIZATION__LIDAR_CAM_TF_NODE_HPP_
#define NEDO_LOCALIZATION__LIDAR_CAM_TF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
namespace nedo_localization{
    class LidarCamTF : public rclcpp::Node
    {
        public:
            explicit LidarCamTF(const rclcpp::NodeOptions & options);
        private:
        void load_transform_from_json();
        void publish_tf();
        rclcpp::TimerBase::SharedPtr timer_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        geometry_msgs::msg::TransformStamped transform_;
    };
}

#endif //NEDO_LOCALIZATION__LIDAR_CAM_TF_NODE_HPP_
#ifndef NEDO_LOCALIZATION__MATCHER_NODE_HPP_
#define NEDO_LOCALIZATION__MATCHER_NODE_HPP_

#include "nedo_localization/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vehicle_interface/msg/marker_point_with_id.hpp>
#include <vehicle_interface/msg/projected_points.hpp>
namespace nedo_localization{

    class PointMatcherNode :public rclcpp::Node
    {
        public:
            explicit PointMatcherNode(const rclcpp::NodeOptions & options);
        
        private:
            void projection_callback(const vehicle_interface::msg::ProjectedPoints::SharedPtr projected_points);
            void detection_callback(const vehicle_interface::msg::MarkerPointWithId::SharedPtr marker_point);
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_puber_;
            rclcpp::Subscription<vehicle_interface::msg::MarkerPointWithId>::SharedPtr marker_point_suber_;
            rclcpp::Subscription<vehicle_interface::msg::ProjectedPoints>::SharedPtr projected_points_suber_;
    };

} //nedo_localization

#endif //NEDO_LOCALIZATION__MATCHER_NODE_HPP_
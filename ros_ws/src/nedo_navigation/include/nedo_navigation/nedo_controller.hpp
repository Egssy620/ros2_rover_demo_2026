#ifndef NEDO_NAVIGATION__NEDO_CONTROLLER_HPP_
#define NEDO_NAVIGATION__NEDO_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace nedo_navigation{
    class NedoController : public rclcpp::Node
    {
        public:
        NedoController();
        private:
        double distance_thresh_;
        double angle_thresh_;
        double scale_factor_;
        double max_linear_vel_;
        double max_angular_vel_;
        double duration_tolerance_;
        void timer_callback();
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void calculate_rotation(geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff);
        void calculate_translation(double angle_to_target, geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff, double robot_yaw);
        void calculate_steering(double angle_to_target, geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff, double robot_yaw);
        bool pose_arrive_recent_;
        bool is_goal_set_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_puber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_suber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_suber_;
        rclcpp::Time last_pose_arrive_;
        rclcpp::Duration duration_;
        geometry_msgs::msg::PoseStamped goal_pose_;
    };

} // nedo_navigation

#endif // NEDO_NAVIGATION__NEDO_CONTROLLER_HPP_
#include "nedo_navigation/nedo_controller.hpp"

namespace nedo_navigation{

    NedoController::NedoController():Node("nedo_controller"),
    duration_(rclcpp::Duration::from_seconds(10))
    {
        //create timer callback
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(500),
            std::bind(&NedoController::timer_callback,this)
        );
        //create publisher
        cmd_puber_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::SystemDefaultsQoS());
        //create subscriber
        pose_suber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose",rclcpp::SensorDataQoS(),
            std::bind(&NedoController::pose_callback,this,std::placeholders::_1));
        goal_suber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",rclcpp::SensorDataQoS(),
            std::bind(&NedoController::goal_callback,this,std::placeholders::_1));
        last_pose_arrive_ = rclcpp::Time(0,0,RCL_ROS_TIME);
        duration_ = this->now() - last_pose_arrive_;
        pose_arrive_recent_ = false;
        is_goal_set_ = false;
        distance_thresh_ = this->declare_parameter<double>("distance_thresh",0.5);
        double default_angle_thresh_ = 1.0*M_PI / 180.0;
        angle_thresh_ = this->declare_parameter<double>("angle_thresh",default_angle_thresh_);
        scale_factor_ = this->declare_parameter<double>("scale_factor",0.05);
        max_linear_vel_ = this->declare_parameter<double>("max_vel",0.5);
        max_angular_vel_ = this->declare_parameter<double>("max_angle",0.5);
    }

    void NedoController::timer_callback(){
        duration_ = this->now() - last_pose_arrive_;
        if(duration_.seconds()>3.0){
            geometry_msgs::msg::Twist stop_cmd;
            pose_arrive_recent_ = false;
            cmd_puber_->publish(stop_cmd);
        }else{
            pose_arrive_recent_ = true;
        }
    }

    void NedoController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        geometry_msgs::msg::Twist cmd_vel_msg;
        // 現在の時刻を取得
        last_pose_arrive_ = this->now();
        if (pose_arrive_recent_) {
            // 目標が設定されていない場合
            if (!is_goal_set_) {
                //cmd_puber_ -> publish(cmd_vel_msg); for experiment
                return;
            } else {
                // 現在位置・姿勢(x, y, yaw)
                double robot_x = msg->pose.position.x;
                double robot_y = msg->pose.position.y;
                double robot_yaw = std::atan2(2.0 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y),
                                            1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y + msg->pose.orientation.z * msg->pose.orientation.z));
                double target_x_ = goal_pose_.pose.position.x;
                double target_y_ = goal_pose_.pose.position.y;
                double target_theta_ = std::atan2(2.0 * (goal_pose_.pose.orientation.w * goal_pose_.pose.orientation.z + goal_pose_.pose.orientation.x * goal_pose_.pose.orientation.y),
                                               1.0 - 2.0 * (goal_pose_.pose.orientation.y * goal_pose_.pose.orientation.y + goal_pose_.pose.orientation.z * goal_pose_.pose.orientation.z));
                // 目標位置方向への角度
                double angle_to_target = std::atan2(target_y_ - robot_y, target_x_ - robot_x);

                // 目標位置までの距離
                double distance = std::hypot(target_x_ - robot_x, target_y_ - robot_y);

                // 目標姿勢までの角度差
                double anglediff = target_theta_ - robot_yaw;
                // [-π, π]の範囲に正規化
                while (anglediff > M_PI) anglediff -= 2.0 * M_PI;
                while (anglediff < -M_PI) anglediff += 2.0 * M_PI;

                // 到達判定・速度司令
                if (distance < distance_thresh_ && std::abs(anglediff) < angle_thresh_) {
                    cmd_puber_->publish(cmd_vel_msg);
                    RCLCPP_INFO(this->get_logger(), "Reached target point and orientation.");
                } else {
                    if (distance < distance_thresh_ && std::abs(anglediff) >= angle_thresh_) {
                        calculate_rotation(cmd_vel_msg, distance, anglediff);
                    } else if (std::abs(anglediff) < angle_thresh_ && distance >= distance_thresh_) {
                        calculate_translation(angle_to_target, cmd_vel_msg, distance, anglediff, robot_yaw);
                    } else {
                        calculate_steering(angle_to_target, cmd_vel_msg, distance, anglediff, robot_yaw);
                    }
                }
            }
            cmd_puber_ -> publish(cmd_vel_msg);
        } else {
            // cmd_velの発信から一定時間が経過している場合、停止処理
            cmd_puber_ -> publish(cmd_vel_msg);
        }
        RCLCPP_INFO(this->get_logger(), "cmd_vel: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
                    cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);
    }

    void NedoController::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        is_goal_set_ = true;
        goal_pose_ = *msg;
    }

    void NedoController::calculate_rotation(geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff){
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        double A = 1.0;
        cmd_vel_msg.angular.z = std::min(A * anglediff, max_angular_vel_);  // 角速度（最大角速度を制限）
        RCLCPP_INFO(this->get_logger(), "Rotating towards target (distance: %.2f, angle: %.2f)", distance, anglediff);
    }

    void NedoController::calculate_translation(double angle_to_target, geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff, double robot_yaw){
        double V = std::min(std::sqrt(scale_factor_ * distance), max_linear_vel_); // 最大速度を制限
        cmd_vel_msg.linear.x = V * cos(angle_to_target - robot_yaw);   // X方向速度
        cmd_vel_msg.linear.y = V * sin(angle_to_target - robot_yaw);   // Y方向速度
        cmd_vel_msg.angular.z = 0;
        RCLCPP_INFO(this->get_logger(), "Moving towards target (distance: %.2f, angle: %.2f)", distance, anglediff);
    }

    void NedoController::calculate_steering(double angle_to_target, geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff, double robot_yaw){
        double V = std::min(std::sqrt(scale_factor_ * distance), max_linear_vel_); // 最大速度を制限
        cmd_vel_msg.linear.x = V * cos(angle_to_target - robot_yaw);   // X方向速度
        cmd_vel_msg.linear.y = V * sin(angle_to_target - robot_yaw);   // Y方向速度
        double A = 1.0;
        cmd_vel_msg.angular.z = std::min(A * anglediff, max_angular_vel_); // 角速度（最大角速度を制限）
        RCLCPP_INFO(this->get_logger(), "Moving towards target (distance: %.2f, angle: %.2f)", distance, anglediff);     
    }
    
}// namespace nedo_navigation

    int main(int argc, char ** argv){
        rclcpp::init(argc,argv);
        auto node = std::make_shared<nedo_navigation::NedoController>();

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node->get_node_base_interface());
        executor.spin();
        rclcpp::shutdown();

        return 0;
    }
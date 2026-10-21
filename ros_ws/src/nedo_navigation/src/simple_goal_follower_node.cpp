#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <cmath>
#include <string>

// Simple goal follower node (動作確認用の簡易移動ノード):
//   /goal_pose (map 系の目標姿勢) と /robot_pose (map 系の自己位置) から、全方向台車の /cmd_vel を出す。
//   本番の移動は別研究室が担当する。これは pickup_planner_node の動作確認用の差し替え品。
//
//   - 並進: 目標までのずれを機体座標に直した比例制御。速さは min_linear_vel..max_linear_vel
//   - 回転: 位置が合ってから、その場で向きのずれの比例制御。±max_angular_vel で頭打ち
//   - 位置のずれ < pos_tol かつ 向きのずれ < yaw_tol で停止し、目標を消す
//
//   Ranger (ranger_base_node) の /cmd_vel の解釈に合わせている (2026-09-28 実機で収束しなかったため):
//     - linear.y ≠ 0 だと平行移動モードで 舵角 = atan(y/x)、angular.z は無視される
//       → 並進中は angular.z = 0 にし、回転は並進が済んでから (x = y = 0 のときだけスピンモード)
//     - 横移動で x が 0 付近を行き来すると舵角が ±85° で反転し続けて進まない
//       → 小さい方の成分が大きい方の snap_ratio 倍未満なら 0 にする
//   - /robot_pose が pose_timeout より古いと停止 (自己位置が止まったまま動き続けないため)
//   - 止まっている間は /cmd_vel を出さない (手動操作など他の /cmd_vel を邪魔しない)

namespace
{
double normalize_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double yaw_from_quat(const geometry_msgs::msg::Quaternion &q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class SimpleGoalFollowerNode : public rclcpp::Node
{
public:
    SimpleGoalFollowerNode() : Node("simple_goal_follower_node")
    {
        const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
        const auto pose_topic = declare_parameter<std::string>("robot_pose_topic", "/robot_pose");
        const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        kp_lin_ = declare_parameter<double>("kp_linear", 0.8);
        kp_ang_ = declare_parameter<double>("kp_angular", 1.0);
        // 動作確認用に遅め
        max_lin_ = declare_parameter<double>("max_linear_vel", 0.10);
        max_ang_ = declare_parameter<double>("max_angular_vel", 0.20);
        // pickup_planner_node の到着判定 (arrive_pos_tol 0.05 m / 3 deg) より厳しくする
        // 台車の位置決め精度 (低速の不感帯・舵の切替) に合わせて緩めにする
        pos_tol_ = declare_parameter<double>("pos_tol", 0.05);
        yaw_tol_ = declare_parameter<double>("yaw_tol_deg", 3.0) * M_PI / 180.0;
        // 遅すぎると台車が反応しないので下限を付ける
        min_lin_ = declare_parameter<double>("min_linear_vel", 0.03);
        min_ang_ = declare_parameter<double>("min_angular_vel", 0.05);
        snap_ratio_ = declare_parameter<double>("snap_ratio", 0.3);
        pose_timeout_ = declare_parameter<double>("pose_timeout", 0.5);
        const double rate = declare_parameter<double>("rate", 20.0);

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
        sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                goal_x_ = m->pose.position.x;
                goal_y_ = m->pose.position.y;
                goal_yaw_ = yaw_from_quat(m->pose.orientation);
                has_goal_ = true;
                RCLCPP_INFO(get_logger(), "新しい目標: (%.2f, %.2f, %.1f deg)", goal_x_, goal_y_,
                            goal_yaw_ * 180.0 / M_PI);
            });
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, rclcpp::SystemDefaultsQoS(), [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                x_ = m->pose.position.x;
                y_ = m->pose.position.y;
                yaw_ = yaw_from_quat(m->pose.orientation);
                pose_rx_ = now();
                have_pose_ = true;
            });
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                                   std::bind(&SimpleGoalFollowerNode::control, this));

        RCLCPP_INFO(get_logger(), "SimpleGoalFollowerNode started. max v=%.2f m/s w=%.2f rad/s tol=%.3f m/%.1f deg",
                    max_lin_, max_ang_, pos_tol_, yaw_tol_ * 180.0 / M_PI);
    }

private:
    void stop()
    {
        if (moving_) pub_cmd_->publish(geometry_msgs::msg::Twist());
        moving_ = false;
    }

    void control()
    {
        if (!has_goal_) return stop();
        if (!have_pose_ || (now() - pose_rx_).seconds() > pose_timeout_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/robot_pose が古い/未受信のため停止");
            return stop();
        }

        const double ex = goal_x_ - x_, ey = goal_y_ - y_;
        const double eyaw = normalize_angle(goal_yaw_ - yaw_);
        const double dist = std::hypot(ex, ey);
        if (dist < pos_tol_ && std::abs(eyaw) < yaw_tol_)
        {
            RCLCPP_INFO(get_logger(), "目標に到着 (ずれ %.3f m, %.2f deg)", dist, eyaw * 180.0 / M_PI);
            has_goal_ = false;
            return stop();
        }

        geometry_msgs::msg::Twist cmd;
        if (dist >= pos_tol_)
        {
            // 並進 (angular.z = 0)。map 系のずれを機体座標へ
            const double c = std::cos(yaw_), s = std::sin(yaw_);
            double vx = c * ex + s * ey;
            double vy = -s * ex + c * ey;
            if (std::abs(vx) < snap_ratio_ * std::abs(vy)) vx = 0.0;
            if (std::abs(vy) < snap_ratio_ * std::abs(vx)) vy = 0.0;
            const double e = std::hypot(vx, vy);
            const double v = std::clamp(kp_lin_ * dist, min_lin_, max_lin_);
            cmd.linear.x = v * vx / e;
            cmd.linear.y = v * vy / e;
        }
        else
        {
            // 位置は合った → その場で回転
            const double w = std::clamp(kp_ang_ * std::abs(eyaw), min_ang_, max_ang_);
            cmd.angular.z = std::copysign(w, eyaw);
        }
        pub_cmd_->publish(cmd);
        moving_ = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "残り %.3f m / %.1f deg  cmd (%.3f, %.3f, %.3f)",
                             dist, eyaw * 180.0 / M_PI, cmd.linear.x, cmd.linear.y, cmd.angular.z);
    }

    double kp_lin_{}, kp_ang_{}, max_lin_{}, max_ang_{}, min_lin_{}, min_ang_{}, snap_ratio_{};
    double pos_tol_{}, yaw_tol_{}, pose_timeout_{};
    double goal_x_{}, goal_y_{}, goal_yaw_{};
    double x_{}, y_{}, yaw_{};
    bool has_goal_{false}, have_pose_{false}, moving_{false};
    rclcpp::Time pose_rx_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_, sub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGoalFollowerNode>());
    rclcpp::shutdown();
    return 0;
}

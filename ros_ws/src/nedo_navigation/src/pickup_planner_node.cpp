#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

// Pickup planner node (簡易版 2026-09-29): ~/next (Enter) のたびに、今取れる一番高い点と、そこを取るための台車の目標姿勢を1回出す。
//   台車の移動は別ノード (本番は別研究室、動作確認は simple_goal_follower_node) が担当する。
//
//   前提: アームは機体に固定 (長辺に直角)、グラップルはアーム先端 (根元から r) に固定。台車は全方向移動で向きは変えない。
//   計算は全部機体座標 (base_footprint) で行う。奥 = アームの向き。
//
//   1. 最近傍点 = 物体の点のうち機体矩形に一番近い点 (アーム側 = 根元より奥の点だけ)
//   2. 奥行き範囲 = 最近傍点の奥行き a_n から、アームの向きに r − D 先まで (a_n ≤ a ≤ a_n + r − D)。横は制限しない
//        r = アーム長 (根元 → グラップル中心)、D = 機体と家電の最小距離 (根元はアーム側の長辺上)
//   3. 範囲内の物体の点の中で一番高い点 = 把持点
//   4. 目標姿勢 = 向きは今のまま、アーム先端が把持点の真上に来るように並進 (根元 = 把持点 − r × アームの向き)
//   5. 範囲内に点がなければ「完了」
//
//   r − D にしているので、目標で機体の縁と最近傍点の距離は (奥行き方向で) D 以上になる
//   物体の点は map 系で届く (scrap_pile_skirt_node が自己位置で map 系にして出す。2026-10-16 LM が見えている前提に変更)。
//   ~/next を受けた時の自己位置 (TF map → ground_frame) で機体座標に直して計算し、目標姿勢も同じ自己位置で map 系に戻す。

class PickupPlannerNode : public rclcpp::Node
{
public:
    PickupPlannerNode() : Node("pickup_planner_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        const auto object_topic = declare_parameter<std::string>("object_topic", "/scrap_pile_skirt_node/object_points");
        const auto goal_topic = declare_parameter<std::string>("goal_topic", "/pickup_pose");
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        ground_frame_ = declare_parameter<std::string>("ground_frame", "base_footprint");

        // アーム (scrap_pile_skirt_node と同じ値を launch から渡す)
        arm_base_x_ = declare_parameter<double>("arm_base_x", 0.0);
        arm_base_y_ = declare_parameter<double>("arm_base_y", 0.4);
        arm_yaw_ = declare_parameter<double>("arm_yaw_deg", 90.0) * M_PI / 180.0;
        arm_length_ = declare_parameter<double>("arm_length", 0.7);
        approach_limit_ = declare_parameter<double>("approach_limit", 0.3);
        // 機体矩形 [m, base_footprint]。scrap_pile_skirt_node の body_* と同じ値にする
        body_x_min_ = declare_parameter<double>("body_x_min", -0.5);
        body_x_max_ = declare_parameter<double>("body_x_max", 0.5);
        body_y_min_ = declare_parameter<double>("body_y_min", -0.4);
        body_y_max_ = declare_parameter<double>("body_y_max", 0.4);
        // 点群がこれより古ければ計算しない [s]
        max_cloud_age_ = declare_parameter<double>("max_cloud_age", 1.0);

        pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 1);
        pub_grasp_ = create_publisher<geometry_msgs::msg::PointStamped>("~/grasp_point", 1);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 1);

        auto qos = rclcpp::QoS(1);
        sub_object_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            object_topic, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { object_ = m; object_rx_ = now(); });
        srv_next_ = create_service<std_srvs::srv::Trigger>(
            "~/next", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res) { plan(*res); });

        RCLCPP_INFO(get_logger(), "PickupPlannerNode started. 奥行き範囲 = 最近傍点から r − D = %.2f − %.2f = %.2f m",
                    arm_length_, approach_limit_, arm_length_ - approach_limit_);
    }

private:
    struct Pt
    {
        double a, s, z;  // a = アームの向き (奥)、s = 横、機体座標 (base_footprint) 原点基準
    };

    // 機体矩形から点 (x, y) までの水平距離 (機体座標)
    double distance_to_body(double x, double y) const
    {
        const double dx = std::max({body_x_min_ - x, 0.0, x - body_x_max_});
        const double dy = std::max({body_y_min_ - y, 0.0, y - body_y_max_});
        return std::hypot(dx, dy);
    }

    // map 系の点 → 機体座標 (from_map) → アームの軸
    std::vector<Pt> to_arm_axes(const sensor_msgs::msg::PointCloud2 &msg, const tf2::Transform &from_map) const
    {
        const double c = std::cos(arm_yaw_), s = std::sin(arm_yaw_);
        std::vector<Pt> out;
        for (sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
             ix != ix.end(); ++ix, ++iy, ++iz)
        {
            const tf2::Vector3 p = from_map * tf2::Vector3(*ix, *iy, *iz);
            out.push_back({c * p.x() + s * p.y(), -s * p.x() + c * p.y(), p.z()});
        }
        return out;
    }

    // 受信時刻で見る (header.stamp はセンサの時計のことがある)
    bool too_old(const sensor_msgs::msg::PointCloud2::SharedPtr &m, const rclcpp::Time &rx) const
    {
        return !m || (now() - rx).seconds() > max_cloud_age_;
    }

    void plan(std_srvs::srv::Trigger::Response &res)
    {
        res.success = false;
        tf2::Transform to_map;  // ground_frame → map (今の自己位置)
        try
        {
            const auto t = tf_buffer_.lookupTransform(map_frame_, ground_frame_, tf2::TimePointZero).transform;
            to_map = tf2::Transform(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                                    tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
        }
        catch (const tf2::TransformException &ex)
        {
            res.message = std::string("自己位置がない (") + map_frame_ + " → " + ground_frame_ + "): " + ex.what();
        }
        if (res.message.empty() && arm_length_ - approach_limit_ <= 0.0) res.message = "r − D が 0 以下";
        else if (res.message.empty() && too_old(object_, object_rx_)) res.message = "点群が古い/未受信 (scrap_pile_skirt_node は動いているか)";
        if (!res.message.empty())
        {
            RCLCPP_WARN(get_logger(), "%s", res.message.c_str());
            return;
        }

        const std::vector<Pt> objects = to_arm_axes(*object_, to_map.inverse());
        const double ca = std::cos(arm_yaw_), sa = std::sin(arm_yaw_);
        const double base_a = ca * arm_base_x_ + sa * arm_base_y_;  // アーム根元の奥行き

        // 1. 最近傍点 (アーム側のみ)
        const Pt *n = nullptr;
        double n_d = 0.0;
        for (const auto &p : objects)
        {
            if (p.a <= base_a) continue;
            const double d = distance_to_body(ca * p.a - sa * p.s, sa * p.a + ca * p.s);
            if (!n || d < n_d)
            {
                n = &p;
                n_d = d;
            }
        }
        if (!n)
        {
            publish_markers(nullptr, nullptr, 0.0, {});
            res.success = true;
            res.message = "完了: アーム側に物体の点がない";
            RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
            return;
        }

        // 2-3. 奥行き範囲 a_n ≤ a ≤ a_n + r − D の中で一番高い点
        const double a_near = n->a, a_far = n->a + arm_length_ - approach_limit_;
        bool found = false;
        Pt g{};
        std::vector<Pt> in_band;
        for (const auto &p : objects)
        {
            if (p.a < a_near || p.a > a_far) continue;
            in_band.push_back(p);
            if (!found || p.z > g.z) g = p;
            found = true;
        }
        const Pt near = *n;
        publish_markers(&near, found ? &g : nullptr, a_far, in_band);
        if (!found)
        {
            res.success = true;
            res.message = "完了: 奥行き範囲に物体の点がない";
            RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
            return;
        }

        // 4. アーム根元を 把持点 − r × アームの向き に置く並進 (機体座標) → map 系
        const double gx = ca * g.a - sa * g.s, gy = sa * g.a + ca * g.s;  // 把持点 (機体座標 x, y)
        const double dx = gx - arm_length_ * ca - arm_base_x_;
        const double dy = gy - arm_length_ * sa - arm_base_y_;
        const double yaw = tf2::getYaw(to_map.getRotation());
        const tf2::Vector3 goal_xy = to_map * tf2::Vector3(dx, dy, 0.0);
        const tf2::Vector3 grasp_m = to_map * tf2::Vector3(gx, gy, g.z);

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = now();
        goal.header.frame_id = map_frame_;
        goal.pose.position.x = goal_xy.x();
        goal.pose.position.y = goal_xy.y();
        goal.pose.orientation.z = std::sin(0.5 * yaw);
        goal.pose.orientation.w = std::cos(0.5 * yaw);
        pub_goal_->publish(goal);

        geometry_msgs::msg::PointStamped grasp;
        grasp.header = goal.header;
        grasp.point.x = grasp_m.x();
        grasp.point.y = grasp_m.y();
        grasp.point.z = g.z;  // 高さは地面から (機体座標の z)
        pub_grasp_->publish(grasp);

        char buf[200];
        std::snprintf(buf, sizeof(buf), "把持点 高さ %.2f m (map %.2f, %.2f)。台車を 前後 %+.2f m / 左右 %+.2f m 動かす",
                      g.z, grasp.point.x, grasp.point.y, dx, dy);
        res.success = true;
        res.message = buf;
        RCLCPP_INFO(get_logger(), "%s", buf);
    }

    // 表示 (機体座標): 奥行き範囲 (範囲内の点の横幅だけ半透明の水色で塗る)、最近傍点 (白の球)、把持点 (橙の球)
    void publish_markers(const Pt *n, const Pt *g, double a_far, const std::vector<Pt> &in_band)
    {
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        const double ca = std::cos(arm_yaw_), sa = std::sin(arm_yaw_);
        auto pt = [&](double a, double s, double z) {
            geometry_msgs::msg::Point p;
            p.x = ca * a - sa * s;
            p.y = sa * a + ca * s;
            p.z = z;
            return p;
        };
        auto marker = [&](int id, int type) {
            visualization_msgs::msg::Marker m;
            m.header.stamp = object_->header.stamp;
            m.header.frame_id = ground_frame_;  // 計算した機体座標のまま描く
            m.ns = "pickup_planner";
            m.id = id;
            m.type = type;
            m.pose.orientation.w = 1.0;
            return m;
        };
        auto sphere = [&](int id, const Pt &p, float r, float gr, float b) {
            auto m = marker(id, visualization_msgs::msg::Marker::SPHERE);
            m.pose.position = pt(p.a, p.s, p.z);
            m.scale.x = m.scale.y = m.scale.z = 0.1;
            m.color.r = r;
            m.color.g = gr;
            m.color.b = b;
            m.color.a = 1.0f;
            arr.markers.push_back(m);
        };

        if (n && !in_band.empty())
        {
            double s_min = in_band.front().s, s_max = s_min;
            for (const auto &p : in_band)
            {
                s_min = std::min(s_min, p.s);
                s_max = std::max(s_max, p.s);
            }
            // 範囲内の一番高い点 (把持点) より少し上に描き、範囲内の点に色がかかるようにする
            const double z = g ? g->z + 0.02 : 0.0;
            auto m = marker(0, visualization_msgs::msg::Marker::TRIANGLE_LIST);
            m.scale.x = m.scale.y = m.scale.z = 1.0;
            m.color.g = 0.8f;
            m.color.b = 1.0f;
            m.color.a = 0.3f;
            const auto p0 = pt(n->a, s_min, z), p1 = pt(a_far, s_min, z), p2 = pt(a_far, s_max, z), p3 = pt(n->a, s_max, z);
            m.points = {p0, p1, p2, p0, p2, p3};
            arr.markers.push_back(m);
        }
        if (n) sphere(1, *n, 1.0f, 1.0f, 1.0f);
        if (g) sphere(2, *g, 1.0f, 0.5f, 0.0f);
        pub_markers_->publish(arr);
    }

    std::string map_frame_, ground_frame_;
    double arm_base_x_{}, arm_base_y_{}, arm_yaw_{}, arm_length_{}, approach_limit_{};
    double body_x_min_{}, body_x_max_{}, body_y_min_{}, body_y_max_{}, max_cloud_age_{};
    sensor_msgs::msg::PointCloud2::SharedPtr object_;
    rclcpp::Time object_rx_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_grasp_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_object_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_next_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PickupPlannerNode>());
    rclcpp::shutdown();
    return 0;
}

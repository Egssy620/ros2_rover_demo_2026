#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <optional>
#include <vehicle_interface/msg/pose_stamped_array.hpp>

class LandmarkMapVisualizerNode : public rclcpp::Node
{
public:
    LandmarkMapVisualizerNode() : Node("landmark_map_visualizer_node")
    {
        // パラメータ宣言
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::vector<std::string>>("region_ids",
                                                          std::vector<std::string>{"region1"});

        // ランドマークは /landmark_map (transient_local) 専用。
        // landmark_map_server 側が yaml (新形式/旧形式) を吸収して publish する契約。
        // visualizer はトピック未受信時は何も描かない (transient_local なので起動直後に
        // すぐ届く前提)。地図 yaml の自前パースは持たない (DRY 違反回避)。

        // 領域パラメータ（円筒形）
        this->declare_parameter<double>("region1.center_x", 3.75);
        this->declare_parameter<double>("region1.center_y", 5.0);
        this->declare_parameter<double>("region1.center_z", 0.0);
        this->declare_parameter<double>("region1.diameter", 2.5);
        this->declare_parameter<double>("region1.z_min", -2.0);
        this->declare_parameter<double>("region1.z_max", 3.8);

        // パブリッシャー作成
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/landmark_map_markers", 10);

        // サブスクライブ（/landmark_map があればそこから描画）
        map_sub_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
            "/landmark_map", rclcpp::QoS(1).transient_local().reliable(),
            [this](const vehicle_interface::msg::PoseStampedArray::SharedPtr msg)
            {
                latest_map_ = *msg;
            });

        // タイマー作成（1Hzで更新）
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LandmarkMapVisualizerNode::publish_markers, this));

        RCLCPP_INFO(this->get_logger(), "Landmark map visualizer started");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<vehicle_interface::msg::PoseStampedArray>::SharedPtr map_sub_;
    vehicle_interface::msg::PoseStampedArray latest_map_;
    std::string map_frame_;

    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        // ランドマークを可視化 (/landmark_map 専用)
        for (const auto &p : latest_map_.array)
        {
            const std::string id = p.header.frame_id; // id を frame_id に格納
            auto m = create_landmark_marker_from_pose(id, p, marker_id++);
            if (m)
                marker_array.markers.push_back(*m);
            auto t = create_text_marker_from_pose(id, p, marker_id++);
            if (t)
                marker_array.markers.push_back(*t);
        }

        // 領域を可視化
        auto region_ids = this->get_parameter("region_ids").as_string_array();
        for (const auto &region_id : region_ids)
        {
            auto marker = create_region_marker(region_id, marker_id++);
            if (marker)
            {
                marker_array.markers.push_back(*marker);

                // 領域にテキストラベルを追加
                auto text_marker = create_region_text_marker(region_id, marker_id++);
                if (text_marker)
                {
                    marker_array.markers.push_back(*text_marker);
                }
            }
        }

        marker_pub_->publish(marker_array);
    }

    std::optional<visualization_msgs::msg::Marker> create_landmark_marker_from_pose(
        const std::string &landmark_id, const geometry_msgs::msg::PoseStamped &pose, int marker_id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = this->now();
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = pose.pose;
        marker.pose.position.z += 0.5; // 少し上に表示

        marker.scale.x = 0.8;
        marker.scale.y = 0.8;
        marker.scale.z = 0.8;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        marker.ns = "landmarks";
        marker.lifetime = rclcpp::Duration::from_seconds(2);

        return marker;
    }

    std::optional<visualization_msgs::msg::Marker> create_text_marker_from_pose(
        const std::string &landmark_id, const geometry_msgs::msg::PoseStamped &pose, int marker_id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = this->now();
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = pose.pose;
        marker.pose.position.z += 1.2; // ランドマークより上に表示
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.5; // テキストサイズ

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.text = landmark_id;
        marker.ns = "landmark_labels";
        marker.lifetime = rclcpp::Duration::from_seconds(2);

        return marker;
    }

    std::optional<visualization_msgs::msg::Marker> create_region_marker(
        const std::string &region_id, int marker_id)
    {
        // 円筒形領域のパラメータを取得
        std::string center_x_param = region_id + ".center_x";
        std::string center_y_param = region_id + ".center_y";
        std::string center_z_param = region_id + ".center_z";
        std::string diameter_param = region_id + ".diameter";
        std::string z_min_param = region_id + ".z_min";
        std::string z_max_param = region_id + ".z_max";

        // パラメータの存在確認
        if (!this->has_parameter(center_x_param) ||
            !this->has_parameter(center_y_param) ||
            !this->has_parameter(center_z_param) ||
            !this->has_parameter(diameter_param) ||
            !this->has_parameter(z_min_param) ||
            !this->has_parameter(z_max_param))
        {
            RCLCPP_WARN(this->get_logger(), "Missing parameters for region %s", region_id.c_str());
            return std::nullopt;
        }

        auto center_x = this->get_parameter(center_x_param).as_double();
        auto center_y = this->get_parameter(center_y_param).as_double();
        auto center_z = this->get_parameter(center_z_param).as_double();
        auto diameter = this->get_parameter(diameter_param).as_double();
        auto z_min = this->get_parameter(z_min_param).as_double();
        auto z_max = this->get_parameter(z_max_param).as_double();

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = this->now();
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 円筒の中心を計算（Z軸の中心）
        marker.pose.position.x = center_x;
        marker.pose.position.y = center_y;
        marker.pose.position.z = center_z + (z_max + z_min) / 2.0;
        marker.pose.orientation.w = 1.0;

        // 円筒のサイズ設定
        marker.scale.x = diameter;      // 直径
        marker.scale.y = diameter;      // 直径
        marker.scale.z = z_max - z_min; // 高さ

        // 色設定（半透明の緑）
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;

        marker.ns = "regions";
        marker.lifetime = rclcpp::Duration::from_seconds(2);

        RCLCPP_INFO_ONCE(this->get_logger(), "Created cylinder region at (%.2f, %.2f, %.2f) with diameter %.2f and height %.2f",
                         center_x, center_y, marker.pose.position.z, diameter, marker.scale.z);

        return marker;
    }

    std::optional<visualization_msgs::msg::Marker> create_region_text_marker(
        const std::string &region_id, int marker_id)
    {
        std::string center_x_param = region_id + ".center_x";
        std::string center_y_param = region_id + ".center_y";
        std::string center_z_param = region_id + ".center_z";
        std::string z_max_param = region_id + ".z_max";

        if (!this->has_parameter(center_x_param) ||
            !this->has_parameter(center_y_param) ||
            !this->has_parameter(center_z_param) ||
            !this->has_parameter(z_max_param))
        {
            return std::nullopt;
        }

        auto center_x = this->get_parameter(center_x_param).as_double();
        auto center_y = this->get_parameter(center_y_param).as_double();
        auto center_z = this->get_parameter(center_z_param).as_double();
        auto z_max = this->get_parameter(z_max_param).as_double();

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = this->now();
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = center_x;
        marker.pose.position.y = center_y;
        marker.pose.position.z = z_max + 1.0; // 円筒の上部より上に表示
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.7; // テキストサイズ

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.text = region_id + " (Cylinder Region)";
        marker.ns = "region_labels";
        marker.lifetime = rclcpp::Duration::from_seconds(2);

        return marker;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkMapVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}

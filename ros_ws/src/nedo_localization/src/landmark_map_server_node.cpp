#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vehicle_interface/msg/pose_stamped_array.hpp>

class LandmarkMapServerNode : public rclcpp::Node
{
public:
    LandmarkMapServerNode() : Node("landmark_map_server_node")
    {
        pub_ = this->create_publisher<vehicle_interface::msg::PoseStampedArray>(
            "/landmark_map", rclcpp::QoS(1).transient_local().reliable());
        double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 1.0);
        map_frame_ = this->declare_parameter<std::string>("landmark_frame", "map");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(0.1, publish_rate_hz))),
            std::bind(&LandmarkMapServerNode::publish_map, this));
        load_map_from_params();
        // 起動直後に1回 publish (transient_local subscriber の初回受信遅延を解消)
        publish_map();
    }

private:
    rclcpp::Publisher<vehicle_interface::msg::PoseStampedArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    vehicle_interface::msg::PoseStampedArray map_cache_;
    std::string map_frame_;

    void load_map_from_params()
    {
        map_cache_.array.clear();

        // Try new format first: landmark_ids + <id>.position
        std::vector<std::string> landmark_ids_default;
        std::vector<std::string> landmark_ids =
            this->declare_parameter<std::vector<std::string>>("landmark_ids", landmark_ids_default);

        bool loaded = false;
        if (!landmark_ids.empty())
        {
            for (const auto &id : landmark_ids)
            {
                std::vector<double> pos_default;
                std::vector<double> pos = this->declare_parameter<std::vector<double>>(id + ".position", pos_default);
                if (pos.size() >= 2)
                {
                    geometry_msgs::msg::PoseStamped p;
                    p.header.frame_id = id; // use frame_id as ID tag
                    p.header.stamp = this->now();
                    p.pose.position.x = pos[0];
                    p.pose.position.y = pos[1];
                    p.pose.position.z = pos.size() >= 3 ? pos[2] : 0.0;
                    p.pose.orientation.w = 1.0;
                    map_cache_.array.push_back(p);
                }
            }
            loaded = !map_cache_.array.empty();
        }

        if (!loaded)
        {
            // Legacy format LM_ids/LM_xs/LM_ys
            std::vector<std::string> ids_default;
            std::vector<double> xs_default;
            std::vector<double> ys_default;
            std::vector<std::string> ids = this->declare_parameter<std::vector<std::string>>("LM_ids", ids_default);
            std::vector<double> xs = this->declare_parameter<std::vector<double>>("LM_xs", xs_default);
            std::vector<double> ys = this->declare_parameter<std::vector<double>>("LM_ys", ys_default);
            if (ids.size() == xs.size() && ids.size() == ys.size())
            {
                for (size_t i = 0; i < ids.size(); ++i)
                {
                    geometry_msgs::msg::PoseStamped p;
                    p.header.frame_id = ids[i];
                    p.header.stamp = this->now();
                    p.pose.position.x = xs[i];
                    p.pose.position.y = ys[i];
                    p.pose.position.z = 0.0;
                    p.pose.orientation.w = 1.0;
                    map_cache_.array.push_back(p);
                }
                loaded = true;
            }
            else if (!ids.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "LM_* size mismatch: ids=%zu xs=%zu ys=%zu", ids.size(), xs.size(), ys.size());
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu landmarks from parameters", map_cache_.array.size());
    }

    void publish_map()
    {
        // NOTE: 旧実装は array が空の時 load_map_from_params を再呼び出ししていたが、
        // declare_parameter は二度目で ParameterAlreadyDeclaredException を投げるため
        // クラッシュ要因だった。初回ロードで空のままなら毎周期空の地図を publish する。
        const auto stamp = this->now();
        map_cache_.header.stamp = stamp;
        map_cache_.header.frame_id = map_frame_;   // 旧実装は frame_id 未設定 (空文字)
        for (auto &p : map_cache_.array)
            p.header.stamp = stamp;
        pub_->publish(map_cache_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkMapServerNode>());
    rclcpp::shutdown();
    return 0;
}

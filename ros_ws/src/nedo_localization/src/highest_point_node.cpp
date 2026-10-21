#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class HighestPointNode : public rclcpp::Node
{
public:
    HighestPointNode() : Node("highest_point_node")
    {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/filtered_points");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/filtered_points_highest_point");
        pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_topic_, 10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&HighestPointNode::callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "HighestPointNode started: subscribing %s publishing %s", input_topic_.c_str(), output_topic_.c_str());
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->width * msg->height == 0)
            return;
        int x_off = -1, y_off = -1, z_off = -1, step = msg->point_step;
        for (auto &f : msg->fields)
        {
            if (f.name == "x")
                x_off = f.offset;
            else if (f.name == "y")
                y_off = f.offset;
            else if (f.name == "z")
                z_off = f.offset;
        }
        if (x_off < 0 || y_off < 0 || z_off < 0)
            return;
        float max_z = -1e30f;
        size_t best_idx = SIZE_MAX;
        size_t total = msg->width * msg->height;
        for (size_t i = 0; i < total; ++i)
        {
            const uint8_t *pt = &msg->data[i * step];
            float z;
            std::memcpy(&z, pt + z_off, sizeof(float));
            if (!std::isfinite(z))
                continue;
            if (z > max_z)
            {
                max_z = z;
                best_idx = i;
            }
        }
        if (best_idx == SIZE_MAX)
            return;
        const uint8_t *best_pt = &msg->data[best_idx * step];
        float x, y, z;
        std::memcpy(&x, best_pt + x_off, 4);
        std::memcpy(&y, best_pt + y_off, 4);
        std::memcpy(&z, best_pt + z_off, 4);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            return;
        geometry_msgs::msg::PointStamped out;
        out.header = msg->header;
        out.point.x = x;
        out.point.y = y;
        out.point.z = z;
        pub_->publish(out);
    }
    std::string input_topic_, output_topic_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HighestPointNode>());
    rclcpp::shutdown();
    return 0;
}

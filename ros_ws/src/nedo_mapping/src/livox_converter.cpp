// Converts PointCloud2 to LIO-SAM compatible format (ring uint16, time float32 relative seconds).
// Supports two input formats:
//   Livox (livox_ros_driver2): fields line(uint8) + timestamp(float64 ns) → ring + time(s)
//   Hesai (HesaiLidar_ROS_2.0): fields ring(uint16) + timestamp(float64 s) → ring + time(s)

#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Identical definition to LIO-SAM's imageProjection.cpp so pcl::toROSMsg
// produces the exact byte layout that pcl::moveFromROSMsg expects there.
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(float, time, time)
)

class LivoxConverter : public rclcpp::Node
{
public:
    LivoxConverter() : Node("livox_converter")
    {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "input", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { convert(msg); });

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "output", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "livox_converter ready");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    // Returns byte offset of a named field, -1 if not found.
    static int offset_of(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
    {
        for (const auto & f : msg.fields) {
            if (f.name == name) return static_cast<int>(f.offset);
        }
        return -1;
    }

    void convert(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
    {
        const int off_x   = offset_of(*msg, "x");
        const int off_y   = offset_of(*msg, "y");
        const int off_z   = offset_of(*msg, "z");
        const int off_i   = offset_of(*msg, "intensity");
        const int off_line = offset_of(*msg, "line");      // Livox: uint8 laser line id
        const int off_ring = offset_of(*msg, "ring");      // Hesai: uint16 channel id
        const int off_ts   = offset_of(*msg, "timestamp"); // both: float64, Livox=ns, Hesai=s

        if (off_x < 0 || off_y < 0 || off_z < 0 || off_i < 0) {
            RCLCPP_WARN_ONCE(get_logger(), "Missing xyz/intensity field, dropping message.");
            return;
        }

        // Unknown format: no ring info at all
        if (off_line < 0 && off_ring < 0) {
            RCLCPP_WARN_ONCE(get_logger(), "No 'line' or 'ring' field; publishing as-is.");
            sensor_msgs::msg::PointCloud2 passthrough = *msg;
            passthrough.header.stamp = this->now();
            pub_->publish(passthrough);
            return;
        }

        const bool is_hesai = (off_line < 0 && off_ring >= 0);

        const uint32_t n    = msg->width * msg->height;
        const uint32_t step = msg->point_step;
        const uint8_t * src = msg->data.data();

        pcl::PointCloud<VelodynePointXYZIRT> cloud;
        cloud.reserve(n);

        double t0 = 0.0;
        if (off_ts >= 0 && n > 0) {
            std::memcpy(&t0, src + off_ts, sizeof(double));
        }

        for (uint32_t i = 0; i < n; ++i) {
            const uint8_t * p = src + i * step;

            VelodynePointXYZIRT pt;
            std::memcpy(&pt.x,         p + off_x, 4);
            std::memcpy(&pt.y,         p + off_y, 4);
            std::memcpy(&pt.z,         p + off_z, 4);
            std::memcpy(&pt.intensity, p + off_i, 4);

            if (is_hesai) {
                // Hesai: ring=uint16, timestamp=float64 seconds
                std::memcpy(&pt.ring, p + off_ring, sizeof(uint16_t));
                if (off_ts >= 0) {
                    double ts;
                    std::memcpy(&ts, p + off_ts, sizeof(double));
                    pt.time = static_cast<float>(ts - t0);  // seconds → relative seconds
                } else {
                    pt.time = 0.0f;
                }
            } else {
                // Livox: line=uint8→ring, timestamp=float64 nanoseconds
                pt.ring = static_cast<uint16_t>(*(p + off_line));
                if (off_ts >= 0) {
                    double ts;
                    std::memcpy(&ts, p + off_ts, sizeof(double));
                    pt.time = static_cast<float>((ts - t0) * 1e-9);  // nanoseconds → seconds
                } else {
                    pt.time = 0.0f;
                }
            }

            cloud.push_back(pt);
        }

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(cloud, out);
        out.header = msg->header;
        // Stamp 80ms in the past so imageProjection's scan-end window
        // (stamp + scan_period/2) stays behind the re-stamped IMU stream.
        out.header.stamp = this->now();
        pub_->publish(out);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxConverter>());
    rclcpp::shutdown();
    return 0;
}

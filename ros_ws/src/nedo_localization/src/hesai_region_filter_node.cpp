#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cstring>
#include <limits>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

// PCL for clustering
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <cmath>
#include <mutex>
#include <atomic>

// Dynamic cylinder extraction based on robot pose and world-frame marker coordinates

using namespace std::chrono_literals;

class HesaiRegionFilterNode : public rclcpp::Node
{
public:
    std::atomic<bool> enabled_{false};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;

    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
    enabled_.store(msg->data, std::memory_order_relaxed);
    RCLCPP_INFO(get_logger(), "[scrap_detection] enabled=%s", msg->data ? "true" : "false");
    }
    HesaiRegionFilterNode() : Node("hesai_region_filter_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Parameters
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/lidar_points");
        input_frame_ = this->declare_parameter<std::string>("input_frame", "lidar_link");
        output_frame_ = this->declare_parameter<std::string>("output_frame", "base_link");
        ground_frame_ = this->declare_parameter<std::string>("ground_frame", "base_footprint");
        transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.1);
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/filtered_points");
        
        // Dynamic marker-based cylinder: use robot_pose to transform world markers to robot frame
        use_dynamic_marker_ = this->declare_parameter<bool>("use_dynamic_marker", false);
        robot_pose_topic_ = this->declare_parameter<std::string>("robot_pose_topic", "/robot_pose");
        // World-frame marker coordinates (marker0 and marker7 define diameter)
        marker0_world_x_ = this->declare_parameter<double>("marker0_world_x", -1.4);
        marker0_world_y_ = this->declare_parameter<double>("marker0_world_y", 1.4);
        marker7_world_x_ = this->declare_parameter<double>("marker7_world_x", -3.4);
        marker7_world_y_ = this->declare_parameter<double>("marker7_world_y", 1.4);
        robot_pose_timeout_ = this->declare_parameter<double>("robot_pose_timeout", 1.0);
        // dynamic marker モードで robot_pose が timeout したときの挙動。
        //   "freeze"   : 最後に計算した円柱を使い続ける (機体が動くと ROI が取り残される)
        //   "suppress" : ROI 出力を抑制する (受信再開まで /scrap_filtered_points は publish しない)
        //   "reset"    : have_cylinder_ をクリアして次の robot_pose を待つ (受信再開後に再構築)
        // 起動時に robot_pose が一度も来ていない場合は常に "suppress" と同じ扱い (world coord
        // fallback は機体が原点にいる前提のため誤った ROI を出す危険があり、廃止)。
        pose_timeout_action_ = this->declare_parameter<std::string>("pose_timeout_action", "freeze");
        if (pose_timeout_action_ != "freeze" &&
            pose_timeout_action_ != "suppress" &&
            pose_timeout_action_ != "reset")
        {
            RCLCPP_WARN(this->get_logger(),
                        "pose_timeout_action='%s' is invalid, falling back to 'freeze'",
                        pose_timeout_action_.c_str());
            pose_timeout_action_ = "freeze";
        }
        
        // Static marker-based cylinder specification (for backwards compatibility)
        use_marker_diameter_ = this->declare_parameter<bool>("use_marker_diameter", false);
        marker1_x_ = this->declare_parameter<double>("marker1_x", 0.0);
        marker1_y_ = this->declare_parameter<double>("marker1_y", 0.0);
        marker2_x_ = this->declare_parameter<double>("marker2_x", 0.0);
        marker2_y_ = this->declare_parameter<double>("marker2_y", 0.0);
        
        // Legacy: Fixed cylinder specification (relative to robot base frame):
        forward_offset_ = this->declare_parameter<double>("forward_offset", 0.0);
        left_offset_ = this->declare_parameter<double>("left_offset", 1.5);
        fixed_radius_ = this->declare_parameter<double>("fixed_radius", 1.0);
        cylinder_z_min_ = this->declare_parameter<double>("cylinder_z_min", -1e9);
        cylinder_z_max_ = this->declare_parameter<double>("cylinder_z_max", 1e9);
        
        // Ground-relative height filtering
        enable_ground_relative_ = this->declare_parameter<bool>("enable_ground_relative", false);
        ground_relative_min_height_ = this->declare_parameter<double>("ground_relative_min_height", 0.2);
        ground_relative_max_height_ = this->declare_parameter<double>("ground_relative_max_height", 2.0);
        
        // Clustering parameters
        enable_clustering_ = this->declare_parameter<bool>("enable_clustering", false);
        cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.1);  // 10cm
        min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 200);
        
        enable_max_z_output_ = this->declare_parameter<bool>("enable_max_z_output", false);
        max_z_topic_ = this->declare_parameter<std::string>("max_z_topic", "/scrap_pile_max_height");
        debug_output_ = this->declare_parameter<bool>("debug_output", false);
        publish_transform_info_ = this->declare_parameter<bool>("publish_transform_info", false);
        log_stats_ = this->declare_parameter<bool>("log_stats", true);
        stats_interval_sec_ = this->declare_parameter<double>("stats_interval_sec", 5.0);
        enabled_.store(this->declare_parameter<bool>("enabled_on_startup", false));

        // Publishers / Subscribers
        pub_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
        if (enable_max_z_output_)
        {
            pub_max_z_ = this->create_publisher<std_msgs::msg::Float32>(max_z_topic_, 10);
        }
        

        // QoS with larger queue depth to avoid message drops during heavy processing
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);  // 最新1つだけ保持（処理が追いつかない場合は古いメッセージをドロップ）
        
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&HesaiRegionFilterNode::pointcloud_callback, this, std::placeholders::_1));

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/scrap_detection/enable", 10,
            std::bind(&HesaiRegionFilterNode::enable_callback, this, std::placeholders::_1));
        // Subscribe to robot_pose if dynamic marker mode is enabled
        if (use_dynamic_marker_)
        {
            sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                robot_pose_topic_, rclcpp::SensorDataQoS(),
                std::bind(&HesaiRegionFilterNode::robot_pose_callback, this, std::placeholders::_1));

            // 旧実装はここで world 座標を直接円柱化して have_cylinder_=true にしていたが、
            // それは「機体が world 原点に yaw=0 で立っている」前提と同義で、実機の起動位置が
            // 原点でないとき完全に間違った ROI が出てしまう。robot_pose 受信前は円柱未確定とし、
            // pointcloud_callback で出力抑制する。
            have_cylinder_ = false;

            RCLCPP_INFO(this->get_logger(),
                        "Dynamic marker mode enabled. Subscribing to %s (pose_timeout_action=%s)",
                        robot_pose_topic_.c_str(), pose_timeout_action_.c_str());
            RCLCPP_INFO(this->get_logger(),
                        "World markers: marker0=(%.3f,%.3f) marker7=(%.3f,%.3f). "
                        "Suppressing ROI output until first robot_pose is received.",
                        marker0_world_x_, marker0_world_y_, marker7_world_x_, marker7_world_y_);
        }
        else if (use_marker_diameter_)
        {
            // Static marker mode: compute cylinder once at startup
            center_x_ = (marker1_x_ + marker2_x_) / 2.0;
            center_y_ = (marker1_y_ + marker2_y_) / 2.0;
            double dx = marker2_x_ - marker1_x_;
            double dy = marker2_y_ - marker1_y_;
            radius_ = std::sqrt(dx * dx + dy * dy) / 2.0;
            have_cylinder_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Static marker-based cylinder: marker1=(%.3f,%.3f) marker2=(%.3f,%.3f) -> center=(%.3f,%.3f) radius=%.3f",
                        marker1_x_, marker1_y_, marker2_x_, marker2_y_, center_x_, center_y_, radius_);
        }
        else
        {
            // Legacy fixed cylinder
            center_x_ = forward_offset_;
            center_y_ = left_offset_;
            radius_ = fixed_radius_;
            have_cylinder_ = true;
            if (debug_output_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Fixed cylinder: center=(%.3f,%.3f) r=%.3f z_range=[%.2f,%.2f] frame=%s",
                            center_x_, center_y_, radius_, cylinder_z_min_, cylinder_z_max_, output_frame_.c_str());
            }
        }

        if (log_stats_ && stats_interval_sec_ > 0.0)
        {
            stats_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(stats_interval_sec_),
                std::bind(&HesaiRegionFilterNode::print_stats, this));
        }

        RCLCPP_INFO(this->get_logger(), "HesaiRegionFilterNode started. input_topic=%s, output_topic=%s, %s->%s",
                    input_topic_.c_str(), output_topic_.c_str(), input_frame_.c_str(), output_frame_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Mode: dynamic_marker=%s static_marker=%s fixed=%s",
                    use_dynamic_marker_ ? "true" : "false",
                    use_marker_diameter_ ? "true" : "false",
                    (!use_dynamic_marker_ && !use_marker_diameter_) ? "true" : "false");
    }

private:
    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_robot_pose_ = *msg;
        have_robot_pose_ = true;
        
        // Transform world-frame markers to robot frame using inverse of robot pose
        update_cylinder_from_robot_pose(*msg);
    }

    void update_cylinder_from_robot_pose(const geometry_msgs::msg::PoseStamped &robot_pose)
    {
        double robot_x = robot_pose.pose.position.x;
        double robot_y = robot_pose.pose.position.y;
        
        tf2::Quaternion q(
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        double cos_yaw = std::cos(-yaw);
        double sin_yaw = std::sin(-yaw);
        
        double dx0 = marker0_world_x_ - robot_x;
        double dy0 = marker0_world_y_ - robot_y;
        double marker0_robot_x = cos_yaw * dx0 - sin_yaw * dy0;
        double marker0_robot_y = sin_yaw * dx0 + cos_yaw * dy0;
        
        double dx7 = marker7_world_x_ - robot_x;
        double dy7 = marker7_world_y_ - robot_y;
        double marker7_robot_x = cos_yaw * dx7 - sin_yaw * dy7;
        double marker7_robot_y = sin_yaw * dx7 + cos_yaw * dy7;
        
        double new_center_x = (marker0_robot_x + marker7_robot_x) / 2.0;
        double new_center_y = (marker0_robot_y + marker7_robot_y) / 2.0;
        double ddx = marker7_robot_x - marker0_robot_x;
        double ddy = marker7_robot_y - marker0_robot_y;
        double new_radius = std::sqrt(ddx * ddx + ddy * ddy) / 2.0;
        
        center_x_ = new_center_x;
        center_y_ = new_center_y;
        radius_ = new_radius;
        have_cylinder_ = true;
        
        if (debug_output_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Robot pose: (%.2f, %.2f, yaw=%.2f). Cylinder: center=(%.2f, %.2f), radius=%.2f",
                robot_x, robot_y, yaw, center_x_, center_y_, radius_);
        }
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!enabled_.load(std::memory_order_relaxed)) {
            return;
        }

        received_count_++;
        
        if (use_dynamic_marker_)
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!have_robot_pose_)
            {
                // robot_pose 未受信時は ROI 出力を抑制する (world 座標 fallback は廃止)。
                // localizer が起動して /robot_pose を流し始めるまで /scrap_filtered_points は出ない。
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "robot_pose not yet received on %s; suppressing ROI output",
                    robot_pose_topic_.c_str());
                return;
            }

            auto now = this->now();
            auto pose_age = (now - rclcpp::Time(latest_robot_pose_.header.stamp)).seconds();
            if (pose_age > robot_pose_timeout_)
            {
                if (pose_timeout_action_ == "suppress")
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Robot pose stale (age=%.2fs > %.2fs); suppressing ROI output",
                        pose_age, robot_pose_timeout_);
                    return;
                }
                else if (pose_timeout_action_ == "reset")
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Robot pose stale (age=%.2fs > %.2fs); resetting cylinder, waiting for next pose",
                        pose_age, robot_pose_timeout_);
                    have_cylinder_ = false;
                    return;
                }
                else  // "freeze"
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Robot pose stale (age=%.2fs > %.2fs); freezing on last known cylinder",
                        pose_age, robot_pose_timeout_);
                    // fall through with the most recent cylinder
                }
            }

            // 円柱が一度も計算されていなければ (起動直後の reset 直後など) 出力抑制。
            if (!have_cylinder_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Cylinder not yet established; suppressing ROI output");
                return;
            }
        }
        
        const std::string src_frame = msg->header.frame_id.empty() ? input_frame_ : msg->header.frame_id;

        geometry_msgs::msg::TransformStamped tf;
        try
        {
            // 点群のタイムスタンプがLiDAR内部時計で同期されていない場合があるため、
            // 最新のTFを使用する（rclcpp::Time(0) = 最新のTF）
            tf = tf_buffer_.lookupTransform(output_frame_, src_frame, rclcpp::Time(0),
                                            rclcpp::Duration::from_seconds(transform_timeout_));
            if (publish_transform_info_ && !printed_tf_info_)
            {
                RCLCPP_INFO(this->get_logger(), "Using TF %s -> %s", src_frame.c_str(), output_frame_.c_str());
                printed_tf_info_ = true;
            }
        }
        catch (const tf2::TransformException &ex)
        {
            if (debug_output_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
            }
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_out;
        try
        {
            tf2::doTransform(*msg, cloud_out, tf);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "PointCloud transform failed: %s", e.what());
            return;
        }

        double cx, cy, r;
        bool have_cyl;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            cx = center_x_;
            cy = center_y_;
            r = radius_;
            have_cyl = have_cylinder_;
        }

        size_t original_points = cloud_out.width * cloud_out.height;
        size_t kept_points = original_points;
        if (have_cyl)
        {
            sensor_msgs::msg::PointCloud2 filtered;
            if (enable_ground_relative_)
            {
                // Transform to base_footprint frame for ground-relative filtering
                sensor_msgs::msg::PointCloud2 cloud_ground_frame;
                try
                {
                    geometry_msgs::msg::TransformStamped tf_ground = tf_buffer_.lookupTransform(
                        ground_frame_, cloud_out.header.frame_id, cloud_out.header.stamp,
                        rclcpp::Duration::from_seconds(transform_timeout_));
                    tf2::doTransform(cloud_out, cloud_ground_frame, tf_ground);
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "TF lookup failed for ground frame %s: %s", ground_frame_.c_str(), ex.what());
                    return;
                }
                
                // Filter in ground frame (Z=0 is ground)
                filter_by_cylinder_and_ground_relative(cloud_ground_frame, filtered, cx, cy, r, 
                                                       ground_relative_min_height_, 
                                                       ground_relative_max_height_);
                
                // Transform back to output_frame if needed
                if (ground_frame_ != output_frame_)
                {
                    try
                    {
                        geometry_msgs::msg::TransformStamped tf_back = tf_buffer_.lookupTransform(
                            output_frame_, ground_frame_, filtered.header.stamp,
                            rclcpp::Duration::from_seconds(transform_timeout_));
                        sensor_msgs::msg::PointCloud2 filtered_output_frame;
                        tf2::doTransform(filtered, filtered_output_frame, tf_back);
                        filtered = std::move(filtered_output_frame);
                    }
                    catch (const tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(this->get_logger(), "TF back to output frame failed: %s", ex.what());
                        // Continue with filtered cloud in ground frame
                    }
                }
            }
            else
            {
                filter_by_cylinder_and_z(cloud_out, filtered, cx, cy, r, cylinder_z_min_, cylinder_z_max_);
            }
            cloud_out = std::move(filtered);
            kept_points = cloud_out.width * cloud_out.height;
        }
        else if (cylinder_z_min_ > -1e8 || cylinder_z_max_ < 1e8)
        {
            sensor_msgs::msg::PointCloud2 filtered;
            filter_by_z(cloud_out, filtered, cylinder_z_min_, cylinder_z_max_);
            cloud_out = std::move(filtered);
            kept_points = cloud_out.width * cloud_out.height;
        }
        
        // Apply clustering if enabled
        if (enable_clustering_ && kept_points > 0)
        {
            sensor_msgs::msg::PointCloud2 clustered;
            apply_euclidean_clustering(cloud_out, clustered);
            size_t cluster_points = clustered.width * clustered.height;
            if (debug_output_)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Clustering: in=%zu cluster=%zu", kept_points, cluster_points);
            }
            cloud_out = std::move(clustered);
            kept_points = cluster_points;
        }

        if (enable_max_z_output_ && pub_max_z_)
        {
            float max_z = compute_max_z(cloud_out);
            std_msgs::msg::Float32 zmsg;
            zmsg.data = max_z;
            pub_max_z_->publish(zmsg);
        }

        pub_points_->publish(cloud_out);
        published_count_++;
        
        if (debug_output_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Filter: in=%zu kept=%zu center=(%.2f,%.2f) r=%.2f",
                original_points, kept_points, cx, cy, r);
        }
    }

    static bool get_xyz_fields(const sensor_msgs::msg::PointCloud2 &cloud, int &x_off, int &y_off, int &z_off, int &point_step)
    {
        x_off = y_off = z_off = -1;
        for (const auto &f : cloud.fields)
        {
            if (f.name == "x") x_off = f.offset;
            else if (f.name == "y") y_off = f.offset;
            else if (f.name == "z") z_off = f.offset;
        }
        point_step = cloud.point_step;
        return (x_off >= 0 && y_off >= 0 && z_off >= 0 && point_step > 0);
    }

    static float read_float(const uint8_t *ptr)
    {
        float v;
        std::memcpy(&v, ptr, sizeof(float));
        return v;
    }

    static void write_point(uint8_t *dst, const uint8_t *src, size_t step)
    {
        std::memcpy(dst, src, step);
    }

    static float compute_max_z(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        int x_off, y_off, z_off, step;
        if (!get_xyz_fields(cloud, x_off, y_off, z_off, step))
            return std::numeric_limits<float>::quiet_NaN();
        float max_z = -std::numeric_limits<float>::infinity();
        for (uint32_t r = 0; r < cloud.height; ++r)
        {
            const uint8_t *row = cloud.data.data() + r * cloud.row_step;
            for (uint32_t c = 0; c < cloud.width; ++c)
            {
                float z = read_float(row + c * step + z_off);
                if (std::isfinite(z) && z > max_z) max_z = z;
            }
        }
        return max_z;
    }
    
    static float compute_min_z(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        int x_off, y_off, z_off, step;
        if (!get_xyz_fields(cloud, x_off, y_off, z_off, step))
            return std::numeric_limits<float>::quiet_NaN();
        float min_z = std::numeric_limits<float>::infinity();
        for (uint32_t r = 0; r < cloud.height; ++r)
        {
            const uint8_t *row = cloud.data.data() + r * cloud.row_step;
            for (uint32_t c = 0; c < cloud.width; ++c)
            {
                float z = read_float(row + c * step + z_off);
                if (std::isfinite(z) && z < min_z) min_z = z;
            }
        }
        return min_z;
    }

    static void filter_by_z(const sensor_msgs::msg::PointCloud2 &in, sensor_msgs::msg::PointCloud2 &out, double zmin, double zmax)
    {
        int x_off, y_off, z_off, step;
        if (!get_xyz_fields(in, x_off, y_off, z_off, step)) { out = in; return; }
        out = in;
        out.data.clear();
        out.width = 0;
        out.height = 1;
        out.is_dense = false;
        std::vector<uint8_t> buffer;
        buffer.reserve(in.data.size());
        for (uint32_t r = 0; r < in.height; ++r)
        {
            const uint8_t *row = in.data.data() + r * in.row_step;
            for (uint32_t c = 0; c < in.width; ++c)
            {
                const uint8_t *pt = row + c * step;
                float z = read_float(pt + z_off);
                if (std::isfinite(z) && z >= zmin && z <= zmax)
                {
                    buffer.insert(buffer.end(), pt, pt + step);
                    out.width++;
                }
            }
        }
        out.row_step = out.width * step;
        out.data = std::move(buffer);
    }

    static void filter_by_cylinder_and_z(const sensor_msgs::msg::PointCloud2 &in, sensor_msgs::msg::PointCloud2 &out,
                                         double cx, double cy, double radius, double zmin, double zmax)
    {
        int x_off, y_off, z_off, step;
        if (!get_xyz_fields(in, x_off, y_off, z_off, step)) { out = in; return; }
        const double r2 = radius * radius;
        out = in;
        out.data.clear();
        out.width = 0;
        out.height = 1;
        out.is_dense = false;
        std::vector<uint8_t> buffer;
        buffer.reserve(in.data.size());
        for (uint32_t r = 0; r < in.height; ++r)
        {
            const uint8_t *row = in.data.data() + r * in.row_step;
            for (uint32_t c = 0; c < in.width; ++c)
            {
                const uint8_t *pt = row + c * step;
                float x = read_float(pt + x_off);
                float y = read_float(pt + y_off);
                float z = read_float(pt + z_off);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                double dx = x - cx;
                double dy = y - cy;
                if (dx*dx + dy*dy <= r2 && z >= zmin && z <= zmax)
                {
                    buffer.insert(buffer.end(), pt, pt + step);
                    out.width++;
                }
            }
        }
        out.row_step = out.width * step;
        out.data = std::move(buffer);
    }
    
    // Filter by cylinder and height relative to base_footprint (ground plane)
    // Point cloud should be in base_footprint frame where Z=0 is ground
    static void filter_by_cylinder_and_ground_relative(const sensor_msgs::msg::PointCloud2 &in, sensor_msgs::msg::PointCloud2 &out,
                                                       double cx, double cy, double radius, 
                                                       double min_height, double max_height)
    {
        int x_off, y_off, z_off, step;
        if (!get_xyz_fields(in, x_off, y_off, z_off, step)) { out = in; return; }
        
        // Filter by cylinder and Z-height (Z=0 is ground in base_footprint frame)
        const double r2 = radius * radius;
        out = in;
        out.data.clear();
        out.width = 0;
        out.height = 1;
        out.is_dense = false;
        std::vector<uint8_t> buffer;
        buffer.reserve(in.data.size());
        
        for (uint32_t r = 0; r < in.height; ++r)
        {
            const uint8_t *row = in.data.data() + r * in.row_step;
            for (uint32_t c = 0; c < in.width; ++c)
            {
                const uint8_t *pt = row + c * step;
                float x = read_float(pt + x_off);
                float y = read_float(pt + y_off);
                float z = read_float(pt + z_off);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                double dx = x - cx;
                double dy = y - cy;
                // Z is height from ground in base_footprint frame
                if (dx*dx + dy*dy <= r2 && z >= min_height && z <= max_height)
                {
                    buffer.insert(buffer.end(), pt, pt + step);
                    out.width++;
                }
            }
        }
        out.row_step = out.width * step;
        out.data = std::move(buffer);
    }
    
    void apply_euclidean_clustering(const sensor_msgs::msg::PointCloud2 &in, sensor_msgs::msg::PointCloud2 &out)
    {
        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Manual conversion from sensor_msgs to PCL
        int x_off, y_off, z_off, step;
        if (!get_xyz_fields(in, x_off, y_off, z_off, step))
        {
            out = in;
            return;
        }
        
        pcl_cloud->points.reserve(in.width * in.height);
        for (uint32_t r = 0; r < in.height; ++r)
        {
            const uint8_t *row = in.data.data() + r * in.row_step;
            for (uint32_t c = 0; c < in.width; ++c)
            {
                const uint8_t *pt = row + c * step;
                float x = read_float(pt + x_off);
                float y = read_float(pt + y_off);
                float z = read_float(pt + z_off);
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
                {
                    pcl_cloud->points.emplace_back(x, y, z);
                }
            }
        }
        pcl_cloud->width = pcl_cloud->points.size();
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = true;
        
        if (pcl_cloud->points.empty())
        {
            out = in;
            out.data.clear();
            out.width = 0;
            out.height = 1;
            out.row_step = 0;
            return;
        }
        
        // Create KdTree for search
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pcl_cloud);
        
        // Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(pcl_cloud->points.size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcl_cloud);
        ec.extract(cluster_indices);
        
        if (cluster_indices.empty())
        {
            // No clusters found - output empty cloud
            out = in;
            out.data.clear();
            out.width = 0;
            out.height = 1;
            out.row_step = 0;
            RCLCPP_DEBUG(this->get_logger(), "No clusters found (min_size=%d)", min_cluster_size_);
            return;
        }
        
        // Find largest cluster
        size_t max_size = 0;
        size_t max_idx = 0;
        for (size_t i = 0; i < cluster_indices.size(); ++i)
        {
            if (cluster_indices[i].indices.size() > max_size)
            {
                max_size = cluster_indices[i].indices.size();
                max_idx = i;
            }
        }
        
        // Extract largest cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster_indices[max_idx].indices)
        {
            cluster_cloud->points.push_back(pcl_cloud->points[idx]);
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        
        // Convert PCL back to ROS PointCloud2 (manual conversion)
        out = in;
        out.data.clear();
        out.width = cluster_cloud->points.size();
        out.height = 1;
        out.row_step = out.width * step;
        out.is_dense = true;
        
        std::vector<uint8_t> buffer;
        buffer.reserve(out.row_step);
        
        for (const auto &pt : cluster_cloud->points)
        {
            std::vector<uint8_t> pt_data(step, 0);
            std::memcpy(pt_data.data() + x_off, &pt.x, sizeof(float));
            std::memcpy(pt_data.data() + y_off, &pt.y, sizeof(float));
            std::memcpy(pt_data.data() + z_off, &pt.z, sizeof(float));
            buffer.insert(buffer.end(), pt_data.begin(), pt_data.end());
        }
        
        out.data = std::move(buffer);
        
        RCLCPP_DEBUG(this->get_logger(), "Found %zu clusters, largest has %zu points", 
                     cluster_indices.size(), max_size);
    }

    std::string input_topic_, input_frame_, output_frame_, output_topic_;
    std::string ground_frame_;  // base_footprint for ground reference
    double transform_timeout_{}, cylinder_z_min_{}, cylinder_z_max_{}, fixed_radius_{};
    bool enable_max_z_output_{};
    std::string max_z_topic_;
    bool debug_output_{}, publish_transform_info_{}, printed_tf_info_{}, log_stats_{};
    double stats_interval_sec_{};
    size_t received_count_{}, published_count_{};
    double forward_offset_{}, left_offset_{}, center_x_{}, center_y_{}, radius_{};
    bool have_cylinder_{false}, logged_cylinder_{false};
    bool use_marker_diameter_{false};
    double marker1_x_{}, marker1_y_{}, marker2_x_{}, marker2_y_{};
    bool use_dynamic_marker_{false};
    std::string robot_pose_topic_;
    double marker0_world_x_{}, marker0_world_y_{}, marker7_world_x_{}, marker7_world_y_{};
    double robot_pose_timeout_{};
    std::string pose_timeout_action_{"freeze"};
    bool have_robot_pose_{false};
    geometry_msgs::msg::PoseStamped latest_robot_pose_;
    std::mutex pose_mutex_;
    
    // Ground-relative and clustering parameters
    bool enable_ground_relative_{};
    double ground_relative_min_height_{}, ground_relative_max_height_{};
    bool enable_clustering_{};
    double cluster_tolerance_{};
    int min_cluster_size_{};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_max_z_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    void print_stats()
    {
        if (!log_stats_) return;
        RCLCPP_INFO(this->get_logger(), "stats: received=%zu published=%zu have_pose=%s",
                    received_count_, published_count_, have_robot_pose_ ? "true" : "false");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HesaiRegionFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

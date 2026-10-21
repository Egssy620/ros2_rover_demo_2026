#include "nedo_localization/projection_node.hpp"
#include "nedo_localization/projection_util.hpp"

namespace nedo_localization
{
    ProjectionNode::ProjectionNode(const rclcpp::NodeOptions &options) : Node("projection_node", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        initialize_ = false;
        // パラメータでフレーム名を設定可能にする（デフォルトはHesai QT128用）
        this->declare_parameter<std::string>("lidar_frame", "lidar_link");
        this->declare_parameter<std::string>("camera_frame", "camera");
        
        lidar_frame_name_ = this->get_parameter("lidar_frame").as_string();
        camera_frame_name_ = this->get_parameter("camera_frame").as_string();
        
        RCLCPP_INFO(this->get_logger(), "ProjectionNode initialized - Camera frame: %s, LiDAR frame: %s",
                    camera_frame_name_.c_str(), lidar_frame_name_.c_str());
        
        point_suber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "sync_point",
            rclcpp::SensorDataQoS(),
            std::bind(&ProjectionNode::point_callback, this, std::placeholders::_1));
        caminfo_suber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::SensorDataQoS(),
            std::bind(&ProjectionNode::caminfo_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<vehicle_interface::msg::ProjectedPoints>("projected", rclcpp::SensorDataQoS());
    }
    void ProjectionNode::caminfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo)
    {
        std::cerr << "[ProjectionNode] caminfo_callback triggered, width=" << caminfo->width 
                  << ", height=" << caminfo->height << std::endl;
        if (caminfo_.width == 0)
        { // when caminfo is empty.
            caminfo_ = *caminfo;
            RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d", caminfo_.width, caminfo_.height);
        }
    }

    void ProjectionNode::point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        std::cerr << "[ProjectionNode] point_callback triggered, points: " << cloud_msg->width * cloud_msg->height << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        pcl::fromROSMsg(*cloud_msg, latest_scan_);
        if (!initialize_)
        {
            try
            {
                // OpenCVのprojectPointsはワールド(LiDAR)→カメラへの変換が必要なので、lidar→cameraの順でlookup
                camera_lidar_transform_ = lookup_transform(lidar_frame_name_, camera_frame_name_);
                initialize_ = true;  // TFのルックアップが成功したら初期化完了
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Could not lookup transform from %s to %s: %s", 
                    camera_frame_name_.c_str(), lidar_frame_name_.c_str(), ex.what());
                return;
            }
        }
        else
        {
            // 既に初期化済みの場合も、TFを更新(動的なTF変更に対応)
            try
            {
                camera_lidar_transform_ = lookup_transform(lidar_frame_name_, camera_frame_name_);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Could not lookup transform from %s to %s: %s", 
                    lidar_frame_name_.c_str(), camera_frame_name_.c_str(), ex.what());
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        // RCLCPP_INFO(this->get_logger(),"took %ld milliseconds to lookup transform.",elapsed_time);
        std::cerr << "[ProjectionNode] Checks: latest_scan_.empty()=" << latest_scan_.empty() 
                  << ", caminfo_.width=" << caminfo_.width << std::endl;
        if (!latest_scan_.empty() && caminfo_.width != 0)
        {
            std::vector<Eigen::RowVectorXd> projected_points = project_image_coordinate(caminfo_, camera_lidar_transform_, latest_scan_, true);
            end = std::chrono::high_resolution_clock::now();
            elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // RCLCPP_INFO(this->get_logger(),"took %ld milliseconds to projection ",elapsed_time);
            auto pub_msg = create_projected_msg(projected_points);
            pub_msg.header = cloud_msg->header;
            publisher_->publish(pub_msg);
        }
        end = std::chrono::high_resolution_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        // RCLCPP_INFO(this->get_logger(),"took %ld milliseconds to projection projection callback.",elapsed_time);
    }
    geometry_msgs::msg::TransformStamped ProjectionNode::lookup_transform(const std::string &source_frame, const std::string &target_frame)
    {
        try
        {
            // tf_buffer_.lookupTransform(target, source, ...) returns a transform
            // that converts coordinates from 'source' into 'target'. The helper
            // API accepts (source_frame, target_frame) for readability, so call
            // lookupTransform with reversed arguments.
            return tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
            throw ex;
        }
    }
} // namespace nedo_localization

// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::ProjectionNode)
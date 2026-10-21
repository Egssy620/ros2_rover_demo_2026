#include "nedo_localization/lidar_cam_tf_node.hpp"
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
using json = nlohmann::json;

namespace nedo_localization
{
    LidarCamTF::LidarCamTF(const rclcpp::NodeOptions & options)
    : Node("cam_lidar_tf",options),tf_broadcaster_(this) {
        this->declare_parameter<std::string>("json_file_path", "");
        this->declare_parameter<std::string>("camera_frame", "camera");
        this->declare_parameter<std::string>("lidar_frame", "lidar_link");  // デフォルトはHesai用
        load_transform_from_json();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LidarCamTF::publish_tf, this));
    }

    void LidarCamTF::load_transform_from_json(){
        std::string json_file_path = this->get_parameter("json_file_path").as_string();
        RCLCPP_INFO(this->get_logger(), "Attempting to load calibration from: %s", json_file_path.c_str());
        
        if (json_file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "JSON file path is not set. Use the 'json_file_path' parameter.");
            rclcpp::shutdown();
            return;
        }

        // 相対パスの場合、パッケージのshareディレクトリを基準とした絶対パスに変換
        std::filesystem::path file_path(json_file_path);
        if (file_path.is_relative()) {
            try {
                std::string package_share_dir = ament_index_cpp::get_package_share_directory("nedo_bringup");
                file_path = std::filesystem::path(package_share_dir) / json_file_path;
                RCLCPP_INFO(this->get_logger(), "Converted relative path to absolute: %s", file_path.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not resolve package path, using as-is: %s", e.what());
            }
        }

        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file: %s", file_path.c_str());
            rclcpp::shutdown();
            return;
        }

        try {
            json j;
            file >> j;
            auto T_lidar_camera = j["results"]["T_lidar_camera"];

            if (T_lidar_camera.size() != 7) {
                RCLCPP_ERROR(this->get_logger(), "T_lidar_camera must have 7 elements: [x, y, z, qx, qy, qz, qw]");
                rclcpp::shutdown();
                return;
            }

            transform_.transform.translation.x = T_lidar_camera[0];
            transform_.transform.translation.y = T_lidar_camera[1];
            transform_.transform.translation.z = T_lidar_camera[2];
            transform_.transform.rotation.x = T_lidar_camera[3];
            transform_.transform.rotation.y = T_lidar_camera[4];
            transform_.transform.rotation.z = T_lidar_camera[5];
            transform_.transform.rotation.w = T_lidar_camera[6];

            transform_.header.frame_id = this->get_parameter("lidar_frame").as_string();  // パラメータから取得（Livox/Hesai対応）
            transform_.child_frame_id = this->get_parameter("camera_frame").as_string();
            
            RCLCPP_INFO(this->get_logger(), "Loaded camera-LiDAR calibration from: %s", json_file_path.c_str());
            RCLCPP_INFO(this->get_logger(), "  LiDAR frame (parent): %s", transform_.header.frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "  Camera frame (child): %s", transform_.child_frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "  Translation: [%.3f, %.3f, %.3f]",
                       transform_.transform.translation.x,
                       transform_.transform.translation.y,
                       transform_.transform.translation.z);
            RCLCPP_INFO(this->get_logger(), "  Rotation (quat): [%.3f, %.3f, %.3f, %.3f]",
                       transform_.transform.rotation.x,
                       transform_.transform.rotation.y,
                       transform_.transform.rotation.z,
                       transform_.transform.rotation.w);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing JSON file: %s", e.what());
            rclcpp::shutdown();
        }

    }

    void LidarCamTF::publish_tf(){
        transform_.header.stamp = this->get_clock()->now();
        tf_broadcaster_.sendTransform(transform_);
    }
}
// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::LidarCamTF)
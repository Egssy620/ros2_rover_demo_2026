#include "nedo_capture/caminfo_node.hpp"

namespace nedo_capture
{
    CaminfoNode::CaminfoNode(const rclcpp::NodeOptions & options)
    :Node("caminfo_node",options){
        std::string calibration_file = this->declare_parameter("calib_file","/home/oden/calibrationdata/ost.yaml");
        std::string camera_name = this->declare_parameter("camera_name","narrow_stereo");
        camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this,camera_name, calibration_file);
        if (!camera_info_manager_->isCalibrated())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load calibration file: %s", calibration_file.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Loaded calibration file: %s", calibration_file.c_str());
        }
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info",10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CaminfoNode::publish_caminfo,this)
        );
    }
    
    void CaminfoNode::publish_caminfo(){
        auto camera_info_msg = camera_info_manager_->getCameraInfo();
        camera_info_pub_->publish(camera_info_msg);
    }

}

// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_capture::CaminfoNode)

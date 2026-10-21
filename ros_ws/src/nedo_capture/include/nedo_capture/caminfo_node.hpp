#ifndef NEDO_CAPTURE__CAMINFO_NODE_HPP_
#define NEDO_CAPTURE__CAMINFO_NODE_HPP_

#include "nedo_capture/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
namespace nedo_capture
{
    class CaminfoNode :public rclcpp::Node
    {
        public:
            explicit CaminfoNode(const rclcpp::NodeOptions & options);
        
        private:
            void publish_caminfo();
            std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
            rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif // NEDO_CAPTURE__CAMINFO_NODE_HPP_
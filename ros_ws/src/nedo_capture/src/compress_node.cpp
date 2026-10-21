#include "nedo_capture/compress_node.hpp"

namespace nedo_capture
{
CompressNode::CompressNode(const rclcpp::NodeOptions& options)
:Node("compress_node",options){
    image_sub_ = image_transport::create_subscription(
        this,
        "debug_image",
        std::bind(&CompressNode::imageCallback,this,std::placeholders::_1),
        "raw",
        rmw_qos_profile_sensor_data  // ← ここを追加
    );
    
    image_pub_ = image_transport::create_publisher(
        this,
        "image_pub",
        rmw_qos_profile_sensor_data  // ← これがBEST_EFFORT
    );
    
}

void CompressNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
    image_pub_.publish(msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_capture::CompressNode)
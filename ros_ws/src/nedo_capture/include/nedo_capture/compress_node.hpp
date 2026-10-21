#ifndef NEDO_CAPTURE__COMPRESS_NODE_HPP_
#define NEDO_CAPTURE__COMPRESS_NODE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
namespace nedo_capture
{
    class CompressNode : public rclcpp::Node
    {
        public:
            explicit CompressNode(const rclcpp::NodeOptions & options);
        
        private:
            void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
            image_transport::Subscriber image_sub_;
            image_transport::Publisher image_pub_;
    };
} 

#endif //NEDO_CAPTURE__COMPRESS_NODE_HPP_
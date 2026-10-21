#ifndef NEDO_CAPTURE__NEDO_CAP_NODE_HPP_
#define NEDO_CAPTURE__NEDO_CAP_NODE_HPP_

#include "nedo_capture/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace nedo_capture
{

  class NedoCapNode : public rclcpp::Node
  {
    public:
      explicit NedoCapNode(const rclcpp::NodeOptions & options);
      virtual ~NedoCapNode();
    private:
      void timer_callback();
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
      rclcpp::TimerBase::SharedPtr timer_;
      std::string device;
      std::string frame_id;
      cv::VideoCapture cap;
  };

};  // namespace nedo_capture

#endif  // NEDO_CAPTURE__NEDO_CAP_NODE_HPP_

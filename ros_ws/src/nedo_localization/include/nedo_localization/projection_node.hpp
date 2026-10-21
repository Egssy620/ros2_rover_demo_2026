#ifndef NEDO_LOCALIZATION__PROJECTION_NODE_HPP_
#define NEDO_LOCALIZATION__PROJECTION_NODE_HPP_

#include "nedo_localization/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vehicle_interface/msg/projected_points.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cstring>

namespace nedo_localization
{
  class ProjectionNode : public rclcpp::Node
  {
    public :
      ProjectionNode(const rclcpp::NodeOptions & options);
    private:
      void point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
      void caminfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo);
      geometry_msgs::msg::TransformStamped lookup_transform(const std::string &target_frame,const std::string &source_frame);
      rclcpp::Publisher<vehicle_interface::msg::ProjectedPoints>::SharedPtr publisher_;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_suber_;
      rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_suber_;

      pcl::PointCloud<pcl::PointXYZ> latest_scan_;
      sensor_msgs::msg::CameraInfo caminfo_;
      std::string camera_frame_name_;
      std::string lidar_frame_name_;

      geometry_msgs::msg::TransformStamped camera_lidar_transform_;
      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;

      bool initialize_;
  };
}  // namespace nedo_localization

#endif  // NEDO_LOCALIZATION__PROJECTION_NODE_HPP_

#ifndef NEDO_LOCALIZATION__PROJECTION_UTIL_HPP
#define NEDO_LOCALIZATION__PROJECTION_UTIL_HPP

#include <Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vehicle_interface/msg/projected_points.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <opencv2/opencv.hpp>

namespace nedo_localization {

    std::vector<Eigen::RowVectorXd> project_image_coordinate(
        const sensor_msgs::msg::CameraInfo &caminfo,
        const geometry_msgs::msg::TransformStamped &cam_to_lidar,
        const pcl::PointCloud<pcl::PointXYZ> &pointcloud,
        bool opencv_ver
    );

    std::vector<Eigen::RowVectorXd> project_image_coordinate(
        const sensor_msgs::msg::CameraInfo &caminfo,
        const geometry_msgs::msg::TransformStamped &cam_to_lidar,
        const pcl::PointCloud<pcl::PointXYZ> &pointcloud
    );

    vehicle_interface::msg::ProjectedPoints create_projected_msg(
        const std::vector<Eigen::RowVectorXd> &projected_msg
    );

} // namespace nedo_localization

#endif // NEDO_LOCALIZATION__PROJECTION_UTIL_HPP
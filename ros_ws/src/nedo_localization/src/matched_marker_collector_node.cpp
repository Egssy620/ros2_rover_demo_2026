#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_interface/msg/pose_stamped_array.hpp"

class MatchedMarkerCollectorNode : public rclcpp::Node
{
public:
  MatchedMarkerCollectorNode()
  : Node("matched_marker_collector_node")
  {
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/matched_marker");
    collect_seconds_ = this->declare_parameter<double>("collect_seconds", 20.0);
    output_dir_path_ = this->declare_parameter<std::string>(
      "output_dir_path", "/root/NEDO_ROS/ros_ws/src/nedo_bringup/config");
    output_yaml_name_ = this->declare_parameter<std::string>(
      "output_yaml_name", "params.yaml");
    output_yaml_path_ = output_dir_path_ + "/" + output_yaml_name_;
    output_yaml_format_ = this->declare_parameter<std::string>("output_yaml_format", "params");
    min_samples_per_marker_ = this->declare_parameter<int>("min_samples_per_marker", 20);
    merge_with_existing_map_ = this->declare_parameter<bool>("merge_with_existing_map", true);
    use_robot_pose_for_merge_after_first_ = this->declare_parameter<bool>(
      "use_robot_pose_for_merge_after_first", true);
    robot_pose_topic_name_ = this->declare_parameter<std::string>("robot_pose_topic_name", "/robot_pose");
    session_output_dir_path_ = this->declare_parameter<std::string>(
      "session_output_dir_path", "/root/NEDO_ROS/bags/sessions");
    session_output_prefix_ = this->declare_parameter<std::string>(
      "session_output_prefix", "landmark_measurement");
    // marker6 を原点として地図を再センタリングする旧仕様 (Livox dual-LiDAR時代の名残)。
    // BIN方式 + OT128 単一センサ構成では不要なため、デフォルトで無効化。
    enable_recenter_ = this->declare_parameter<bool>("enable_recenter", false);

    start_time_ = this->now();

    sub_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
      topic_name_, rclcpp::SensorDataQoS(),
      std::bind(&MatchedMarkerCollectorNode::matchedMarkerCallback, this, std::placeholders::_1));

    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      robot_pose_topic_name_, rclcpp::SystemDefaultsQoS(),
      std::bind(&MatchedMarkerCollectorNode::robotPoseCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MatchedMarkerCollectorNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
        "Collecting %s for %.1f sec. Output: %s (format=%s, merge=%s, mode=%s, pose_topic=%s, session_dir=%s)",
      topic_name_.c_str(), collect_seconds_, output_yaml_path_.c_str(),
        output_yaml_format_.c_str(),
        merge_with_existing_map_ ? "true" : "false",
        use_robot_pose_for_merge_after_first_ ? "robot_pose" : "overlap",
        robot_pose_topic_name_.c_str(), session_output_dir_path_.c_str());
  }

private:
  struct MarkerStats
  {
    double sum_x {0.0};
    double sum_y {0.0};
    double sum_z {0.0};
    std::size_t count {0};
  };

  struct MarkerEstimate
  {
    double x {0.0};
    double y {0.0};
    double z {0.0};
    std::size_t sample_count {0};
  };

  struct Transform2D
  {
    double cos_yaw {1.0};
    double sin_yaw {0.0};
    double tx {0.0};
    double ty {0.0};
    double yaw {0.0};
  };

  struct RobotPoseAccumulator
  {
    double sum_x {0.0};
    double sum_y {0.0};
    double sum_sin_yaw {0.0};
    double sum_cos_yaw {0.0};
    std::size_t count {0};
  };

  void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (finished_) {
      return;
    }

    const double elapsed = (this->now() - start_time_).seconds();
    if (elapsed > collect_seconds_) {
      return;
    }

    latest_robot_pose_ = *msg;

    const auto & q = msg->pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    robot_pose_acc_.sum_x += msg->pose.position.x;
    robot_pose_acc_.sum_y += msg->pose.position.y;
    robot_pose_acc_.sum_sin_yaw += std::sin(yaw);
    robot_pose_acc_.sum_cos_yaw += std::cos(yaw);
    robot_pose_acc_.count += 1;
  }

  std::optional<geometry_msgs::msg::PoseStamped> buildAveragedRobotPose() const
  {
    if (robot_pose_acc_.count == 0) {
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped averaged = latest_robot_pose_;
    const double n = static_cast<double>(robot_pose_acc_.count);
    averaged.pose.position.x = robot_pose_acc_.sum_x / n;
    averaged.pose.position.y = robot_pose_acc_.sum_y / n;

    const double yaw = std::atan2(robot_pose_acc_.sum_sin_yaw, robot_pose_acc_.sum_cos_yaw);
    averaged.pose.position.z = 0.0;
    averaged.pose.orientation.x = 0.0;
    averaged.pose.orientation.y = 0.0;
    averaged.pose.orientation.z = std::sin(yaw * 0.5);
    averaged.pose.orientation.w = std::cos(yaw * 0.5);

    return averaged;
  }

  void matchedMarkerCallback(const vehicle_interface::msg::PoseStampedArray::SharedPtr msg)
  {
    if (finished_) {
      return;
    }

    const double elapsed = (this->now() - start_time_).seconds();
    if (elapsed > collect_seconds_) {
      return;
    }

    for (const auto & pose_stamped : msg->array) {
      const std::string marker_id = pose_stamped.header.frame_id;
      if (marker_id.empty()) {
        continue;
      }

      auto & stats = marker_stats_[marker_id];
      stats.sum_x += pose_stamped.pose.position.x;
      stats.sum_y += pose_stamped.pose.position.y;
      stats.sum_z += pose_stamped.pose.position.z;
      stats.count += 1;
      total_samples_ += 1;
    }
  }

  void timerCallback()
  {
    if (finished_) {
      return;
    }

    const double elapsed = (this->now() - start_time_).seconds();
    const double remaining = std::max(0.0, collect_seconds_ - elapsed);

    RCLCPP_INFO(
      this->get_logger(),
      "Collecting... %.1f sec remaining, markers: %zu, total samples: %zu",
      remaining, marker_stats_.size(), total_samples_);

    if (elapsed >= collect_seconds_) {
      finished_ = true;
      mergeAndWriteYaml();
      RCLCPP_INFO(this->get_logger(), "Finished collection. Shutting down.");
      rclcpp::shutdown();
    }
  }

  std::string nowStamp() const
  {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  std::vector<double> parseNumberList(const std::string & text) const
  {
    std::vector<double> values;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stod(token));
      } catch (...) {
        return {};
      }
    }
    return values;
  }

  std::map<std::string, MarkerEstimate> buildMeasurementAverages() const
  {
    std::map<std::string, MarkerEstimate> out;
    for (const auto & kv : marker_stats_) {
      if (static_cast<int>(kv.second.count) < min_samples_per_marker_) {
        continue;
      }
      const auto & s = kv.second;
      MarkerEstimate e;
      e.x = s.sum_x / static_cast<double>(s.count);
      e.y = s.sum_y / static_cast<double>(s.count);
      e.z = s.sum_z / static_cast<double>(s.count);
      e.sample_count = s.count;
      out[kv.first] = e;
    }
    return out;
  }

  bool loadExistingMergedMap(std::map<std::string, MarkerEstimate> & map_out) const
  {
    map_out.clear();
    if (!std::filesystem::exists(output_yaml_path_)) {
      return false;
    }

    std::ifstream ifs(output_yaml_path_);
    if (!ifs.is_open()) {
      return false;
    }

    std::string line;
    while (std::getline(ifs, line)) {
      const auto pos_key = line.find(".position:");
      if (pos_key != std::string::npos) {
        std::string trimmed = line;
        const auto first = trimmed.find_first_not_of(" \t");
        if (first == std::string::npos) {
          continue;
        }
        trimmed.erase(0, first);
        const auto name_end = trimmed.find(".position:");
        if (name_end == std::string::npos) {
          continue;
        }
        const std::string marker_id = trimmed.substr(0, name_end);
        if (marker_id.find('.') != std::string::npos || marker_id.empty()) {
          continue;
        }
        const auto lb = trimmed.find('[');
        const auto rb = trimmed.find(']');
        if (lb == std::string::npos || rb == std::string::npos || rb <= lb + 1) {
          continue;
        }
        const auto values = parseNumberList(trimmed.substr(lb + 1, rb - lb - 1));
        if (values.size() < 2) {
          continue;
        }
        auto & e = map_out[marker_id];
        e.x = values[0];
        e.y = values[1];
        e.z = values.size() >= 3 ? values[2] : 0.0;
        if (e.sample_count == 0) {
          e.sample_count = 1;
        }
        continue;
      }

      const auto count_key = line.find(".sample_count:");
      if (count_key != std::string::npos) {
        std::string trimmed = line;
        const auto first = trimmed.find_first_not_of(" \t");
        if (first == std::string::npos) {
          continue;
        }
        trimmed.erase(0, first);
        const auto name_end = trimmed.find(".sample_count:");
        if (name_end == std::string::npos) {
          continue;
        }
        const std::string marker_id = trimmed.substr(0, name_end);
        if (marker_id.find('.') != std::string::npos || marker_id.empty()) {
          continue;
        }
        const auto colon = trimmed.find(':', name_end);
        if (colon == std::string::npos || colon + 1 >= trimmed.size()) {
          continue;
        }
        try {
          const auto c = static_cast<std::size_t>(std::stoul(trimmed.substr(colon + 1)));
          map_out[marker_id].sample_count = std::max<std::size_t>(1, c);
        } catch (...) {
          // Ignore malformed sample count.
        }
      }
    }

    return !map_out.empty();
  }

  std::optional<Transform2D> estimateTransformByOverlap(
    const std::map<std::string, MarkerEstimate> & measurement,
    const std::map<std::string, MarkerEstimate> & merged) const
  {
    std::vector<std::string> overlap_ids;
    overlap_ids.reserve(measurement.size());
    for (const auto & kv : measurement) {
      if (merged.find(kv.first) != merged.end()) {
        overlap_ids.push_back(kv.first);
      }
    }

    if (overlap_ids.size() < 2) {
      return std::nullopt;
    }

    double cx_m = 0.0;
    double cy_m = 0.0;
    double cx_w = 0.0;
    double cy_w = 0.0;
    for (const auto & id : overlap_ids) {
      cx_m += measurement.at(id).x;
      cy_m += measurement.at(id).y;
      cx_w += merged.at(id).x;
      cy_w += merged.at(id).y;
    }
    const double n = static_cast<double>(overlap_ids.size());
    cx_m /= n;
    cy_m /= n;
    cx_w /= n;
    cy_w /= n;

    double sxx = 0.0;
    double sxy = 0.0;
    for (const auto & id : overlap_ids) {
      const double mx = measurement.at(id).x - cx_m;
      const double my = measurement.at(id).y - cy_m;
      const double wx = merged.at(id).x - cx_w;
      const double wy = merged.at(id).y - cy_w;
      sxx += mx * wx + my * wy;
      sxy += mx * wy - my * wx;
    }

    const double yaw = std::atan2(sxy, sxx);
    Transform2D tf;
    tf.cos_yaw = std::cos(yaw);
    tf.sin_yaw = std::sin(yaw);
    tf.yaw = yaw;
    tf.tx = cx_w - (tf.cos_yaw * cx_m - tf.sin_yaw * cy_m);
    tf.ty = cy_w - (tf.sin_yaw * cx_m + tf.cos_yaw * cy_m);
    return tf;
  }

  MarkerEstimate applyTransform(const MarkerEstimate & m, const Transform2D & tf) const
  {
    MarkerEstimate out = m;
    out.x = tf.cos_yaw * m.x - tf.sin_yaw * m.y + tf.tx;
    out.y = tf.sin_yaw * m.x + tf.cos_yaw * m.y + tf.ty;
    return out;
  }

  Transform2D transformFromRobotPose(const geometry_msgs::msg::PoseStamped & pose) const
  {
    Transform2D tf;
    tf.tx = pose.pose.position.x;
    tf.ty = pose.pose.position.y;

    const auto & q = pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    tf.yaw = std::atan2(siny_cosp, cosy_cosp);
    tf.cos_yaw = std::cos(tf.yaw);
    tf.sin_yaw = std::sin(tf.yaw);
    return tf;
  }

  bool recenterMapByMarker6(std::map<std::string, MarkerEstimate> & map_data) const
  {
    const auto it = map_data.find("marker6");
    if (it == map_data.end()) {
      return false;
    }

    const double ref_x = it->second.x;
    const double ref_y = it->second.y;
    for (auto & kv : map_data) {
      kv.second.x -= ref_x;
      kv.second.y -= ref_y;
    }
    return true;
  }

  void writeMergedMapYaml(const std::map<std::string, MarkerEstimate> & merged)
  {
    const bool debug_format = (output_yaml_format_ == "debug");
    if (!debug_format && output_yaml_format_ != "params") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown output_yaml_format='%s'. Falling back to params format.",
        output_yaml_format_.c_str());
    }

    std::map<std::string, MarkerEstimate> merged_for_output = merged;
    if (enable_recenter_) {
      if (!recenterMapByMarker6(merged_for_output)) {
        RCLCPP_WARN(
          this->get_logger(),
          "marker6 is not present in merged map. Output is written without recentering.");
      }
    }

    std::vector<std::string> ids;
    ids.reserve(merged_for_output.size());
    for (const auto & kv : merged_for_output) {
      ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());

    std::ofstream ofs(output_yaml_path_);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_yaml_path_.c_str());
      return;
    }

    ofs << std::fixed << std::setprecision(6);

    if (debug_format) {
      ofs << "/**:\n";
      ofs << "  ros__parameters:\n";

      ofs << "    landmark_ids: [";
      for (std::size_t i = 0; i < ids.size(); ++i) {
        ofs << '"' << ids[i] << '"';
        if (i + 1 < ids.size()) {
          ofs << ", ";
        }
      }
      ofs << "]\n";

      for (const auto & id : ids) {
        const auto & e = merged_for_output.at(id);
        ofs << "    " << id << ".position: [" << e.x << ", " << e.y << ", " << e.z << "]\n";
        ofs << "    " << id << ".sample_count: " << e.sample_count << "\n";
      }

      ofs << "\n";
      ofs << "    LM_ids: [";
      for (std::size_t i = 0; i < ids.size(); ++i) {
        ofs << '"' << ids[i] << '"';
        if (i + 1 < ids.size()) {
          ofs << ", ";
        }
      }
      ofs << "]\n";

      ofs << "    LM_xs: [";
      for (std::size_t i = 0; i < ids.size(); ++i) {
        const auto & e = merged_for_output.at(ids[i]);
        ofs << e.x;
        if (i + 1 < ids.size()) {
          ofs << ", ";
        }
      }
      ofs << "]\n";

      ofs << "    LM_ys: [";
      for (std::size_t i = 0; i < ids.size(); ++i) {
        const auto & e = merged_for_output.at(ids[i]);
        ofs << e.y;
        if (i + 1 < ids.size()) {
          ofs << ", ";
        }
      }
      ofs << "]\n";

      return;
    }

    ofs << "/**:\n";
    ofs << "  ros__parameters:\n";
    ofs << "    LM_ids: [";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      ofs << '"' << ids[i] << '"';
      if (i + 1 < ids.size()) {
        ofs << ", ";
      }
    }
    ofs << "]\n";

    ofs << "    LM_xs: [";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const auto & e = merged_for_output.at(ids[i]);
      ofs << e.x;
      if (i + 1 < ids.size()) {
        ofs << ", ";
      }
    }
    ofs << "]\n";

    ofs << "    LM_ys: [";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const auto & e = merged_for_output.at(ids[i]);
      ofs << e.y;
      if (i + 1 < ids.size()) {
        ofs << ", ";
      }
    }
    ofs << "]\n";
  }

  void writeSessionYaml(
    const std::string & session_id,
    const std::map<std::string, MarkerEstimate> & measurement,
    const Transform2D & robot_pose_in_map,
    std::size_t overlap_count) const
  {
    std::filesystem::create_directories(session_output_dir_path_);
    const std::string path =
      session_output_dir_path_ + "/" + session_output_prefix_ + "_" + session_id + ".yaml";

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open session output file: %s", path.c_str());
      return;
    }

    std::map<std::string, MarkerEstimate> measurement_for_output = measurement;
    if (enable_recenter_) {
      if (!recenterMapByMarker6(measurement_for_output)) {
        RCLCPP_WARN(
          this->get_logger(),
          "marker6 is not present in session measurement. Session output is written without recentering.");
      }
    }

    std::vector<std::string> ids;
    ids.reserve(measurement_for_output.size());
    for (const auto & kv : measurement_for_output) {
      ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());

    ofs << "/**:\n";
    ofs << "  ros__parameters:\n";
    ofs << "    session_id: \"" << session_id << "\"\n";
    ofs << "    overlap_marker_count: " << overlap_count << "\n";
    ofs << std::fixed << std::setprecision(6);
    ofs << "    robot_pose_in_map: ["
        << robot_pose_in_map.tx << ", " << robot_pose_in_map.ty << ", 0.0, " << robot_pose_in_map.yaw
        << "]\n";

    ofs << "    marker_ids: [";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      ofs << '"' << ids[i] << '"';
      if (i + 1 < ids.size()) {
        ofs << ", ";
      }
    }
    ofs << "]\n";

    for (const auto & id : ids) {
      const auto & e = measurement_for_output.at(id);
      ofs << "    " << id << ".position_in_robot: [" << e.x << ", " << e.y << ", " << e.z << "]\n";
      ofs << "    " << id << ".sample_count: " << e.sample_count << "\n";
    }

    RCLCPP_INFO(this->get_logger(), "Wrote session result: %s", path.c_str());
  }

  void mergeAndWriteYaml()
  {
    const auto measurement = buildMeasurementAverages();
    if (measurement.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid marker data to write.");
      return;
    }

    std::filesystem::create_directories(output_dir_path_);

    std::map<std::string, MarkerEstimate> merged;
    const bool has_existing = merge_with_existing_map_ && loadExistingMergedMap(merged);

    Transform2D tf;
    bool tf_ready = false;
    bool pose_based_merge = false;
    std::size_t overlap_count = 0;
    if (!has_existing) {
      merged = measurement;
      RCLCPP_INFO(this->get_logger(), "No existing map found. Initialized merged map from current measurement.");
    } else {
      if (use_robot_pose_for_merge_after_first_) {
        const auto averaged_robot_pose = buildAveragedRobotPose();
        if (!averaged_robot_pose.has_value()) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed to merge: robot pose average is not available on %s.",
            robot_pose_topic_name_.c_str());
          return;
        }
        tf = transformFromRobotPose(averaged_robot_pose.value());
        tf_ready = true;
        pose_based_merge = true;

        RCLCPP_INFO(
          this->get_logger(),
          "Using averaged robot pose samples: %zu",
          robot_pose_acc_.count);
      } else {
        for (const auto & kv : measurement) {
          if (merged.find(kv.first) != merged.end()) {
            overlap_count += 1;
          }
        }

        auto tf_opt = estimateTransformByOverlap(measurement, merged);
        if (!tf_opt.has_value()) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed to merge: overlap markers are %zu (need >= 2 with existing map).",
            overlap_count);
          return;
        }
        tf = tf_opt.value();
        tf_ready = true;
      }

      if (!tf_ready) {
        RCLCPP_ERROR(this->get_logger(), "Failed to merge: transform is not ready.");
        return;
      }

      for (const auto & kv : measurement) {
        const auto transformed = applyTransform(kv.second, tf);
        auto it = merged.find(kv.first);
        if (it == merged.end()) {
          merged[kv.first] = transformed;
          continue;
        }

        const auto old_n = std::max<std::size_t>(1, it->second.sample_count);
        const auto new_n = std::max<std::size_t>(1, transformed.sample_count);
        const auto sum_n = old_n + new_n;

        it->second.x = (it->second.x * static_cast<double>(old_n) +
                        transformed.x * static_cast<double>(new_n)) /
                       static_cast<double>(sum_n);
        it->second.y = (it->second.y * static_cast<double>(old_n) +
                        transformed.y * static_cast<double>(new_n)) /
                       static_cast<double>(sum_n);
        it->second.z = (it->second.z * static_cast<double>(old_n) +
                        transformed.z * static_cast<double>(new_n)) /
                       static_cast<double>(sum_n);
        it->second.sample_count = sum_n;
      }
    }

    writeMergedMapYaml(merged);

    const std::string session_id = nowStamp();
    writeSessionYaml(session_id, measurement, tf, overlap_count);

    RCLCPP_INFO(
      this->get_logger(),
      "Merged map updated: markers=%zu, current measurement markers=%zu, overlap=%zu, mode=%s",
      merged.size(), measurement.size(), overlap_count,
      pose_based_merge ? "robot_pose" : (has_existing ? "overlap" : "initialize"));
  }

  std::string topic_name_;
  double collect_seconds_ {20.0};
  std::string output_yaml_name_;
  std::string output_dir_path_;
  std::string output_yaml_path_;
  std::string output_yaml_format_;
  std::string session_output_dir_path_;
  std::string session_output_prefix_;
  std::string robot_pose_topic_name_;
  int min_samples_per_marker_ {5};
  bool merge_with_existing_map_ {true};
  bool use_robot_pose_for_merge_after_first_ {true};
  bool enable_recenter_ {false};

  rclcpp::Time start_time_;
  bool finished_ {false};
  std::size_t total_samples_ {0};
  geometry_msgs::msg::PoseStamped latest_robot_pose_;
  RobotPoseAccumulator robot_pose_acc_;

  std::map<std::string, MarkerStats> marker_stats_;

  rclcpp::Subscription<vehicle_interface::msg::PoseStampedArray>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MatchedMarkerCollectorNode>());
  rclcpp::shutdown();
  return 0;
}

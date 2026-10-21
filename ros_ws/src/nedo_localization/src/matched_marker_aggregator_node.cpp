#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interface/msg/pose_stamped_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MatchedMarkerAggregatorNode : public rclcpp::Node
{
public:
  MatchedMarkerAggregatorNode()
  : Node("matched_marker_aggregator_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    front_topic_ = this->declare_parameter<std::string>("front_topic", "/matched_marker_front");
    back_topic_ = this->declare_parameter<std::string>("back_topic", "/matched_marker_back");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/matched_marker");
    merge_window_ms_ = this->declare_parameter<int>("merge_window_ms", 50);
    timer_period_ms_ = this->declare_parameter<int>("timer_period_ms", 10);

    const auto qos = rclcpp::SensorDataQoS();
    pub_ = this->create_publisher<vehicle_interface::msg::PoseStampedArray>(output_topic_, qos);

    front_sub_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
      front_topic_, qos,
      std::bind(&MatchedMarkerAggregatorNode::frontCallback, this, std::placeholders::_1));

    back_sub_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
      back_topic_, qos,
      std::bind(&MatchedMarkerAggregatorNode::backCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_),
      std::bind(&MatchedMarkerAggregatorNode::onTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Matched marker aggregator started: front=%s back=%s out=%s",
      front_topic_.c_str(), back_topic_.c_str(), output_topic_.c_str());
  }

private:
  struct StampedMsg
  {
    vehicle_interface::msg::PoseStampedArray msg;
    rclcpp::Time msg_stamp;
    rclcpp::Time received_time;
  };

  void frontCallback(const vehicle_interface::msg::PoseStampedArray::SharedPtr msg)
  {
    latest_front_.msg = *msg;
    latest_front_.msg_stamp = resolveMessageStamp(*msg);
    latest_front_.received_time = this->now();
    has_front_ = true;
    tryPublishMergedIfReady();
  }

  void backCallback(const vehicle_interface::msg::PoseStampedArray::SharedPtr msg)
  {
    latest_back_.msg = *msg;
    latest_back_.msg_stamp = resolveMessageStamp(*msg);
    latest_back_.received_time = this->now();
    has_back_ = true;
    tryPublishMergedIfReady();
  }

  void onTimer()
  {
    const auto now = this->now();
    if (has_front_ && has_back_ && withinMergeWindow(latest_front_.msg_stamp, latest_back_.msg_stamp)) {
      publishCombined(true, true, "merged_by_timer");
      has_front_ = false;
      has_back_ = false;
      return;
    }
    if (has_front_ && (now - latest_front_.received_time).nanoseconds() >= mergeWindowNs()) {
      publishCombined(true, false, "front_timeout");
      has_front_ = false;
    }
    if (has_back_ && (now - latest_back_.received_time).nanoseconds() >= mergeWindowNs()) {
      publishCombined(false, true, "back_timeout");
      has_back_ = false;
    }
  }

  void tryPublishMergedIfReady()
  {
    if (!has_front_ || !has_back_) return;
    if (withinMergeWindow(latest_front_.msg_stamp, latest_back_.msg_stamp)) {
      publishCombined(true, true, "merged_on_arrival");
      has_front_ = false;
      has_back_ = false;
    }
  }

  bool withinMergeWindow(const rclcpp::Time & a, const rclcpp::Time & b) const
  {
    return std::llabs((a - b).nanoseconds()) <= mergeWindowNs();
  }

  int64_t mergeWindowNs() const { return static_cast<int64_t>(merge_window_ms_) * 1000000LL; }

  rclcpp::Time resolveMessageStamp(const vehicle_interface::msg::PoseStampedArray & msg) const
  {
    /* 
    // 個別のマーカーのタイムスタンプを確認する厳密な処理（一旦コメントアウト中）
    rclcpp::Time best(0, 0, this->get_clock()->get_clock_type());

    if (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0) {
      best = rclcpp::Time(msg.header.stamp);
    }

    for (const auto & pose : msg.array) {
      if (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) {
        continue;
      }
      const rclcpp::Time pose_time(pose.header.stamp);
      if (best.nanoseconds() == 0 || pose_time > best) {
        best = pose_time;
      }
    }

    if (best.nanoseconds() == 0) {
      return this->now();
    }
    return best;
    */

    // 配列のヘッダー時刻のみを使用する簡易版
    if (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0) {
      return rclcpp::Time(msg.header.stamp);
    }
    return this->now();
  }

  rclcpp::Time resolvePoseStamp(
    const geometry_msgs::msg::PoseStamped & pose,
    const rclcpp::Time & fallback) const
  {
    /*
    // 個別のマーカー時刻情報を使う厳密な処理（一旦コメントアウト中）
    if (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) {
      return fallback;
    }
    return rclcpp::Time(pose.header.stamp);
    */

    // 常に配列全体の代表時刻（fallback）を使用する
    (void)pose; // 未使用警告を回避
    return fallback;
  }

  void publishCombined(bool include_front, bool include_back, const char * reason)
  {
    std::map<std::string, geometry_msgs::msg::PoseStamped> merged_by_id;
    std::map<std::string, rclcpp::Time> stamp_by_id;

    auto merge_from = [&](const StampedMsg & source) {
      std::string source_frame = source.msg.header.frame_id;
      if(source_frame.empty() || source_frame == "map") {
         RCLCPP_WARN(this->get_logger(), "Incoming marker array has empty or 'map' frame_id! Transformations might fail.");
      }

      geometry_msgs::msg::TransformStamped transform;
      bool can_transform = true;
      try {
        if (!source_frame.empty() && source_frame != "base_link") {
          // タイムアウトをゼロ（待機しない）に修正し、処理のブロックを防ぐ
          transform = tf_buffer_.lookupTransform("base_link", source_frame, tf2::TimePointZero);
        }
      } catch(const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
           "Failed to lookup transform from %s to base_link: %s", source_frame.c_str(), ex.what());
        can_transform = false;
      }

      for (const auto & pose : source.msg.array) {
        if (pose.header.frame_id.empty()) continue;

        geometry_msgs::msg::PoseStamped transformed_pose = pose;
        if (can_transform && source_frame != "base_link") {
           // Transform position
           geometry_msgs::msg::PoseStamped raw_pose;
           raw_pose.pose = pose.pose;
           tf2::doTransform(raw_pose, transformed_pose, transform);
           // Restore the marker id in frame_id, as localizer uses it to identify the marker!
           transformed_pose.header.frame_id = pose.header.frame_id;
           transformed_pose.header.stamp = pose.header.stamp;
        }

        const auto stamp = resolvePoseStamp(pose, source.msg_stamp);
        const auto it = stamp_by_id.find(pose.header.frame_id);

        if (it == stamp_by_id.end() || stamp > it->second) {
          merged_by_id[pose.header.frame_id] = transformed_pose;
          stamp_by_id[pose.header.frame_id] = stamp;
        }
      }
    };

    if (include_front) merge_from(latest_front_);
    if (include_back) merge_from(latest_back_);

    if (merged_by_id.empty()) return;

    vehicle_interface::msg::PoseStampedArray out;
    out.array.reserve(merged_by_id.size());
    rclcpp::Time newest(0, 0, this->get_clock()->get_clock_type());

    for (const auto & kv : merged_by_id) {
      out.array.push_back(kv.second);
      if (stamp_by_id[kv.first] > newest) newest = stamp_by_id[kv.first];
    }

    out.header.frame_id = "base_link"; // Successfully transformed to base_link
    out.header.stamp = newest.nanoseconds() == 0 ? this->now() : newest;
    pub_->publish(out);
  }

  std::string front_topic_, back_topic_, output_topic_;
  int merge_window_ms_, timer_period_ms_;
  rclcpp::Publisher<vehicle_interface::msg::PoseStampedArray>::SharedPtr pub_;
  rclcpp::Subscription<vehicle_interface::msg::PoseStampedArray>::SharedPtr front_sub_, back_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  StampedMsg latest_front_, latest_back_;
  bool has_front_{false}, has_back_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MatchedMarkerAggregatorNode>());
  rclcpp::shutdown();
  return 0;
}

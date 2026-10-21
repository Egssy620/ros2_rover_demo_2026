// manage_node/src/manage_node_master.cpp
// 自律移動の全体管理ノード (マスター接続版)
//   manage_node.cpp (シャトル運転) との違い:
//   - 目的地は goals の往復ではなく、マスターから /goal_pose (PoseStamped, map 系) で1つずつ受け取る
//   - 着いたら目的地の向き (/goal_pose の向き) を向いて止まる (次の /goal_pose を待つ)
//   - 目的地のそばが塞がっていたら、次へ飛ばさずに止まる (次の /goal_pose を待つ)
//   - 結果は /navigation/result (std_msgs/String) でマスターに返す。simple_goal_follower_node と同じ名前・同じ文字列:
//       "planned" = 目的地を受け取り経路を作った / "arrived" = 着いた / "blocked" = 塞がっていて止まった
//   - ゴール巡回を前提にした障害物チェック (フルスキャン)・task_node 連携は外した

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <deque>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <ctime>
#include <fstream>
#include <cstdlib>

static inline double wrapToPi(double a){
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

enum class Phase {
    IDLE,             // 目的地待ち (起動直後・到着後・塞がり停止後)
    HEADING,          // ゴール方向へ向く
    DRIVE,            // 前進
    HEADING_TO_GOAL,  // 目的地の向きに向く
};

class ManageNodeMaster : public rclcpp::Node {
public:
  ManageNodeMaster() : Node("manage_node_master")
  {
    // ログディレクトリ作成
    log_dir_ = makeLogDirectory();
    std::string pose_path = log_dir_ + "/pose_log.csv";
    log_pose_file_.open(pose_path, std::ios::out);
    log_pose_file_ << "time,x,y,yaw\n";
    RCLCPP_INFO(this->get_logger(), "Pose log saved to: %s", pose_path.c_str());

    // パラメータ (manage_node と同じ)
    lookahead_dist_ = this->declare_parameter("lookahead_distance", 0.5);  // LookAhead距離
    lidar_history_sec_ = this->declare_parameter<double>("lidar_history_sec", 3.0);  // 過去点群の保持期間 [s]
    robot_radius_           = this->declare_parameter<double>("robot_radius", 0.2);            // ローバ半径 (最近点除外閾値)
    obstacle_avoid_radius_  = this->declare_parameter<double>("obstacle_avoid_radius", 0.5);   // 障害物回避円 r2
    path_band_half_width_   = this->declare_parameter<double>("path_band_half_width", 0.2);    // 経路バンド片側幅
    voicevox_url_           = this->declare_parameter<std::string>("voicevox_url", "http://localhost:50021");  // 空文字で無効

    //  仮自己位置
    car_pos_ = {0.0, 0.0};
    car_yaw_ = 0.0;
    received_pose_ = false;

    // Publisher
    {
      rclcpp::QoS latched(1);
      latched.transient_local();
      heading_target_pub_ = this->create_publisher<std_msgs::msg::Float32>("/heading_target", latched);
    }
    target_pub_         = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_point", 10);
    cmd_source_pub_     = this->create_publisher<std_msgs::msg::String>("/cmd_source", 10);
    result_pub_         = this->create_publisher<std_msgs::msg::String>("/navigation/result", 10);

    {
      rclcpp::QoS latched(1);
      latched.transient_local();
      goal_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/current_goal", latched);
    }

    // Subscriber
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&ManageNodeMaster::onGoalPose, this, std::placeholders::_1));

    pose2d_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/robot_pose2d", 10,
      std::bind(&ManageNodeMaster::onPose2D, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/filtered_points_shuttle", 10,
      std::bind(&ManageNodeMaster::onLidar, this, std::placeholders::_1));

    marker_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco_markers", 10,
      std::bind(&ManageNodeMaster::onMarkerArray, this, std::placeholders::_1));

    marker_id_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/aruco_marker_ids", 10,
      std::bind(&ManageNodeMaster::onMarkerIDs, this, std::placeholders::_1));

    heading_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/heading_done", 10,
      std::bind(&ManageNodeMaster::onHeadingDone, this, std::placeholders::_1));

    drive_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/drive_done", 10,
      std::bind(&ManageNodeMaster::onDriveDone, this, std::placeholders::_1));

    // TF buffer (os_gravity -> odom 変換用)
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //  Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ManageNodeMaster::onTimer, this));

    // 初期フェーズ: 目的地が来るまで動かない
    phase_ = Phase::IDLE;

    std_msgs::msg::String src;
    src.data = "none";
    cmd_source_pub_->publish(src);

    RCLCPP_INFO(this->get_logger(), "準備できました。マスターから目的地が届くのを待ちます");
  }

  ~ManageNodeMaster(){
    if (log_pose_file_.is_open())
      log_pose_file_.close();
  }

private:
  // ============================================================
  //                      VOICEVOX スピーカ
  // ============================================================
  std::string makeLogDirectory()
  {
    const char* home = std::getenv("HOME");
    if (!home) home = "/tmp"; // fallback

    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << home << "/ros2_logs/" << std::put_time(&tm, "%Y-%m-%d");
    std::string dir_path = oss.str();

    std::filesystem::create_directories(dir_path);
    return dir_path;
  }

  void playVoice(const std::string &text)
  {
    if (voicevox_url_.empty()) {
      RCLCPP_INFO(this->get_logger(), "[VOICE disabled] %s", text.c_str());
      return;
    }

    auto url_encode = [&](const std::string &s) {
      std::ostringstream encoded;
      encoded << std::hex << std::uppercase;
      for (unsigned char c : s) {
        if (isalnum(c) || c=='-'||c=='_'||c=='.'||c=='~')
          encoded << c;
        else
          encoded << '%' << std::setw(2) << int(c);
      }
      return encoded.str();
    };

    std::string encoded_text = url_encode(text);
    std::string cmd =
      "curl -s -X POST '" + voicevox_url_ + "/audio_query?text=" + encoded_text +
      "&speaker=3' "
      "| curl -s -H 'Content-Type: application/json' -X POST -d @- "
      "'" + voicevox_url_ + "/synthesis?speaker=3' "
      "-o /tmp/voice.wav && aplay -q /tmp/voice.wav";

    RCLCPP_INFO(this->get_logger(), "[VOICE] %s", text.c_str());
    std::system(cmd.c_str());
  }

  // ============================================================
  //                  マスターへの結果
  // ============================================================
  void publishResult(const std::string &result)
  {
    std_msgs::msg::String msg;
    msg.data = result;
    result_pub_->publish(msg);
  }

  // ============================================================
  //                  目的地受信 (マスターから)
  // ============================================================
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
      RCLCPP_WARN(this->get_logger(),
                  "目的地が map 以外の座標 (%s) で届きました。map の座標として扱います",
                  msg->header.frame_id.c_str());
    }

    const auto &q = msg->pose.orientation;
    goal_ = cv::Point2d(msg->pose.position.x, msg->pose.position.y);
    goal_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    has_goal_ = true;
    have_last_selected_ = false;

    RCLCPP_INFO(this->get_logger(),
                "目的地 (%.2f, %.2f, %.0f°) が届きました。向きを合わせてから進みます",
                goal_.x, goal_.y, goal_yaw_ * 180.0 / M_PI);

    // 経路生成と HEADING 開始 (自己位置がまだなら onPose2D で向きを送る)
    if (received_pose_) {
      updatePathToCurrentGoal();
      std_msgs::msg::Float32 msg_target;
      msg_target.data = std::atan2(goal_.y - car_pos_.y, goal_.x - car_pos_.x);
      heading_target_pub_->publish(msg_target);
      heading_sent_ = true;
    }

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING;
    publishResult("planned");
  }

  // ============================================================
  //                  ゴール近傍に障害物があるか
  // ============================================================
  bool isGoalBlocked()
  {
    if (!received_pose_) return false;
    if (lidar_points_.empty()) return false;

    const double R_GOAL   = 0.15; // ゴール周囲半径
    const int    MIN_PTS  = 120;   // 最低点数

    cv::Point2d g = goal_;
    int cnt = 0;

    for (auto &p : lidar_points_) {
      double d_goal = cv::norm(p - g);
      if (d_goal < R_GOAL) cnt++;
    }

    if (cnt >= MIN_PTS) {
      RCLCPP_WARN(this->get_logger(),
                  "Goal blocked: %d points within %.2f m", cnt, R_GOAL);
      return true;
    }
    return false;
  }

  // ============================================================
  //                  ゴールが塞がっていたら止まる
  // ============================================================
  void stopAtBlockedGoal()
  {
    std_msgs::msg::String stop_src;
    stop_src.data = "none";
    cmd_source_pub_->publish(stop_src);

    phase_ = Phase::IDLE;
    publishResult("blocked");

    RCLCPP_WARN(this->get_logger(),
                "目的地のそばに障害物があるので止まりました。次の目的地を待ちます");
    playVoice("目的地の付近に障害物があります。停止します。");
  }

  // ============================================================
  //                        Heading Done
  // ============================================================
  void onHeadingDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;

    if (phase_ == Phase::HEADING_TO_GOAL) {
      // 到着: 止まってマスターに知らせる
      std_msgs::msg::String src;
      src.data = "none";
      cmd_source_pub_->publish(src);

      phase_ = Phase::IDLE;
      publishResult("arrived");

      RCLCPP_INFO(this->get_logger(), "目的地に着きました。次の目的地を待ちます");
      return;
    }

    if (phase_ == Phase::HEADING) {
      // 経路生成
      updatePathToCurrentGoal();
      heading_sent_ = false;

      std_msgs::msg::String src;
      src.data = "drive";
      cmd_source_pub_->publish(src);
      phase_ = Phase::DRIVE;
      return;
    }
  }

  // ============================================================
  //                        Drive Done
  // ============================================================
  void onDriveDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;
    if (phase_ != Phase::DRIVE) return;

    // 目的地の向きに合わせる
    std_msgs::msg::Float32 msg_yaw;
    msg_yaw.data = goal_yaw_;
    heading_target_pub_->publish(msg_yaw);

    std_msgs::msg::String src;
    src.data = "heading";
    cmd_source_pub_->publish(src);

    phase_ = Phase::HEADING_TO_GOAL;
  }

  // ============================================================
  //                         Pose 受信
  // ============================================================
  void onPose2D(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    car_pos_ = cv::Point2d(msg->x, msg->y);
    car_yaw_ = wrapToPi(msg->theta);

    // 初回受信処理
    if (!received_pose_) {
        received_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "初回Pose受信");
        if (has_goal_) updatePathToCurrentGoal();
    }

    trajectory_.push_back(car_pos_);
    if (trajectory_.size() > 200)
      trajectory_.erase(trajectory_.begin());

    // 自己位置より先に目的地が届いていた場合の、最初の heading_target
    if (phase_ == Phase::HEADING && !heading_sent_) {
      double dx = goal_.x - car_pos_.x;
      double dy = goal_.y - car_pos_.y;
      double theta_target = std::atan2(dy, dx);

      std_msgs::msg::Float32 msg_target;
      msg_target.data = theta_target;
      heading_target_pub_->publish(msg_target);

      heading_sent_ = true;
    }
  }

  // ============================================================
  //                       Marker / ID
  // ============================================================
  void onMarkerArray(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    marker_points_.clear();
    for (auto &p : msg->poses)
      marker_points_.emplace_back(p.position.x, p.position.y);
  }

  void onMarkerIDs(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    marker_ids_ = msg->data;
  }

  // ============================================================
  //                         LiDAR（通常）
  // ============================================================
  void onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!received_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "自己位置未受信のため LiDAR 点群を描画できません");
      return;
    }

    // /filtered_points (frame=os_gravity) を odom frame に TF 変換
    sensor_msgs::msg::PointCloud2 cloud_odom;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        "odom", msg->header.frame_id,
        tf2::TimePointZero, std::chrono::milliseconds(50));
      tf2::doTransform(*msg, cloud_odom, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF 変換失敗 (%s → odom): %s",
                           msg->header.frame_id.c_str(), ex.what());
      return;
    }

    // 現在フレームの点群を odom frame で抽出
    const auto now_stamp = this->now();
    LidarFrame frame;
    frame.stamp = now_stamp;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_odom, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_odom, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_odom, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y)) continue;
      if (*iter_z < -0.2 || *iter_z > 0.5) continue;
      frame.points.emplace_back(*iter_x, *iter_y);
    }

    // 履歴に追加し、lidar_history_sec_ より古いフレームを破棄
    lidar_history_.push_back(std::move(frame));
    while (!lidar_history_.empty() &&
           (now_stamp - lidar_history_.front().stamp).seconds() > lidar_history_sec_) {
      lidar_history_.pop_front();
    }

    // 全履歴フレームをフラット化して lidar_points_ を再構築
    lidar_points_.clear();
    for (const auto & f : lidar_history_) {
      lidar_points_.insert(lidar_points_.end(), f.points.begin(), f.points.end());
    }
  }

  // ============================================================
  //                        経路生成
  // ============================================================
  void updatePathToCurrentGoal()
  {
    path_.clear();
    cv::Point2d start = car_pos_;
    cv::Point2d goal  = goal_;
    const int N = 20;
    for (int i=0; i<=N; ++i) {
      double t = static_cast<double>(i) / N;
      double x = (1-t)*start.x + t*goal.x;
      double y = (1-t)*start.y + t*goal.y;
      path_.emplace_back(x, y);
    }
    heading_sent_ = false;
  }

  // Pure Pursuit 用の注視点
  cv::Point2d computeLookaheadPoint()
  {
    const double r = lookahead_dist_;
    const cv::Point2d C = car_pos_;

    cv::Point2d best_pt = goal_;
    bool found = false;

    for (size_t i=0; i+1<path_.size(); ++i) {
      cv::Point2d P1 = path_[i];
      cv::Point2d P2 = path_[i+1];
      cv::Point2d d  = P2 - P1;

      double a = d.dot(d);
      double b = 2 * ((P1.x-C.x)*d.x + (P1.y-C.y)*d.y);
      double c = (P1.x-C.x)*(P1.x-C.x) + (P1.y-C.y)*(P1.y-C.y) - r*r;

      double D = b*b - 4*a*c;
      if (D < 0) continue;

      double sqrtD = std::sqrt(D);
      double t1 = (-b + sqrtD) / (2*a);
      double t2 = (-b - sqrtD) / (2*a);

      std::vector<double> ts;
      if (t1 >= 0.0 && t1 <= 1.0) ts.push_back(t1);
      if (t2 >= 0.0 && t2 <= 1.0) ts.push_back(t2);

      if (!ts.empty()) {
        double t = *std::max_element(ts.begin(), ts.end());
        best_pt = P1 + t*d;
        found = true;
      }
    }

    double dist_to_goal = cv::norm(goal_ - car_pos_);
    if (dist_to_goal < lookahead_dist_) {
      best_pt = goal_;
      found = true;
    }

    return found ? best_pt : goal_;
  }

  // ============================================================
  //                         可視化 Timer
  // ============================================================
  cv::Point toPix(const cv::Point2d& pt, double scale, const cv::Point2d& offset) {
    double x_vis = pt.x;
    double y_vis = -pt.y;
    return cv::Point(static_cast<int>(offset.x + x_vis*scale),
                     static_cast<int>(offset.y + y_vis*scale));
  }

  void onTimer()
  {
    // === ゴール付近の危険チェック (DRIVE中のみ) ===
    if (phase_ == Phase::DRIVE) {
      if (isGoalBlocked()) {
        stopAtBlockedGoal();
        return;
      }
    }

    static bool window_initialized = false;
    if (!window_initialized) {
      cv::namedWindow("Manage Visualization", cv::WINDOW_NORMAL);
      cv::moveWindow("Manage Visualization", 50, 50);
      window_initialized = true;
    }

    // === 可視化のスケール・オフセットを動的計算 ===
    // バウンディングボックス (ローバ + ゴール + 経路) を全体表示
    const int W = 800, H = 800;
    double xmin = car_pos_.x, xmax = car_pos_.x;
    double ymin = car_pos_.y, ymax = car_pos_.y;
    auto expand_box = [&](const cv::Point2d & p) {
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y);
      ymax = std::max(ymax, p.y);
    };
    if (has_goal_) expand_box(goal_);
    for (const auto & p : path_)  expand_box(p);
    // マージン 1.0m
    xmin -= 10.0; xmax += 10.0;
    ymin -= 10.0; ymax += 10.0;
    const double world_w = xmax - xmin;
    const double world_h = ymax - ymin;
    double scale = std::min(W / world_w, H / world_h);
    if (scale < 50.0)  scale = 50.0;    // 下限 (広域表示)
    if (scale > 200.0) scale = 200.0;   // 上限 (過拡大防止)
    // バウンディング中心が画面中心 (W/2, H/2) に来るよう offset 計算
    // toPix 内で y_vis = -pt.y しているので offset.y 側で調整
    const double cx = (xmin + xmax) / 2.0;
    const double cy = (ymin + ymax) / 2.0;
    const cv::Point2d offset(W / 2.0 - cx * scale, H / 2.0 + cy * scale);

    cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(255,255,255));

    // 経路線 (car_pos_ → 現在の goal_)
    for (size_t i=0; i+1<path_.size(); ++i) {
      cv::line(canvas, toPix(path_[i],   scale, offset),
                        toPix(path_[i+1], scale, offset),
               cv::Scalar(0,0,0), 2);
    }

    // ゴール
    if (has_goal_) {
      cv::circle(canvas, toPix(goal_, scale, offset),
                 6, cv::Scalar(0,0,0), 2);
    }

    // マーカー描画
    for (size_t i=0; i<marker_points_.size(); ++i) {
      cv::Point2d m = marker_points_[i];
      cv::circle(canvas, toPix(m, scale, offset),
                 5, cv::Scalar(0,0,255), -1);
      if (i < marker_ids_.size()) {
        std::string label = "ID:" + std::to_string(marker_ids_[i]);
        cv::putText(canvas, label, toPix(m, scale, offset)+cv::Point(8,-8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);
      }
    }

    if (!received_pose_) {
      cv::imshow("Manage Visualization", canvas);
      cv::waitKey(1);
      return;
    }

    // 軌跡
    for (size_t i=1; i<trajectory_.size(); ++i) {
      cv::line(canvas, toPix(trajectory_[i-1], scale, offset),
                        toPix(trajectory_[i],   scale, offset),
               cv::Scalar(255,0,0), 1);
    }

    // LiDAR
    for (auto &p : lidar_points_) {
      cv::circle(canvas, toPix(p, scale, offset),
                 1, cv::Scalar(120,120,120), -1);
    }

    // 現在位置と向き
    cv::circle(canvas, toPix(car_pos_, scale, offset),
               6, cv::Scalar(255,0,0), -1);

    cv::Point2d head = car_pos_ + 0.6*cv::Point2d(std::cos(car_yaw_), std::sin(car_yaw_));
    cv::arrowedLine(canvas, toPix(car_pos_, scale, offset),
                            toPix(head,    scale, offset),
                    cv::Scalar(255,0,0), 2);

    // 目的地がまだ無いときは、DriveNode に目標を出さない
    if (!has_goal_) {
      cv::imshow("Manage Visualization", canvas);
      cv::waitKey(1);
      return;
    }

    // 注視円
    cv::circle(canvas, toPix(car_pos_, scale, offset),
               static_cast<int>(lookahead_dist_*scale),
               cv::Scalar(255,0,0), 1, cv::LINE_AA);

    // ① Pure Pursuit 注視点
    cv::Point2d lookahead = computeLookaheadPoint();
    cv::circle(canvas, toPix(lookahead, scale, offset),
               6, cv::Scalar(0,200,255), -1);

    // ② 障害物回避
    cv::Point2d selected_pt = lookahead;

    if (phase_ == Phase::DRIVE && !lidar_points_.empty()) {
      // 経路バンド: car_pos → goal 直線を中心に幅 BAND_WIDTH の領域
      const double ROBOT_RADIUS   = robot_radius_;
      const double BAND_HALF      = path_band_half_width_;   // 片側 (全幅 = 2*BAND_HALF)
      const double BAND_GOAL_MARG = 0.3;   // ゴールを越えて先まで許容

      // 経路方向の単位ベクトル u と 法線 n
      double gx = goal_.x - car_pos_.x;
      double gy = goal_.y - car_pos_.y;
      double gl = std::hypot(gx, gy);

      double min_dist = 1e9;
      cv::Point2d nearest_pt;

      if (gl > 1e-3) {
        double ux = gx / gl;
        double uy = gy / gl;
        // 法線 (-uy, ux)
        const double s_max = gl + BAND_GOAL_MARG;

        for (auto &p : lidar_points_) {
          double dx = p.x - car_pos_.x;
          double dy = p.y - car_pos_.y;
          double d = std::hypot(dx, dy);
          if (d < ROBOT_RADIUS) continue;

          // 経路方向の射影 s と 法線方向距離 t
          double s = dx * ux + dy * uy;
          double t = std::fabs(-dx * uy + dy * ux);

          // バンド外・後方・ゴール先越しは無視
          if (t > BAND_HALF) continue;
          if (s < 0.0 || s > s_max) continue;

          if (d < min_dist) {
            min_dist = d;
            nearest_pt = p;
          }
        }
      }

      // 回避基点と回避円の可視化 + バンドの可視化
      if (gl > 1e-3) {
        double ux = gx / gl, uy = gy / gl;
        cv::Point2d n1(car_pos_.x + (-uy)*BAND_HALF, car_pos_.y + ux*BAND_HALF);
        cv::Point2d n2(car_pos_.x - (-uy)*BAND_HALF, car_pos_.y - ux*BAND_HALF);
        cv::Point2d g1(goal_.x + (-uy)*BAND_HALF, goal_.y + ux*BAND_HALF);
        cv::Point2d g2(goal_.x - (-uy)*BAND_HALF, goal_.y - ux*BAND_HALF);
        cv::line(canvas, toPix(n1, scale, offset), toPix(g1, scale, offset),
                 cv::Scalar(180,180,180), 1, cv::LINE_AA);
        cv::line(canvas, toPix(n2, scale, offset), toPix(g2, scale, offset),
                 cv::Scalar(180,180,180), 1, cv::LINE_AA);
      }

      if (min_dist < 1e9) {
        cv::circle(canvas, toPix(nearest_pt, scale, offset),
                   4, cv::Scalar(0,0,255), -1);
        cv::circle(canvas, toPix(nearest_pt, scale, offset),
                   static_cast<int>(obstacle_avoid_radius_ * scale),
                   cv::Scalar(0,0,255), 1, cv::LINE_AA);
        std::string r2_label = "r2=" + std::to_string(obstacle_avoid_radius_).substr(0, 3);
        cv::putText(canvas, r2_label,
                    toPix(nearest_pt, scale, offset) + cv::Point(6, -6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255), 1);
      }

      if (min_dist < 1e9) {
        double r1 = lookahead_dist_;
        double r2 = obstacle_avoid_radius_;

        cv::Point2d C1 = car_pos_;
        cv::Point2d C2 = nearest_pt;
        double dx = C2.x - C1.x;
        double dy = C2.y - C1.y;
        double d = std::hypot(dx, dy);

        std::vector<cv::Point2d> intersections;

        if (d < (r1 + r2) && d > std::fabs(r1 - r2)) {
          double a = (r1*r1 - r2*r2 + d*d) / (2*d);
          double h = std::sqrt(std::max(r1*r1 - a*a, 0.0));

          cv::Point2d P0 = C1 + a * (C2 - C1) / d;
          double rx = -dy * (h / d);
          double ry =  dx * (h / d);

          intersections.emplace_back(P0.x + rx, P0.y + ry);
          intersections.emplace_back(P0.x - rx, P0.y - ry);
        }

        // 前方 ±90° の交点のみ採用
        std::vector<cv::Point2d> front_inters;
        for (auto &p : intersections) {
          double dx2 = p.x - car_pos_.x;
          double dy2 = p.y - car_pos_.y;
          double angle = wrapToPi(std::atan2(dy2, dx2) - car_yaw_);
          if (std::fabs(angle) < M_PI/2.0) {
            front_inters.push_back(p);
          }
        }

        if (!front_inters.empty()) {
          if (have_last_selected_) {
            // 連続性優先: 前回 selected_pt に最も近い交点を選ぶ (スイッチング防止)
            double best_dist = 1e9;
            for (auto &p : front_inters) {
              double dist = cv::norm(p - last_selected_pt_);
              if (dist < best_dist) {
                best_dist = dist;
                selected_pt = p;
              }
            }
          } else {
            // 初回: ゴールに最も近い交点
            double best_dist = 1e9;
            for (auto &p : front_inters) {
              double dist = std::hypot(p.x - goal_.x, p.y - goal_.y);
              if (dist < best_dist) {
                best_dist = dist;
                selected_pt = p;
              }
            }
          }
          last_selected_pt_ = selected_pt;
          have_last_selected_ = true;

          // 描画
          for (auto &p : intersections) {
            cv::circle(canvas, toPix(p, scale, offset),
                       5, cv::Scalar(0,0,255), -1);
          }
          cv::circle(canvas, toPix(selected_pt, scale, offset),
                     7, cv::Scalar(0,0,0), 2);
        }
      } else {
        // 回避発動なし → 次回の回避は初回扱い (前回記憶をリセット)
        have_last_selected_ = false;
      }
    } else {
      // DRIVE 以外、または LiDAR 無し → リセット
      have_last_selected_ = false;
    }

    // ④ DriveNode 用 target_point を publish
    geometry_msgs::msg::PointStamped tgt_msg;
    tgt_msg.header.stamp = this->now();
    tgt_msg.header.frame_id = "map";
    tgt_msg.point.x = selected_pt.x;
    tgt_msg.point.y = selected_pt.y;
    tgt_msg.point.z = 0.0;
    target_pub_->publish(tgt_msg);

    // ゴールを publish
    geometry_msgs::msg::Pose2D goal_msg;
    goal_msg.x = goal_.x;
    goal_msg.y = goal_.y;
    goal_msg.theta = goal_yaw_;
    goal_pub_->publish(goal_msg);

    cv::imshow("Manage Visualization", canvas);
    cv::waitKey(1);
  }

  // ============================================================
  //                         メンバ変数
  // ============================================================
  // Sub / Pub / Timer
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr marker_id_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_done_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            heading_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr        goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             cmd_source_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             result_pub_;

  // TF (os_gravity → odom 変換用)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  // 状態管理
  Phase phase_;
  bool received_pose_ = false;
  bool heading_sent_  = false;

  cv::Point2d car_pos_;
  double car_yaw_;
  std::vector<cv::Point2d> trajectory_;

  bool        has_goal_ = false;
  cv::Point2d goal_;
  double      goal_yaw_ = 0.0;

  double lookahead_dist_;
  double lidar_history_sec_;    // 過去点群の保持期間 [s]
  double robot_radius_;         // ローバ半径 (最近点除外閾値)
  double obstacle_avoid_radius_;// 障害物回避円 r2
  double path_band_half_width_; // 経路バンド片側幅

  std::string voicevox_url_;    // Voicevox サーバ URL (空で無効)

  // 回避交点のスイッチング防止 (連続性維持)
  cv::Point2d last_selected_pt_{0.0, 0.0};
  bool have_last_selected_{false};
  std::vector<cv::Point2d> path_;

  std::vector<cv::Point2d> marker_points_;
  std::vector<int>         marker_ids_;

  // 過去 LiDAR フレーム (odom frame, z フィルタ済み) の時系列蓄積
  struct LidarFrame {
    rclcpp::Time stamp;
    std::vector<cv::Point2d> points;
  };
  std::deque<LidarFrame> lidar_history_;
  std::vector<cv::Point2d> lidar_points_;

  std::ofstream log_pose_file_;
  std::string   log_dir_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManageNodeMaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

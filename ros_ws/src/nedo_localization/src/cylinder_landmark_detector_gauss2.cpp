#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <lidar_practice3_msgs/msg/landmark_detection.hpp>
#include <lidar_practice3_msgs/msg/landmark_detection_array.hpp>
#include <cmath>
#include <vector>
#include <queue>
#include <deque>
#include <numeric>
#include <algorithm>
#include <map>
#include <string>

namespace Params
{
  // --- 強度フィルター ---
  constexpr float INTENSITY_RATIO   = 0.60f;
  constexpr float INTENSITY_ABS_MIN = 160.0f;

  // --- フレーム蓄積 ---
  constexpr int ACCUMULATE_FRAMES   = 1;
  constexpr int MAX_ACCUMULATED_PTS = 10000;

  // --- クラスタリング ---
  constexpr float CLUSTER_DIST_BASE = 0.10f;
  constexpr float CLUSTER_DIST      = 0.12f;
  constexpr int   MIN_CLUSTER_SIZE  = 2;
  constexpr int   MAX_CLUSTER_SIZE  = 1000;

  // --- 距離適応型クラスタリング ---
  constexpr bool  CLUSTER_DIST_ADAPTIVE = true;
  constexpr float CLUSTER_DIST_SLOPE    = 0.004f;
  constexpr float CLUSTER_DIST_MAX      = 0.30f;

  // --- 壁除去 ---
  constexpr float WALL_SIZE_MAX = 1.50f;

  // --- 反射材クラスタ判定 ---
  constexpr float STRIPE_HEIGHT_MIN     = 0.03f;
  constexpr float STRIPE_HEIGHT_MAX     = 0.50f;
  constexpr float STRIPE_SIZE_THRESHOLD = 0.30f;
  constexpr float GAP_THRESHOLD         = 0.25f;

  // --- ストライプ高さの距離補正 ---
  constexpr bool  STRIPE_HEIGHT_DIST_ADAPTIVE = true;
  constexpr float STRIPE_HEIGHT_MIN_FAR       = 0.02f;

  // --- フラットトップ中心法 ---
  constexpr int   HALF_VALUE_MIN_POINTS = 2;
  constexpr int   GAP_TRIM_MIN_POINTS   = 5;
  constexpr float GAP_FACTOR            = 3.0f;
  constexpr float GAP_ABS_MIN           = 0.02f;

  // --- グループ化 ---
  constexpr int   EXPECTED_STRIPE_COUNT = 3;
  constexpr float GROUP_XY_THRESHOLD   = 0.30f;

  // --- パターンマッチング ---
  constexpr int  HAMMING_MAX_DIST       = 1;
  constexpr bool GAP_CORRECTION_ENABLED = true;

  // --- 時間的多数決 ---
  constexpr int VOTE_WINDOW     = 1;
  constexpr int VOTE_THRESHOLD  = 1;
  constexpr int MAX_HOLD_FRAMES = 0;

  // --- 形状フィルタリング ---
  constexpr bool  SHAPE_FILTER_ENABLED = true;
  constexpr float SHAPE_XY_MIN = 0.00f;
  constexpr float SHAPE_XY_MAX = 0.60f;

  // --- 反射材サイズ判定モード ---
  // MODE_HEIGHT     = 0: Z高さ（BBox）で判定
  // MODE_POINTCOUNT = 1: 点数比率で判定
  // MODE_HALF_WIDTH = 2: 半値幅で判定（erf-boxプロファイルに基づく）
  constexpr int   SIZE_JUDGE_MODE              = 0;
  constexpr float THICK_RATIO_THRESHOLD        = 0.70f;
  constexpr int   MIN_POINTS_FOR_SIZE_JUDGMENT = 3;
  constexpr bool  SIZE_DIST_CORRECTION         = true;
  constexpr float HALF_WIDTH_SIZE_THRESHOLD    = 0.12f;
}

const std::map<std::string, std::vector<int>> KNOWN_PATTERNS = {
  {"ID_1", {1, 0, 0, 0, 0}},
  {"ID_2", {0, 0, 1, 0, 0}},
  {"ID_3", {0, 0, 0, 0, 1}},
  {"ID_4", {0, 0, 0, 1, 0}},
  {"ID_5", {0, 1, 0, 0, 0}},
};

struct Color { float r, g, b; };
const std::map<std::string, Color> ID_COLORS = {
  {"ID_1", {1.0f, 0.0f, 0.0f}},
  {"ID_2", {0.0f, 1.0f, 0.0f}},
  {"ID_3", {0.0f, 0.0f, 1.0f}},
  {"ID_4", {1.0f, 1.0f, 0.0f}},
  {"ID_5", {1.0f, 0.0f, 1.0f}},
};

struct Centroid { float x, y, z; };
struct BBox {
  float x_min, x_max, y_min, y_max, z_min, z_max;
  float width()  const { return x_max - x_min; }
  float depth()  const { return y_max - y_min; }
  float height() const { return z_max - z_min; }
};
struct Point4 { float x, y, z, intensity; };

static float dist3d(const Point4& a, const Point4& b)
{
  float dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
  return std::sqrt(dx*dx+dy*dy+dz*dz);
}

static Centroid calcCentroid(const std::vector<int>& c, const std::vector<Point4>& pts)
{
  Centroid g{0,0,0};
  for (int i:c) { g.x+=pts[i].x; g.y+=pts[i].y; g.z+=pts[i].z; }
  float n=c.size(); g.x/=n; g.y/=n; g.z/=n;
  return g;
}

static BBox calcBBox(const std::vector<int>& c, const std::vector<Point4>& pts)
{
  BBox b{1e6f,-1e6f,1e6f,-1e6f,1e6f,-1e6f};
  for (int i:c) {
    b.x_min=std::min(b.x_min,pts[i].x); b.x_max=std::max(b.x_max,pts[i].x);
    b.y_min=std::min(b.y_min,pts[i].y); b.y_max=std::max(b.y_max,pts[i].y);
    b.z_min=std::min(b.z_min,pts[i].z); b.z_max=std::max(b.z_max,pts[i].z);
  }
  return b;
}

static bool isWall(const BBox& b)
{ return b.width()>Params::WALL_SIZE_MAX || b.height()>Params::WALL_SIZE_MAX; }

static bool isStripe(const BBox& b, float xy_dist = 0.0f)
{
  float h_min = Params::STRIPE_HEIGHT_MIN;
  if (Params::STRIPE_HEIGHT_DIST_ADAPTIVE && xy_dist > 15.0f) {
    const float t = std::min((xy_dist - 15.0f) / 15.0f, 1.0f);
    h_min = Params::STRIPE_HEIGHT_MIN
          - t * (Params::STRIPE_HEIGHT_MIN - Params::STRIPE_HEIGHT_MIN_FAR);
  }
  return b.height() >= h_min && b.height() <= Params::STRIPE_HEIGHT_MAX;
}

static bool shapeFilter(const std::vector<int>& cluster,
                        const std::vector<Point4>& pts)
{
  if (!Params::SHAPE_FILTER_ENABLED) return true;

  float x_min=1e6f, x_max=-1e6f, y_min=1e6f, y_max=-1e6f;
  for (int i:cluster) {
    x_min=std::min(x_min,pts[i].x); x_max=std::max(x_max,pts[i].x);
    y_min=std::min(y_min,pts[i].y); y_max=std::max(y_max,pts[i].y);
  }
  const float xy_spread = std::sqrt(
    (x_max-x_min)*(x_max-x_min) + (y_max-y_min)*(y_max-y_min));

  const bool ok = (xy_spread >= Params::SHAPE_XY_MIN &&
                   xy_spread <= Params::SHAPE_XY_MAX);

  if (!ok)
    RCLCPP_DEBUG(rclcpp::get_logger("cylinder_landmark_detector_gauss2"),
      "  [形状フィルタ] XY広がり=%.3fm → スキップ", xy_spread);

  return ok;
}

// 実測プロファイルがフラットトップなので端のギャップのみ非対称トリムして中点を取る
static float flatTopCenterZ(const std::vector<int>& cluster,
                            const std::vector<Point4>& pts,
                            const BBox& b,
                            float& half_width_out,
                            bool& success)
{
  success = false;
  half_width_out = 0.0f;

  const int n = static_cast<int>(cluster.size());
  const float z_bbox_center = (b.z_min + b.z_max) * 0.5f;

  if (n < Params::HALF_VALUE_MIN_POINTS) return z_bbox_center;

  std::vector<float> valid_zs;
  valid_zs.reserve(n);
  for (int k = 0; k < n; k++)
    valid_zs.push_back(pts[cluster[k]].z);

  const int nv = static_cast<int>(valid_zs.size());

  if (nv < Params::HALF_VALUE_MIN_POINTS) {
    half_width_out = b.height();
    success = true;
    return z_bbox_center;
  }

  std::sort(valid_zs.begin(), valid_zs.end());

  if (nv < Params::GAP_TRIM_MIN_POINTS) {
    half_width_out = valid_zs[nv-1] - valid_zs[0];
    success = true;
    return (valid_zs[0] + valid_zs[nv-1]) * 0.5f;
  }

  const int n_gaps = nv - 1;
  std::vector<float> gaps(n_gaps);
  for (int i = 0; i < n_gaps; i++)
    gaps[i] = valid_zs[i+1] - valid_zs[i];

  // 内部ギャップ（両端を除く）の中央値を基準にして端の外れ値のみ判定する
  std::vector<float> inner_gaps;
  inner_gaps.reserve(n_gaps);
  for (int i = 1; i < n_gaps - 1; i++)
    inner_gaps.push_back(gaps[i]);

  float median_gap;
  if (inner_gaps.empty()) {
    median_gap = gaps[0];
    if (n_gaps > 1) median_gap = std::min(gaps[0], gaps[n_gaps-1]);
  } else {
    std::sort(inner_gaps.begin(), inner_gaps.end());
    median_gap = inner_gaps[inner_gaps.size() / 2];
  }

  // GAP_ABS_MINは密度非対称時に中央値が密な側へ引きずられて正常点を誤判定するのを防ぐ
  const float gap_thr = std::max(median_gap * Params::GAP_FACTOR, Params::GAP_ABS_MIN);

  int lo = 0;
  while (lo < nv - 1 && gaps[lo] > gap_thr) lo++;

  int hi = nv - 1;
  while (hi > lo && gaps[hi-1] > gap_thr) hi--;

  if (lo >= hi) {
    half_width_out = b.height();
    success = true;
    return z_bbox_center;
  }

  half_width_out = valid_zs[hi] - valid_zs[lo];
  success = true;

  return (valid_zs[lo] + valid_zs[hi]) * 0.5f;
}

static int stripeSize(float h)
{ return h>=Params::STRIPE_SIZE_THRESHOLD ? 1 : 0; }

static int gapSize(float cd, float hl, float hu)
{
  float gap=cd-hl*0.5f-hu*0.5f;
  return gap>=Params::GAP_THRESHOLD ? 1 : 0;
}

class CylinderLandmarkDetectorGauss2 : public rclcpp::Node
{
public:
  CylinderLandmarkDetectorGauss2() : Node("cylinder_landmark_detector_gauss2")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar_3JEDM4V001DY881", qos,
      std::bind(&CylinderLandmarkDetectorGauss2::callback, this,
                std::placeholders::_1));

    pub_cloud_  = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cylinder_landmark_points_gauss2", 10);
    pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/matched_marker", 10);
    pub_poses_  = create_publisher<geometry_msgs::msg::PoseArray>(
      "/cylinder_landmark_poses", 10);
    pub_detections_ = create_publisher<lidar_practice3_msgs::msg::LandmarkDetectionArray>(
      "/landmark_detections", 10);

    RCLCPP_INFO(get_logger(), "ランドマーク検出ノード起動（フラットトップ中心法）");
    RCLCPP_INFO(get_logger(),
      "  [形状フィルタ] %s  XY広がり %.3f〜%.3fm",
      Params::SHAPE_FILTER_ENABLED?"有効":"無効",
      Params::SHAPE_XY_MIN, Params::SHAPE_XY_MAX);

    int id_idx = 0;
    for (const auto& [id,_]:KNOWN_PATTERNS) {
      landmark_confirmed_[id]=false;
      confirmed_positions_[id]={0,0,0};
      frames_since_confirmed_[id]=0;
      marker_id_base_[id] = id_idx * 2;
      id_idx++;
    }
  }

private:
  struct TimedFrame { std::vector<Point4> points; };
  std::deque<TimedFrame> frame_buffer_;

  std::map<std::string,std::deque<int>> detection_history_;
  std::map<std::string,bool>            landmark_confirmed_;
  std::map<std::string,Centroid>        confirmed_positions_;
  std::map<std::string,int>             frames_since_confirmed_;
  std::map<std::string,int>             marker_id_base_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    pub_poses_;
  rclcpp::Publisher<lidar_practice3_msgs::msg::LandmarkDetectionArray>::SharedPtr pub_detections_;

  std::vector<Point4> intensityFilter(const std::vector<Point4>& pts)
  {
    float mx=0;
    for (auto& p:pts) mx=std::max(mx,p.intensity);
    float thr=std::max(mx*Params::INTENSITY_RATIO, Params::INTENSITY_ABS_MIN);
    std::vector<Point4> res;
    for (auto& p:pts) if (p.intensity>thr) res.push_back(p);
    RCLCPP_DEBUG(get_logger(),"[強度フィルター] max=%.1f 閾値=%.1f 通過=%zu点",
      mx,thr,res.size());
    return res;
  }

  std::vector<Point4> accumulateFrames(const std::vector<Point4>& cur)
  {
    frame_buffer_.push_back({cur});
    while ((int)frame_buffer_.size()>Params::ACCUMULATE_FRAMES)
      frame_buffer_.pop_front();

    std::vector<Point4> acc;
    for (const auto& frame : frame_buffer_)
      acc.insert(acc.end(), frame.points.begin(), frame.points.end());

    if ((int)acc.size()>Params::MAX_ACCUMULATED_PTS)
      acc.erase(acc.begin(),acc.begin()+acc.size()-Params::MAX_ACCUMULATED_PTS);

    return acc;
  }

  std::vector<std::vector<int>> clustering(const std::vector<Point4>& pts)
  {
    float cd = Params::CLUSTER_DIST;
    if (Params::CLUSTER_DIST_ADAPTIVE && !pts.empty()) {
      float avg_d = 0;
      for (auto& p:pts) avg_d += std::sqrt(p.x*p.x + p.y*p.y);
      avg_d /= pts.size();
      cd = Params::CLUSTER_DIST_BASE + avg_d * Params::CLUSTER_DIST_SLOPE;
      cd = std::min(cd, Params::CLUSTER_DIST_MAX);
      RCLCPP_DEBUG(get_logger(),
        "[クラスタリング] 平均距離=%.1fm → 閾値=%.3fm", avg_d, cd);
    }

    std::vector<bool> vis(pts.size(),false);
    std::vector<std::vector<int>> clusters;
    for (size_t i=0;i<pts.size();i++) {
      if (vis[i]) continue;
      vis[i]=true;
      std::vector<int> cl; std::queue<int> bfs;
      bfs.push(i); cl.push_back(i);
      while (!bfs.empty()) {
        int c=bfs.front(); bfs.pop();
        for (size_t j=0;j<pts.size();j++)
          if (!vis[j]&&dist3d(pts[c],pts[j])<cd)
            { vis[j]=true; cl.push_back(j); bfs.push(j); }
      }
      if ((int)cl.size()>=Params::MIN_CLUSTER_SIZE &&
          (int)cl.size()<=Params::MAX_CLUSTER_SIZE)
        clusters.push_back(cl);
    }
    return clusters;
  }

  struct StripeInfo {
    Centroid centroid;
    float    height;
    float    half_width;
    bool     half_value_ok;
    int      point_count;
    float    xy_dist;
    float    norm_count;
  };
  struct LandmarkGroup {
    std::vector<StripeInfo> stripes;
    Centroid group_centroid;
    std::vector<int> pattern;
    std::string matched_id;
  };

  std::vector<LandmarkGroup> detectLandmarks(
    const std::vector<std::vector<int>>& clusters,
    const std::vector<Point4>& pts)
  {
    std::vector<StripeInfo> cands;

    for (auto& cl:clusters)
    {
      BBox b=calcBBox(cl,pts);
      if (isWall(b)) continue;
      float cl_xy_dist = 0;
      for (int i:cl) cl_xy_dist += std::sqrt(pts[i].x*pts[i].x + pts[i].y*pts[i].y);
      cl_xy_dist /= cl.size();
      if (!isStripe(b, cl_xy_dist)) {
        RCLCPP_DEBUG(get_logger(),"  [スキップ] Z=%.3fm → 反射材サイズ外",b.height());
        continue;
      }

      if (!shapeFilter(cl,pts)) continue;

      float half_w=0; bool ok=false;
      float z_center=flatTopCenterZ(cl,pts,b,half_w,ok);
      Centroid xy=calcCentroid(cl,pts);
      const int   pt_count = static_cast<int>(cl.size());
      const float d_xy     = std::sqrt(xy.x*xy.x + xy.y*xy.y);
      const float norm_cnt = static_cast<float>(pt_count) * d_xy * d_xy;
      cands.push_back({{xy.x,xy.y,z_center}, b.height(), half_w, ok,
                       pt_count, d_xy, norm_cnt});
    }

    if (cands.empty()) return {};

    std::vector<bool> grouped(cands.size(),false);
    std::vector<LandmarkGroup> lms;

    for (size_t i=0;i<cands.size();i++) {
      if (grouped[i]) continue;
      LandmarkGroup g; g.stripes.push_back(cands[i]); grouped[i]=true;
      for (size_t j=i+1;j<cands.size();j++) {
        if (grouped[j]) continue;
        float dx=cands[i].centroid.x-cands[j].centroid.x;
        float dy=cands[i].centroid.y-cands[j].centroid.y;
        if (std::sqrt(dx*dx+dy*dy)<Params::GROUP_XY_THRESHOLD)
          { g.stripes.push_back(cands[j]); grouped[j]=true; }
      }
      const int n_stripes = static_cast<int>(g.stripes.size());

      if (n_stripes != Params::EXPECTED_STRIPE_COUNT) {
        RCLCPP_DEBUG(get_logger(),"  [スキップ] 反射材%d枚 → 期待%d枚と不一致",
          n_stripes, Params::EXPECTED_STRIPE_COUNT);
        continue;
      }

      std::sort(g.stripes.begin(),g.stripes.end(),
        [](const StripeInfo& a,const StripeInfo& b){ return a.centroid.z<b.centroid.z; });

      Centroid gc{0,0,0};
      for (auto& s:g.stripes) { gc.x+=s.centroid.x; gc.y+=s.centroid.y; gc.z+=s.centroid.z; }
      float n=g.stripes.size(); gc.x/=n; gc.y/=n; gc.z/=n;
      g.group_centroid=gc;

      auto& s0=g.stripes[0]; auto& s1=g.stripes[1]; auto& s2=g.stripes[2];

      int sz0, sz1, sz2;
      if (Params::SIZE_JUDGE_MODE == 0) {
        sz0 = stripeSize(s0.height);
        sz1 = stripeSize(s1.height);
        sz2 = stripeSize(s2.height);
        RCLCPP_DEBUG(get_logger(),
          "[サイズ判定/BBox高さ] 下=%.3fm 中=%.3fm 上=%.3fm",
          s0.height, s1.height, s2.height);
      } else if (Params::SIZE_JUDGE_MODE == 2) {
        sz0 = (s0.half_value_ok && s0.half_width >= Params::HALF_WIDTH_SIZE_THRESHOLD) ? 1 : 0;
        sz1 = (s1.half_value_ok && s1.half_width >= Params::HALF_WIDTH_SIZE_THRESHOLD) ? 1 : 0;
        sz2 = (s2.half_value_ok && s2.half_width >= Params::HALF_WIDTH_SIZE_THRESHOLD) ? 1 : 0;
        RCLCPP_DEBUG(get_logger(),
          "[サイズ判定/半値幅] 下=%.3fm(%s) 中=%.3fm(%s) 上=%.3fm(%s)",
          s0.half_width, s0.half_value_ok?"OK":"NG",
          s1.half_width, s1.half_value_ok?"OK":"NG",
          s2.half_width, s2.half_value_ok?"OK":"NG");
      } else {
        float val0, val1, val2, max_val;
        if (Params::SIZE_DIST_CORRECTION) {
          val0 = s0.norm_count;
          val1 = s1.norm_count;
          val2 = s2.norm_count;
        } else {
          val0 = static_cast<float>(s0.point_count);
          val1 = static_cast<float>(s1.point_count);
          val2 = static_cast<float>(s2.point_count);
        }
        max_val = std::max({val0, val1, val2});

        const bool enough0 = s0.point_count >= Params::MIN_POINTS_FOR_SIZE_JUDGMENT;
        const bool enough1 = s1.point_count >= Params::MIN_POINTS_FOR_SIZE_JUDGMENT;
        const bool enough2 = s2.point_count >= Params::MIN_POINTS_FOR_SIZE_JUDGMENT;

        if (max_val > 0.0f) {
          sz0 = (enough0 && val0/max_val >= Params::THICK_RATIO_THRESHOLD) ? 1 : 0;
          sz1 = (enough1 && val1/max_val >= Params::THICK_RATIO_THRESHOLD) ? 1 : 0;
          sz2 = (enough2 && val2/max_val >= Params::THICK_RATIO_THRESHOLD) ? 1 : 0;
        } else {
          sz0 = sz1 = sz2 = 0;
        }

        RCLCPP_DEBUG(get_logger(),
          "[サイズ判定/点数比率%s] 下=%d点(%.2f) 中=%d点(%.2f) 上=%d点(%.2f) max=%.1f",
          Params::SIZE_DIST_CORRECTION?"補正あり":"補正なし",
          s0.point_count, (max_val>0?val0/max_val:0.0f),
          s1.point_count, (max_val>0?val1/max_val:0.0f),
          s2.point_count, (max_val>0?val2/max_val:0.0f),
          max_val);
      }

      int gp0=gapSize(s1.centroid.z-s0.centroid.z,s0.height,s1.height);
      int gp1=gapSize(s2.centroid.z-s1.centroid.z,s1.height,s2.height);
      g.pattern={sz0,gp0,sz1,gp1,sz2};

      RCLCPP_DEBUG(get_logger(),
        "[グループ] 重心:(%.2f,%.2f,%.2f)"
        "  下:%s 隙間(%s) 中:%s 隙間(%s) 上:%s"
        "  → パターン:{%d,%d,%d,%d,%d}",
        gc.x,gc.y,gc.z,
        sz0?"太":"細", gp0?"大+":"大",
        sz1?"太":"細", gp1?"大+":"大",
        sz2?"太":"細",
        sz0,gp0,sz1,gp1,sz2);

      g.matched_id = "";
      {
        std::vector<int> match_pattern = g.pattern;

        if (Params::GAP_CORRECTION_ENABLED) {
          bool corrected = false;
          if (match_pattern.size() >= 5) {
            if (match_pattern[1] != 0) { match_pattern[1] = 0; corrected = true; }
            if (match_pattern[3] != 0) { match_pattern[3] = 0; corrected = true; }
          }
          if (corrected) {
            RCLCPP_DEBUG(get_logger(),
              "[パターン補正] {%d,%d,%d,%d,%d} → {%d,%d,%d,%d,%d} (ギャップ補正)",
              g.pattern[0], g.pattern[1], g.pattern[2],
              g.pattern[3], g.pattern[4],
              match_pattern[0], match_pattern[1], match_pattern[2],
              match_pattern[3], match_pattern[4]);
          }
        }

        int best_dist = 999;
        std::string best_id = "";
        bool ambiguous = false;

        for (const auto& [id, known] : KNOWN_PATTERNS) {
          int d = 0;
          for (int k = 0; k < (int)match_pattern.size() && k < (int)known.size(); k++)
            if (match_pattern[k] != known[k]) d++;

          if (d < best_dist) {
            best_dist = d;
            best_id = id;
            ambiguous = false;
          } else if (d == best_dist) {
            ambiguous = true;
          }
        }

        if (best_dist <= Params::HAMMING_MAX_DIST && !ambiguous) {
          g.matched_id = best_id;
          if (best_dist > 0) {
            RCLCPP_DEBUG(get_logger(),
              "[ハミング] {%d,%d,%d,%d,%d} → %s (距離%d)",
              match_pattern[0], match_pattern[1], match_pattern[2],
              match_pattern[3], match_pattern[4],
              best_id.c_str(), best_dist);
          }
        } else if (ambiguous) {
          RCLCPP_DEBUG(get_logger(),
            "[ハミング] {%d,%d,%d,%d,%d} → 曖昧(距離%d, 複数候補)",
            match_pattern[0], match_pattern[1], match_pattern[2],
            match_pattern[3], match_pattern[4], best_dist);
        } else {
          RCLCPP_DEBUG(get_logger(),
            "[ハミング] {%d,%d,%d,%d,%d} → 不一致(最小距離%d > %d)",
            match_pattern[0], match_pattern[1], match_pattern[2],
            match_pattern[3], match_pattern[4],
            best_dist, Params::HAMMING_MAX_DIST);
        }
      }

      lms.push_back(g);
    }
    return lms;
  }

  bool temporalVote(const std::string& id, bool det)
  {
    auto& h=detection_history_[id];
    h.push_back(det?1:0);
    if ((int)h.size()>Params::VOTE_WINDOW) h.pop_front();
    return std::accumulate(h.begin(),h.end(),0)>=Params::VOTE_THRESHOLD;
  }

  visualization_msgs::msg::MarkerArray makeMarkers(
    const std_msgs::msg::Header& header)
  {
    visualization_msgs::msg::MarkerArray arr;

    for (const auto& [id,confirmed]:landmark_confirmed_) {
      const int base_mid = marker_id_base_[id];

      if (!confirmed) {
        // 各IDに固定のマーカーIDを割り当てることで古いマーカーが残らない
        for (int k = 0; k < 2; k++) {
          visualization_msgs::msg::Marker del;
          del.header=header;
          del.ns = (k == 0) ? "landmarks" : "landmark_labels";
          del.id=base_mid + k;
          del.action=visualization_msgs::msg::Marker::DELETE;
          arr.markers.push_back(del);
        }
        continue;
      }
      const Centroid& pos=confirmed_positions_[id];
      const Color& col=ID_COLORS.at(id);

      visualization_msgs::msg::Marker sphere;
      sphere.header=header; sphere.ns="landmarks"; sphere.id=base_mid;
      sphere.type=visualization_msgs::msg::Marker::SPHERE;
      sphere.action=visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x=pos.x; sphere.pose.position.y=pos.y; sphere.pose.position.z=pos.z;
      sphere.pose.orientation.w=1.0;
      sphere.scale.x=sphere.scale.y=sphere.scale.z=0.15;
      sphere.color.r=col.r; sphere.color.g=col.g; sphere.color.b=col.b; sphere.color.a=0.8f;
      arr.markers.push_back(sphere);

      visualization_msgs::msg::Marker text;
      text.header=header; text.ns="landmark_labels"; text.id=base_mid + 1;
      text.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action=visualization_msgs::msg::Marker::ADD;
      text.pose.position.x=pos.x; text.pose.position.y=pos.y; text.pose.position.z=pos.z+0.3;
      text.pose.orientation.w=1.0; text.scale.z=0.2;
      text.color.r=1.0f; text.color.g=1.0f; text.color.b=1.0f; text.color.a=1.0f;

      char buf[64];
      std::snprintf(buf,sizeof(buf),"%s\nx:%.3f y:%.3f",id.c_str(),pos.x,pos.y);
      text.text=buf;
      arr.markers.push_back(text);
    }
    return arr;
  }

  geometry_msgs::msg::PoseArray makePoseArray(
    const std_msgs::msg::Header& header)
  {
    geometry_msgs::msg::PoseArray pa;
    pa.header=header;
    for (const auto& [id,confirmed]:landmark_confirmed_) {
      if (!confirmed) continue;
      const Centroid& pos=confirmed_positions_[id];
      geometry_msgs::msg::Pose p;
      p.position.x=pos.x; p.position.y=pos.y; p.position.z=pos.z;
      p.orientation.w=1.0;
      pa.poses.push_back(p);
    }
    return pa;
  }

  lidar_practice3_msgs::msg::LandmarkDetectionArray makeDetectionArray(
    const std_msgs::msg::Header& header)
  {
    lidar_practice3_msgs::msg::LandmarkDetectionArray arr;
    arr.header = header;
    for (const auto& [id, confirmed] : landmark_confirmed_) {
      if (!confirmed) continue;
      const Centroid& pos = confirmed_positions_[id];
      lidar_practice3_msgs::msg::LandmarkDetection det;
      det.header      = header;
      det.landmark_id = id;
      det.x           = pos.x;
      det.y           = pos.y;
      det.z           = pos.z;
      det.distance    = std::sqrt(pos.x * pos.x + pos.y * pos.y);
      arr.detections.push_back(det);
    }
    return arr;
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<Point4> all;
    sensor_msgs::PointCloud2Iterator<float> ix(*msg,"x"),iy(*msg,"y"),
                                            iz(*msg,"z"),ii(*msg,"intensity");
    for (;ix!=ix.end();++ix,++iy,++iz,++ii)
      if (!std::isnan(*ix)&&!std::isnan(*iy)&&!std::isnan(*iz))
        all.push_back({*ix,*iy,*iz,*ii});
    if (all.empty()) return;

    auto filtered    = intensityFilter(all);
    if (filtered.empty()) return;
    auto accumulated = accumulateFrames(filtered);
    auto clusters    = clustering(accumulated);
    auto landmarks   = detectLandmarks(clusters, accumulated);

    std::vector<Point4> out_pts;
    for (auto& cl:clusters) {
      BBox b=calcBBox(cl,accumulated);
      if (!isWall(b)) for (int i:cl) out_pts.push_back(accumulated[i]);
    }

    std::map<std::string,bool>     det; for (auto& [id,_]:KNOWN_PATTERNS) det[id]=false;
    std::map<std::string,Centroid> pos;
    for (auto& lm:landmarks)
      if (!lm.matched_id.empty()) { det[lm.matched_id]=true; pos[lm.matched_id]=lm.group_centroid; }

    for (auto& [id,_]:KNOWN_PATTERNS) {
      bool confirmed=temporalVote(id,det[id]);
      if (confirmed&&det[id]) {
        landmark_confirmed_[id]=true; frames_since_confirmed_[id]=0;
        confirmed_positions_[id]=pos[id];
        auto& p=confirmed_positions_[id];
        RCLCPP_INFO(get_logger(),"[%s] x=%.3f y=%.3f z=%.3f",id.c_str(),p.x,p.y,p.z);
      } else {
        frames_since_confirmed_[id]++;
        if (frames_since_confirmed_[id]>Params::MAX_HOLD_FRAMES)
          landmark_confirmed_[id]=false;
        else if (landmark_confirmed_[id]) {
          auto& p=confirmed_positions_[id];
          RCLCPP_INFO(get_logger(),"[%s] 保持中 x=%.3f y=%.3f z=%.3f",id.c_str(),p.x,p.y,p.z);
        }
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header=msg->header; cloud.height=1;
    cloud.width=static_cast<uint32_t>(out_pts.size());
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1,"xyz"); mod.resize(out_pts.size());
    sensor_msgs::PointCloud2Iterator<float> ox(cloud,"x"),oy(cloud,"y"),oz(cloud,"z");
    for (auto& p:out_pts) { *ox=p.x;*oy=p.y;*oz=p.z;++ox;++oy;++oz; }
    pub_cloud_->publish(cloud);
    pub_marker_->publish(makeMarkers(msg->header));
    pub_poses_->publish(makePoseArray(msg->header));
    pub_detections_->publish(makeDetectionArray(msg->header));
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<CylinderLandmarkDetectorGauss2>());
  rclcpp::shutdown();
  return 0;
}
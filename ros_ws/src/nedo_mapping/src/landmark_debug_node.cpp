// landmark_debug_node.cpp
//
// 目的（デバッグ可視化）:
//   LiDAR点群トピックを購読し、landmark_mapper_node.cpp と同一ロジックで
//     ① 強度フィルタ後の点群
//     ② ID認識（BINパターン判定）に実際に用いた点群
//   をそれぞれ PointCloud2 として配信する。RViz2 で検出器の挙動を確認するための
//   オフライン解析用ノード（地図生成・CSV出力は行わない）。
//
// 配信トピック:
//   ~/intensity_filtered : 強度フィルタを通過した点
//   ~/id_points          : Zスパンゲートを通った候補円柱の、円柱XY半径内かつ
//                          各BINのZ範囲に入る点（= BINパターンを決めた点）
//                          intensity フィールドに「BIN番号(1..4)」を格納し、
//                          RViz2 で intensity 着色するとビンごとに色分けされる。
//
// 購読トピック:
//   /lidar_points : sensor_msgs/PointCloud2（実LiDAR点群。-r で変更可）
//
// ※ 検出パラメータは landmark_mapper_node.cpp と一致させてある。
//   片方を変えたら両方更新すること。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// =============================================================================
// パラメータ（landmark_mapper_node.cpp と同一）
// =============================================================================
namespace Params
{
  constexpr float INTENSITY_RATIO       = 0.80f;
  constexpr float INTENSITY_ABS_MIN     = 160.0f;

  constexpr float ANCHOR_HEIGHT         = 0.30f;
  constexpr float BIN_HEIGHT            = 0.30f;
  constexpr int   BIN_COUNT             = 4;

  constexpr float HEIGHT_TOLERANCE_LOW  = 0.15f;
  constexpr float HEIGHT_TOLERANCE_HIGH = 0.15f;

  constexpr float GROUP_XY_THRESHOLD    = 0.30f;
  constexpr int   MIN_GROUP_POINTS      = 3;

  constexpr float CYLINDER_XY_RADIUS    = 0.20f;
  constexpr float BIN_INNER_MARGIN      = 0.05f;
}

struct Point4 { float x, y, z, intensity; };

// =============================================================================
// 強度フィルタ（landmark_mapper_node.cpp と同一）
// =============================================================================
static std::vector<Point4> intensityFilter(const std::vector<Point4>& pts)
{
  float mx = 0.0f;
  for (const auto& p : pts) mx = std::max(mx, p.intensity);
  const float thr = std::max(mx * Params::INTENSITY_RATIO, Params::INTENSITY_ABS_MIN);
  std::vector<Point4> res;
  res.reserve(pts.size());
  for (const auto& p : pts) if (p.intensity > thr) res.push_back(p);
  return res;
}

// =============================================================================
// ID認識に用いた点群を抽出
//   landmark_mapper_node.cpp の detectLandmarks のステップ①〜⑤を再現し、
//   各候補円柱で BIN カウントに使われた点を（BIN番号付きで）集める。
//   返す点の intensity には BIN番号(1..4) を入れる（可視化用）。
// =============================================================================
static std::vector<Point4> collectIdPoints(const std::vector<Point4>& pts)
{
  std::vector<Point4> out;
  if (pts.empty()) return out;
  const size_t N = pts.size();

  // --- ① 2D XY BFS グループ化 ---
  const float xy_d2 = Params::GROUP_XY_THRESHOLD * Params::GROUP_XY_THRESHOLD;
  std::vector<int> gid(N, -1);
  int ng = 0;
  for (size_t i = 0; i < N; i++) {
    if (gid[i] != -1) continue;
    gid[i] = ng;
    std::queue<int> q; q.push((int)i);
    while (!q.empty()) {
      int cur = q.front(); q.pop();
      for (size_t j = 0; j < N; j++) {
        if (gid[j] != -1) continue;
        const float dx = pts[cur].x - pts[j].x, dy = pts[cur].y - pts[j].y;
        if (dx * dx + dy * dy <= xy_d2) { gid[j] = ng; q.push((int)j); }
      }
    }
    ng++;
  }
  std::vector<std::vector<int>> groups(ng);
  for (size_t i = 0; i < N; i++) groups[gid[i]].push_back((int)i);

  const float expected_dist =
    2.0f * Params::ANCHOR_HEIGHT + Params::BIN_COUNT * Params::BIN_HEIGHT
    - 2.0f * Params::BIN_INNER_MARGIN;
  const float r2 = Params::CYLINDER_XY_RADIUS * Params::CYLINDER_XY_RADIUS;

  for (const auto& grp : groups) {
    if ((int)grp.size() < Params::MIN_GROUP_POINTS) continue;

    // --- ② Z分布 ---
    std::vector<float> zv; zv.reserve(grp.size());
    float sx = 0.0f, sy = 0.0f;
    for (int idx : grp) { zv.push_back(pts[idx].z); sx += pts[idx].x; sy += pts[idx].y; }
    std::sort(zv.begin(), zv.end());
    const int n = (int)zv.size();

    const int k = (n >= 8) ? 1 : 0;
    const float z_low  = zv[k];
    const float z_high = zv[n - 1 - k];
    const float z_span = z_high - z_low;

    if (z_span < expected_dist - Params::HEIGHT_TOLERANCE_LOW ||
        z_span > expected_dist + Params::HEIGHT_TOLERANCE_HIGH)
      continue;

    const float cx = sx / n, cy = sy / n;

    // --- ③ スケール適応型アンカー・ビン境界推定 ---
    const float scale =
      z_span / (2.0f * Params::ANCHOR_HEIGHT +
                Params::BIN_COUNT * Params::BIN_HEIGHT);
    const float scaled_anchor = Params::ANCHOR_HEIGHT * scale;

    const float anchor_frac =
      Params::ANCHOR_HEIGHT /
      (2.0f * Params::ANCHOR_HEIGHT + Params::BIN_COUNT * Params::BIN_HEIGHT);
    const float ba_hi = z_low  + z_span * anchor_frac;
    const float ta_lo = z_high - z_span * anchor_frac;

    float ba_sum = 0.0f, ta_sum = 0.0f;
    int bot_cnt = 0, top_cnt = 0;
    for (int idx : grp) {
      const float z = pts[idx].z;
      if (z <= ba_hi) { ba_sum += z; bot_cnt++; }
      if (z >= ta_lo) { ta_sum += z; top_cnt++; }
    }
    const float z_ba = (bot_cnt > 0) ? ba_sum / bot_cnt : z_low  + scaled_anchor * 0.5f;
    const float z_ta = (top_cnt > 0) ? ta_sum / top_cnt : z_high - scaled_anchor * 0.5f;

    const float s = (z_ta - z_ba) / (Params::BIN_COUNT + 1);
    const float scale_local = s / Params::BIN_HEIGHT;
    const float half_w =
      (Params::BIN_HEIGHT * 0.5f - Params::BIN_INNER_MARGIN) * scale_local;

    // --- ④ 各ビンのZ範囲 ---
    std::vector<float> bl(Params::BIN_COUNT), bh(Params::BIN_COUNT);
    for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
      const float center = z_ba + s * (kk + 1);
      bl[kk] = center - half_w;
      bh[kk] = center + half_w;
    }

    // --- ⑤ 円柱XY半径内 & ビンZ範囲内の点を収集（BIN番号付き）---
    for (const auto& p : pts) {
      const float dx = p.x - cx, dy = p.y - cy;
      if (dx * dx + dy * dy > r2) continue;
      for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
        if (p.z >= bl[kk] && p.z < bh[kk]) {
          out.push_back({p.x, p.y, p.z, (float)(kk + 1)});  // intensity = BIN番号
          break;
        }
      }
    }
  }
  return out;
}

// =============================================================================
// PointCloud2 構築（x,y,z,intensity : float32）
// =============================================================================
static sensor_msgs::msg::PointCloud2 buildCloud(
  const std_msgs::msg::Header& header, const std::vector<Point4>& pts)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> ii(msg, "intensity");
  for (const auto& p : pts) {
    *ix = p.x; *iy = p.y; *iz = p.z; *ii = p.intensity;
    ++ix; ++iy; ++iz; ++ii;
  }
  msg.is_dense = true;
  return msg;
}

// =============================================================================
// ノード本体
// =============================================================================
class LandmarkDebugNode : public rclcpp::Node
{
public:
  LandmarkDebugNode() : Node("landmark_debug_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(msg); });

    pub_filtered_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/intensity_filtered", rclcpp::SensorDataQoS());
    pub_id_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/id_points", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "landmark_debug_node ready (sub: '/lidar_points' -> pub: '~/intensity_filtered', '~/id_points')");
  }

private:
  static int offsetOf(const sensor_msgs::msg::PointCloud2& msg, const std::string& name)
  {
    for (const auto& f : msg.fields) if (f.name == name) return (int)f.offset;
    return -1;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    const int ox = offsetOf(*msg, "x");
    const int oy = offsetOf(*msg, "y");
    const int oz = offsetOf(*msg, "z");
    const int oi = offsetOf(*msg, "intensity");
    if (ox < 0 || oy < 0 || oz < 0) {
      RCLCPP_WARN_ONCE(get_logger(), "入力に x/y/z フィールドがありません。スキップします。");
      return;
    }
    if (oi < 0) {
      RCLCPP_WARN_ONCE(get_logger(),
        "入力に intensity フィールドがありません。強度フィルタは機能しません。");
    }

    const size_t npts = (size_t)msg->width * msg->height;
    std::vector<Point4> pts;
    pts.reserve(npts);
    const uint8_t* base = msg->data.data();
    for (size_t k = 0; k < npts; k++) {
      const uint8_t* p = base + k * msg->point_step;
      Point4 q;
      std::memcpy(&q.x, p + ox, 4);
      std::memcpy(&q.y, p + oy, 4);
      std::memcpy(&q.z, p + oz, 4);
      q.intensity = 0.0f;
      if (oi >= 0) std::memcpy(&q.intensity, p + oi, 4);
      if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z)) continue;
      pts.push_back(q);
    }

    const auto filtered = intensityFilter(pts);
    const auto id_pts   = collectIdPoints(filtered);

    pub_filtered_->publish(buildCloud(msg->header, filtered));
    pub_id_->publish(buildCloud(msg->header, id_pts));

    RCLCPP_DEBUG(get_logger(), "in=%zu filtered=%zu id_points=%zu",
                 pts.size(), filtered.size(), id_pts.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkDebugNode>());
  rclcpp::shutdown();
  return 0;
}

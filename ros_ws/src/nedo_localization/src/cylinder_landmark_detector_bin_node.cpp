#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vehicle_interface/msg/pose_stamped_array.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <numeric>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <string>
#include <tuple>
#include <utility>
#include <fstream>
#include <iomanip>

// ============================================================
//  cylinder_landmark_detector_bin.cpp (v4: 2D XY グループ方式)
//
//  再帰反射円柱ランドマーク検出器。物理仕様: 上下端アンカー30cm赤 + 中間ビン
//  30cm×4 (5cm余白+20cm反射材+5cm余白, 赤=反射有) + 横幅40cm。全高1.80m、
//  検出期待Z幅 expected_dist=1.70m (=全高−2×BIN_INNER_MARGIN, 端取りこぼし補正)。
//
//  処理: 高強度点をXY平面でBFSグループ化 → Zスパンで候補絞り → 上端アンカー基準で
//  ビン境界を算出 → 各ビンの高強度割合でパターン生成 → 辞書照合(ハミング距離) →
//  同一円柱の重複を z_span で除去 → 時間ヒステリシスで1フレーム誤識別を抑制。
// ============================================================

namespace Params
{
  // ============================================================
  //  ランドマーク物理仕様
  // ============================================================
  constexpr float ANCHOR_HEIGHT = 0.30f;       // 上下端アンカーの高さ [m]
  constexpr float BIN_HEIGHT    = 0.30f;       // 中間ビンの高さ      [m]
  constexpr int   BIN_COUNT     = 4;           // 中間ビンの数
  // 実物仕様: 中間ビン 4 個 (各30cm = 5cm余白 + 20cm反射材 + 5cm余白)
  // 全4ビットが識別情報。偶数パリティコード採用(全ペアHD≥2保証)。
  // ランドマーク全高 = 2×0.30 + 4×0.30 = 1.80 m
  // expected_dist     = 1.80 − 2×BIN_INNER_MARGIN = 1.70 m
  constexpr float LANDMARK_WIDTH = 0.40f;      // 横幅 [m]

  // 高さ判定の許容幅 (非対称)
  // z_span が小さい方向(圧縮): 遠距離・30°角で点群が疎になりz_spanが縮む → 40cm 許容
  // z_span が大きい方向(拡張): 偽グループ混入を防ぐため 15cm を維持
  // スケール適応式では z_span が物理 1.80m に近いほど推定が正確になる。
  // z_span がアンカー1個分(0.30m)以上短い場合は「アンカー欠落 + bin 反射に
  // 寄生」状態の可能性が高く、シフト誤識別のリスク源になる。
  // LOW 側を 0.15 に絞り、新LiDAR の実測圧縮 (~0.18m) は許容する。
  constexpr float HEIGHT_TOLERANCE_LOW  = 0.15f;
  constexpr float HEIGHT_TOLERANCE_HIGH = 0.15f;

  // ============================================================
  //  強度フィルター (_gauss2 同等)
  // ============================================================
  constexpr float INTENSITY_RATIO   = 0.80f;
  constexpr float INTENSITY_ABS_MIN = 160.0f;

  // ============================================================
  //  2D XY グループ化
  //
  //  3D BFS クラスタリングを廃止し、XY平面のみで点をグループ化する。
  //  同一円柱の全テープゾーン(アンカー+ビン)が自動的に1グループに入る。
  //  クラスタリング距離・ストライプ高さフィルタに依存しないため、
  //  0°正面視の疎な点群でも各ビンの点が全てカウントされる。
  // ============================================================
  constexpr float GROUP_XY_THRESHOLD = 0.30f;   // XY BFS マージ距離 [m]
  constexpr int   MIN_GROUP_POINTS   = 3;        // グループ最小点数(ノイズ除去)

  // ============================================================
  //  ビン判定 (案A: 物理範囲内の高強度点数カウント方式)
  //
  //  各ビンの判定は以下の流れ:
  //    1) 円柱XY中心 = XY群の重心
  //    2) 各点 p について:
  //         XY距離 ≤ CYLINDER_XY_RADIUS かつ そのビン Z 範囲内 なら加算
  //    3) point_count[k] >= BIN_MIN_POINTS なら pattern[k] = 1
  //
  //  CYLINDER_XY_RADIUS:
  //    円柱表面までのXY距離の許容上限 [m]
  //    横幅40cmなので半径20cm。広げすぎると隣接物体の点も拾うため、
  //    近接物体がある環境では狭く取る必要がある。
  //
  //  BIN_MIN_POINTS:
  //    そのビンを「赤反射材あり」と判定するための最小点数。
  //    強度フィルタにより孤立ノイズは既に排除されているため 1 で運用。
  //    遠距離・0°角でテープゾーン点数が少ない場合の検出率を優先。
  // ============================================================
  constexpr float CYLINDER_XY_RADIUS = 0.20f;
  constexpr int   BIN_MIN_POINTS     = 1;

  // ============================================================
  //  アンカー漏れ込み対策
  //
  //  実物パターンは「アンカー直後の20cmは必ず白」設計。
  //  しかし素朴な実装では、ビン[0]のZ範囲下端 = アンカー上端 となり、
  //  アンカー上端付近の高強度点(スキャン揺らぎでビン境界に乗る点)が
  //  ビン[0]に流入し、本来0であるビン[0]が 1 誤判定される。
  //  同様にビン[BIN_COUNT-1]が上端アンカーから漏れ込む。
  //
  //  対策: 各ビンの両端から 5cm 内側 (= 物理的な余白幅) のみを
  //        反射材有効領域として点数を集計する。
  // ============================================================
  constexpr float BIN_INNER_MARGIN  = 0.05f;

  // ============================================================
  //  パターンマッチング
  // ============================================================
  constexpr int HAMMING_MAX_DIST = 1;
}

// ============================================================
//  既知パターン辞書 (BIN_COUNT = 4、全4ビットが識別情報)
//
//  偶数パリティコード: 全コードワードのビット和が偶数
//  → 全28ペアでハミング距離 ≥ 2 が保証される (最小HD=2)
//  → 1ビット誤り時は必ず ambiguous or 候補外 になり誤IDを防ぐ
//
//  辞書ペア間ハミング距離 (最小 = 2):
//    BIN_1 {0000} BIN_2 {1100} BIN_3 {0110} BIN_4 {0011}
//    BIN_5 {1010} BIN_6 {0101} BIN_7 {1001} BIN_8 {1111}
//
//  上下反転による別ID化リスク:
//    BIN_5 {1010} ↔ BIN_6 {0101}  ←完全反転関係
//    BIN_4 {0011} ↔ BIN_2 {1100}  ←完全反転関係
//    BIN_1, BIN_3, BIN_7, BIN_8 は反転に対して対称(安全)
//
//    通常運用では z 座標で上下確定するので問題ない。
// ============================================================
const std::map<std::string, std::vector<int>> KNOWN_BIN_PATTERNS = {
  // BIN_1 {0,0,0,0} 削除: 反射なし=マーカーではない (2026-07-02, OT128 誤識別対策)
  {"BIN_2", {1, 1, 0, 0}},
  {"BIN_3", {0, 1, 1, 0}},
  {"BIN_4", {0, 0, 1, 1}},
  {"BIN_5", {1, 0, 1, 0}},
  {"BIN_6", {0, 1, 0, 1}},
  {"BIN_7", {1, 0, 0, 1}},
  // BIN_8 {1,1,1,1} 削除: 全埋は虚像込みクラスタの偽マッチが多発 (2026-07-02, 482件)
};

// ============================================================
//  BIN_n → marker_n マッピング (自己位置推定地図との対応)
//  params.yaml の LM_ids に登録される名前と一致させること。
//  配置の変更は params.yaml 側で行い、ここはBINパターン↔ID名の
//  一対一対応のみを定義する。
// ============================================================
const std::map<std::string, std::string> BIN_TO_MARKER_ID = {
  // BIN_1 削除 (2026-07-02): パターン辞書と対応
  {"BIN_2", "marker_2"},
  {"BIN_3", "marker_3"},
  {"BIN_4", "marker_4"},
  {"BIN_5", "marker_5"},
  {"BIN_6", "marker_6"},
  {"BIN_7", "marker_7"},
  // BIN_8 削除 (2026-07-02): パターン辞書と対応
};

struct Centroid { float x, y, z; };
struct Point4 { float x, y, z, intensity; };

// ============================================================
//  地面平面推定 + 点群水平化 (案C, 2026-08-10)
//
//  【背景】検出器の z ロジックは全て入力フレームの生 z で走るため、センサが傾いていると
//  地面/マーカー足元/床反射虚像の z が距離とともに動き、固定 z 閾値では全距離を通せない
//  (20260806 の 4bag で z_min スイープ実測。傾き 1.30〜1.69°)。TF は publish 直前の
//  剛体変換にしか効かないので、static TF を直しても検出率は 1 フレームも変わらない。
//
//  【方法】方位 az_bin × 距離 range_bin のセルごとに最下点を取り (= 地面候補)、
//  z = a·x + b·y + c を反復重み付け最小二乗でフィット。
//  ★重み幅は広い所から段階的に絞る (graduated non-convexity)。等重みから いきなり
//    inlier 幅 0.10m で始めると、壁/マーカー/床反射虚像を拾ったセルに引かれて収束先が壊れる
//    (20260806 15m フレームで傾き 1.56° を 9.9° と誤推定するのを実測)。
//  最後に inlier だけのハードトリム再フィットを 3 回。これで 20260806 の 4bag について
//  オフライン参照値との傾き差 ≤0.08°、搭載高 1.945〜1.975m、inlier 残差 RMS 1.7〜2.0cm。
//  IMU は使わない (Livox/Hesai 内蔵とも不信、[[avia-imu-unreliable]])。
//
//  ★採否ゲートは inlier 残差 (max_resid_m=0.025) が主。20260727 bag の前半のように地面が
//    十分見えていないフレームでは残差が 2.6〜3.0cm に上がり、傾きを 1.0°→2〜3° と誤推定して
//    位置ばらつきが σxy 2.98→7.56mm に悪化した (2026-08-10 実測)。残差 0.025m で切ると
//    その 132/450 フレームが不採用 → 直近平面を保持 → σxy 2.98mm (水平化前と同等) に戻る。
//    健全なフレームの残差は 20260806 の 4bag で 1.6〜2.1cm、20260727 後半で 1.7〜2.0cm。
//  ★min_cells は 60 のまま (200 に上げると再生開始直後の健全フレームまで落ち、直近平面が
//    まだ無いので固定 z フォールバック = 出力座標系が水平化前後で切り替わり σ が跳ねる)。
//
//  【出力座標系】水平化が有効な間、publish される検出位置は必ず「水平化後の系」
//  (= 地面法線を +z に合わせた系。センサ系から高々 1〜2° の回転)。有効な平面が
//  一度も得られていない起動直後だけは系が確定しないので、そのフレームは出力しない
//  (`ground_level_require_plane`、既定 true)。false にすると固定 z フォールバックで
//  「水平化前の系」の検出が同じトピックに混ざるので、地図作成/照合には使わないこと。
// ============================================================
struct GroundPlane {
  bool  valid = false;
  float a = 0.0f, b = 0.0f, c = 0.0f;  // z = a·x + b·y + c
  float tilt_deg = 0.0f;               // 法線と +z のなす角
  float resid_m  = 0.0f;               // inlier 残差 RMS
  int   n_cells  = 0;
};

static GroundPlane fitGroundPlane(const std::vector<Point4>& pts,
                                  float az_bin_deg, float range_bin_m,
                                  float range_min_m, float range_max_m,
                                  int min_cells, int iters, float inlier_m, float trim_m,
                                  float max_tilt_deg, float max_resid_m)
{
  GroundPlane gp;
  if (az_bin_deg <= 0.0f || range_bin_m <= 0.0f || range_max_m <= range_min_m) return gp;
  constexpr float DEG = 180.0f / static_cast<float>(M_PI);
  const int n_az = std::max(1, static_cast<int>(std::lround(360.0f / az_bin_deg)));
  const int n_rg = std::max(1, static_cast<int>(
                     std::ceil((range_max_m - range_min_m) / range_bin_m)));

  // セルごとの最下点 (地面候補)。密なので配列で持つ。
  struct Cell { float z; float x, y; bool used; };
  std::vector<Cell> cells(static_cast<size_t>(n_az) * n_rg,
                          Cell{0.0f, 0.0f, 0.0f, false});
  for (const auto& p : pts) {
    const float r = std::sqrt(p.x * p.x + p.y * p.y);
    if (r < range_min_m || r >= range_max_m) continue;
    float az = std::atan2(p.y, p.x) * DEG;      // (-180, 180]
    if (az < 0.0f) az += 360.0f;
    int ia = static_cast<int>(az / az_bin_deg);
    if (ia >= n_az) ia = n_az - 1;
    int ir = static_cast<int>((r - range_min_m) / range_bin_m);
    if (ir >= n_rg) ir = n_rg - 1;
    Cell& cl = cells[static_cast<size_t>(ia) * n_rg + ir];
    if (!cl.used || p.z < cl.z) { cl.z = p.z; cl.x = p.x; cl.y = p.y; cl.used = true; }
  }

  std::vector<Cell> obs;
  obs.reserve(cells.size());
  for (const auto& cl : cells) { if (cl.used) obs.push_back(cl); }
  gp.n_cells = static_cast<int>(obs.size());
  if (gp.n_cells < min_cells) return gp;

  // 重み付き最小二乗 1 回分 (正規方程式 3x3 を Cramer で解く)。false=特異
  double a = 0.0, b = 0.0, c = 0.0;
  auto solve = [&](const std::vector<double>& w) -> bool {
    double m00=0, m01=0, m02=0, m11=0, m12=0, m22=0, v0=0, v1=0, v2=0;
    for (size_t i = 0; i < obs.size(); i++) {
      const double x = obs[i].x, y = obs[i].y, z = obs[i].z, wi = w[i];
      m00 += wi*x*x; m01 += wi*x*y; m02 += wi*x;
      m11 += wi*y*y; m12 += wi*y;   m22 += wi;
      v0  += wi*x*z; v1  += wi*y*z; v2  += wi*z;
    }
    const double det =
        m00*(m11*m22 - m12*m12) - m01*(m01*m22 - m12*m02) + m02*(m01*m12 - m11*m02);
    if (std::abs(det) < 1e-9) return false;
    a = ( v0*(m11*m22 - m12*m12) - m01*(v1*m22 - m12*v2) + m02*(v1*m12 - m11*v2)) / det;
    b = ( m00*(v1*m22 - m12*v2) - v0*(m01*m22 - m12*m02) + m02*(m01*v2 - v1*m02)) / det;
    c = ( m00*(m11*v2 - v1*m12) - m01*(m01*v2 - v1*m02) + v0*(m01*m12 - m11*m02)) / det;
    return true;
  };

  // ① 段階的 IRLS: 重み幅を 2.0m から幾何級数で inlier_m まで絞る。
  //    いきなり狭い幅で始めると壁/マーカー/床反射虚像に引かれて壊れる (実測)。
  std::vector<double> w(obs.size(), 1.0);
  for (int it = 0; it < std::max(1, iters); it++) {
    if (!solve(w)) return gp;
    const double s = std::max(static_cast<double>(inlier_m), 2.0 * std::pow(0.6, it));
    const double s2 = s * s;
    for (size_t i = 0; i < obs.size(); i++) {
      const double r = obs[i].z - (a*obs[i].x + b*obs[i].y + c);
      w[i] = s2 / (s2 + r*r);
    }
  }
  // ② ハードトリム再フィット: |残差| < trim_m のセルだけを等重みで 3 回
  int n_inl = 0;
  for (int it = 0; it < 3; it++) {
    n_inl = 0;
    for (size_t i = 0; i < obs.size(); i++) {
      const double r = obs[i].z - (a*obs[i].x + b*obs[i].y + c);
      w[i] = (std::abs(r) < trim_m) ? 1.0 : 0.0;
      if (w[i] > 0.0) n_inl++;
    }
    if (n_inl < min_cells) break;
    if (!solve(w)) return gp;
  }

  // inlier 残差 RMS (外れ値を含めない = 地面のばらつきそのもの)
  double sw = 0.0, sr2 = 0.0;
  n_inl = 0;
  for (size_t i = 0; i < obs.size(); i++) {
    const double r = obs[i].z - (a*obs[i].x + b*obs[i].y + c);
    if (std::abs(r) < trim_m) { sw += 1.0; sr2 += r*r; n_inl++; }
  }
  gp.a = static_cast<float>(a);
  gp.b = static_cast<float>(b);
  gp.c = static_cast<float>(c);
  gp.n_cells  = n_inl;
  gp.resid_m  = (sw > 0.0) ? static_cast<float>(std::sqrt(sr2 / sw)) : 0.0f;
  gp.tilt_deg = static_cast<float>(std::atan(std::sqrt(a*a + b*b)) * DEG);
  // 妥当性チェック: inlier が少ない / 傾きすぎ / 残差が大きすぎる (地面が見えていない) なら不採用
  if (n_inl < min_cells || gp.tilt_deg > max_tilt_deg || gp.resid_m > max_resid_m) return gp;
  gp.valid = true;
  return gp;
}

// 地面法線 n ∝ (-a, -b, 1) を +z に合わせる最小回転 (Rodrigues)。R は行優先 3x3。
static void levelRotation(float a, float b, float R[9])
{
  const float nn = std::sqrt(a*a + b*b + 1.0f);
  const float nx = -a / nn, ny = -b / nn, nz = 1.0f / nn;
  // v = n × (0,0,1), s = |v|, c = n·(0,0,1) = nz
  const float vx = ny, vy = -nx, vz = 0.0f;
  const float s2 = vx*vx + vy*vy + vz*vz;
  if (s2 < 1e-18f) {  // 既に水平
    R[0]=1; R[1]=0; R[2]=0; R[3]=0; R[4]=1; R[5]=0; R[6]=0; R[7]=0; R[8]=1;
    return;
  }
  const float k = (1.0f - nz) / s2;
  // R = I + K + K^2·k  (K = [v]_x)
  R[0] = 1.0f + k*(-vy*vy);      R[1] = -vz + k*(vx*vy);    R[2] =  vy + k*(vx*vz);
  R[3] =  vz + k*(vx*vy);        R[4] = 1.0f + k*(-vx*vx);  R[5] = -vx + k*(vy*vz);
  R[6] = -vy + k*(vx*vz);        R[7] =  vx + k*(vy*vz);    R[8] = 1.0f + k*(-vx*vx - vy*vy);
}

// ============================================================
//  形状計測ヘルパー (2026-07-08、計測のみ・棄却には未使用)
//
//  グループ高強度点を XY 平面に投影し、円柱前面弧の形状を数値化する。
//  ゴースト弁別 (エッジ/平面) の候補特徴だが、閾値化は実データ分布を
//  BINCSV で貯めてから行う段階なので、ここでは値を出すだけ。
//   - arc_width : 最大連続方位ラン(0.2°ビン,隙間<=2)の物理幅 [m]
//                 (本物円柱≈0.37-0.44、エッジ=細い、平面sprawl=太い)
//   - r_est     : そのラン点への Kåsa 円フィット半径 [m]
//                 (本物円柱≈0.12-0.19、平面=直線→大。ただし疎な弧で不安定)
//   - arc_npts  : ラン内点数 (r_est の信頼度指標。少ないと円フィットが暴れる)
// ============================================================
//   - knownr_rms: 既知半径 R を固定した円フィットの残差 RMS (2026-08-10 追加、計測専用・ゲートなし)
//     r_est は「半径いくつに見えるか」を 3 自由度で推定するので、遠方で方位列が 3〜5 本に
//     なると測距ノイズに支配される (20260721 実測: 35-41m で r_est の p10 が 0.143 まで落ち、
//     下限 0.14 に接触。中央値は 0.19 で距離不変)。半径を固定すると自由度が 2 に減るので
//     条件が良く、遠方でも劣化しにくいはず — という仮説を検証するための計測列。
struct ArcFeat { float arc_width; float r_est; int arc_npts; float sagitta;
                 float knownr_rms; int knownr_n; };
// 既知半径円フィット (定義は下方)。計測列 knownr_rms のために前方宣言する。
static void losOffset(float cx, float cy, float R, float& out_cx, float& out_cy);
static bool fitCircleKnownR(const std::vector<int>& grp, const std::vector<Point4>& pts,
                            float init_cx, float init_cy, float R, int max_iter,
                            float& out_cx, float& out_cy, float& out_rms);
// window_lo: 形状特徴を計算する z の下限 (= 復号窓の下端)。窓を使わない場合は -inf を渡す。
// known_r: 既知半径円フィット用の半径 (計測列 knownr_rms 用)。
//
// 【なぜ窓で絞るか (2026-08-10, 案B)】
// 復号 (z_span / ビン読取) は decode_top_referenced の窓 [z_high-expected-tol, z_high] で
// 床反射虚像から守られているのに、形状特徴だけがグループ全点を食っていた。この非対称のせいで
// 地面フィルタを外すと 20m の本物マーカーで群点数 105→142 に増え、r_est が 0.167→0.128 と
// 曲率ゲート下限 0.14 を割って BIN_4 が NONE に落ちる (20260806 20m bag で対照実測)。
// 増えた点は演繹的に必ず窓の外 (窓下端 -1.3875 > 地面フィルタ切断 -1.40)。
// → 「復号に使う点集合と形状判定に使う点集合を一致させる」ことで、
//    地面フィルタの設定に形状ゲートが依存しなくなる。
static ArcFeat computeArcFeatures(const std::vector<int>& grp,
                                  const std::vector<Point4>& pts,
                                  float window_lo,
                                  float known_r)
{
  ArcFeat out{0.0f, 0.0f, 0, 0.0f, -1.0f, 0};
  std::vector<int> win;
  win.reserve(grp.size());
  for (int idx : grp) { if (pts[idx].z >= window_lo) win.push_back(idx); }
  const size_t m = win.size();
  if (m < 3) return out;
  constexpr float RAD = static_cast<float>(M_PI) / 180.0f;
  constexpr float BIN = 0.2f;  // 方位ビン幅 [deg]
  // (方位[deg], win内index) を方位でソート
  std::vector<std::pair<float, int>> az;
  az.reserve(m);
  for (size_t i = 0; i < m; i++) {
    const Point4& p = pts[win[i]];
    az.emplace_back(std::atan2(p.y, p.x) / RAD, static_cast<int>(i));
  }
  std::sort(az.begin(), az.end());
  // 隙間 >2 ビンで分割した連続ランのうち、方位スパン最大を採用
  const float az0 = az.front().first;
  int run_s = 0, best_s = 0, best_e = 0;
  float best_span = -1.0f;
  int prev_bin = static_cast<int>(std::floor((az.front().first - az0) / BIN));
  for (size_t i = 1; i <= az.size(); i++) {
    bool brk = (i == az.size());
    if (!brk) {
      const int b = static_cast<int>(std::floor((az[i].first - az0) / BIN));
      if (b - prev_bin > 2) brk = true; else prev_bin = b;
    }
    if (brk) {
      const int e = static_cast<int>(i) - 1;
      const float span = az[e].first - az[run_s].first;
      if (span > best_span) { best_span = span; best_s = run_s; best_e = e; }
      if (i < az.size()) {
        run_s = static_cast<int>(i);
        prev_bin = static_cast<int>(std::floor((az[i].first - az0) / BIN));
      }
    }
  }
  const int npts = best_e - best_s + 1;
  out.arc_npts = npts;
  if (npts < 3) return out;
  double mean_rng = 0.0;
  for (int k = best_s; k <= best_e; k++) {
    const Point4& p = pts[win[az[k].second]];
    mean_rng += std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y);
  }
  mean_rng /= npts;
  out.arc_width = static_cast<float>(mean_rng) * best_span * RAD;
  // Kåsa 代数的円フィット: [2x 2y 1][a b c]^T = x^2+y^2 → R=sqrt(c+a^2+b^2)
  double Sxx=0, Sxy=0, Sx=0, Syy=0, Sy=0, S1=0, Sxz=0, Syz=0, Sz=0;
  for (int k = best_s; k <= best_e; k++) {
    const Point4& p = pts[win[az[k].second]];
    const double X = 2.0 * p.x, Y = 2.0 * p.y;
    const double Z = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y;
    Sxx += X*X; Sxy += X*Y; Sx += X;
    Syy += Y*Y; Sy += Y; S1 += 1.0;
    Sxz += X*Z; Syz += Y*Z; Sz += Z;
  }
  const double det =
      Sxx*(Syy*S1 - Sy*Sy) - Sxy*(Sxy*S1 - Sy*Sx) + Sx*(Sxy*Sy - Syy*Sx);
  if (std::abs(det) > 1e-9) {
    const double a =
        (Sxz*(Syy*S1 - Sy*Sy) - Sxy*(Syz*S1 - Sy*Sz) + Sx*(Syz*Sy - Syy*Sz)) / det;
    const double b =
        (Sxx*(Syz*S1 - Sy*Sz) - Sxz*(Sxy*S1 - Sy*Sx) + Sx*(Sxy*Sz - Syz*Sx)) / det;
    const double c =
        (Sxx*(Syy*Sz - Syz*Sy) - Sxy*(Sxy*Sz - Syz*Sx) + Sxz*(Sxy*Sy - Syy*Sx)) / det;
    const double r2 = c + a*a + b*b;
    out.r_est = (r2 > 0.0) ? static_cast<float>(std::sqrt(r2)) : 0.0f;
  }
  // 既知半径フィットの残差 RMS (計測のみ。r_est と同じ弧の点集合で測る)
  if (known_r > 0.0f) {
    std::vector<int> arc_idx;
    arc_idx.reserve(npts);
    double mx = 0.0, my = 0.0;
    for (int k = best_s; k <= best_e; k++) {
      const int idx = win[az[k].second];
      arc_idx.push_back(idx);
      mx += pts[idx].x; my += pts[idx].y;
    }
    mx /= npts; my /= npts;
    float sx, sy, kx, ky, krms;
    losOffset(static_cast<float>(mx), static_cast<float>(my), known_r, sx, sy);
    if (fitCircleKnownR(arc_idx, pts, sx, sy, known_r, 10, kx, ky, krms)) {
      out.knownr_rms = krms;
      out.knownr_n   = npts;
    }
  }
  // サジッタ (半径方向の奥行き = 全高強度点 XY 主軸の短軸標準偏差)。
  // 平面反射板 (看板/パネル) ≈ 0.005-0.015m、円柱前面弧は 0.04-0.11m。
  // r_est(半径) は直線的な点列にも円を通すため奥行きを測れないが、これは測れる。
  {
    double mx = 0.0, my = 0.0;
    for (size_t i = 0; i < m; i++) { mx += pts[win[i]].x; my += pts[win[i]].y; }
    mx /= m; my /= m;
    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for (size_t i = 0; i < m; i++) {
      const double dx = pts[win[i]].x - mx, dy = pts[win[i]].y - my;
      cxx += dx*dx; cxy += dx*dy; cyy += dy*dy;
    }
    cxx /= m; cxy /= m; cyy /= m;
    const double half = (cxx - cyy) * 0.5;
    const double disc = std::sqrt(std::max(0.0, half*half + cxy*cxy));
    const double lam_min = (cxx + cyy) * 0.5 - disc;  // 短軸固有値
    out.sagitta = (lam_min > 0.0) ? static_cast<float>(std::sqrt(lam_min)) : 0.0f;
  }
  return out;
}

// LOS押し出し: 生重心を LiDAR 原点から半径 R だけ遠方へ動かす。
// known_r の初期値/フォールバック専用 (publish 方式としては露出しない)。
// 重心は円弧上=真の軸中心の約R手前にあるため、これで種を真中心近傍に寄せ、
// Gauss-Newton が手前側(ミラー解)に落ちるのを防ぐ。
static inline void losOffset(float cx, float cy, float R, float& out_cx, float& out_cy)
{
  const float d = std::sqrt(cx * cx + cy * cy);
  if (d < 1e-6f) { out_cx = cx; out_cy = cy; return; }
  const float k = (d + R) / d;
  out_cx = cx * k;
  out_cy = cy * k;
}

// ============================================================
//  既知半径 R 固定の Gauss-Newton 円フィット (publish 用 XY 中心の精緻化)
//   残差 f_i = sqrt((x_i-cx)^2+(y_i-cy)^2) - R
//   ヤコビ df/dcx=-(x_i-cx)/d_i, df/dcy=-(y_i-cy)/d_i
//   正規方程式 (2x2) J^T J·δ = -J^T f を解析逆行列で解く。反復2〜3回で収束。
//   init_cx/cy は los_offset(重心をR押し出した点)を渡す。out_rms は最終残差 RMS。
// ============================================================
static bool fitCircleKnownR(const std::vector<int>& grp,
                            const std::vector<Point4>& pts,
                            float init_cx, float init_cy, float R,
                            int max_iter,
                            float& out_cx, float& out_cy, float& out_rms)
{
  const int n = (int)grp.size();
  if (n < 3) return false;
  float cx = init_cx, cy = init_cy;
  float last_rms = 0.0f;
  for (int it = 0; it < max_iter; it++) {
    double H00 = 0, H01 = 0, H11 = 0, g0 = 0, g1 = 0, sse = 0;
    for (int idx : grp) {
      double dx = pts[idx].x - cx, dy = pts[idx].y - cy;
      double d  = std::sqrt(dx * dx + dy * dy);
      if (d < 1e-6) continue;
      double f = d - R, jx = -dx / d, jy = -dy / d;
      H00 += jx * jx; H01 += jx * jy; H11 += jy * jy;
      g0  += jx * f;  g1  += jy * f;  sse += f * f;
    }
    last_rms = (float)std::sqrt(sse / n);
    double det = H00 * H11 - H01 * H01;
    if (std::abs(det) < 1e-12) return false;
    double inv = 1.0 / det;
    double dcx = -inv * ( H11 * g0 - H01 * g1);
    double dcy = -inv * (-H01 * g0 + H00 * g1);
    cx += (float)dcx; cy += (float)dcy;
    if (std::abs(dcx) < 1e-5 && std::abs(dcy) < 1e-5) break;
  }
  out_cx = cx; out_cy = cy; out_rms = last_rms;
  return true;
}

// ============================================================
//  ノード本体
// ============================================================
class CylinderLandmarkDetectorBin : public rclcpp::Node
{
public:
  CylinderLandmarkDetectorBin() : Node("cylinder_landmark_detector_bin")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ============================================================
    // 入出力トピック / フレームのパラメータ化
    //   デフォルトは Hesai OT128 (実機 @192.168.1.201):
    //     input_topic      : "/hesai_ot128/lidar_points"
    //     input_frame_id   : "hesai_ot128"
    //   Hesai QT128 に切替える場合は launch/yaml で上書き:
    //     input_topic      : "/hesai_qt128/lidar_points"
    //     input_frame_id   : "hesai_qt128"
    //   Livox Avia (運用終了) に戻す場合:
    //     input_topic      : "/livox/lidar_3JEDM4V001DY881"
    //     input_frame_id   : "livox_frame"
    //   ※ 通常は bin/ekf_bin_localization.launch.py が sensor_type に応じて
    //      自動で渡すため、ここを直接書き換える必要はない。
    //   ※ static_transform_publisher で <input_frame_id> -> base_link を
    //      張っておくこと
    // ============================================================
    input_topic_     = this->declare_parameter<std::string>(
      "input_topic", "/hesai_ot128/lidar_points");
    output_topic_    = this->declare_parameter<std::string>(
      "output_topic", "/matched_marker");
    input_frame_id_  = this->declare_parameter<std::string>(
      "input_frame_id", "hesai_ot128");
    output_frame_id_ = this->declare_parameter<std::string>(
      "output_frame_id", "base_link");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();  // publisher用
    // 入力点群の購読QoS。実機は best_effort (既定, リアルタイムで古いフレームを捨てる)。
    // オフライン解析で bag(reliable offer) を取りこぼしなく全フレーム処理したい場合のみ
    // input_qos_reliable:=true で reliable にする (巨大クラウド~3.2MBのDDSドロップを解消)。
    const bool input_qos_reliable = this->declare_parameter("input_qos_reliable", false);
    auto sub_qos = input_qos_reliable
        ? rclcpp::QoS(rclcpp::KeepLast(50)).reliable()
        : rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sub_qos,
      std::bind(&CylinderLandmarkDetectorBin::callback, this,
                std::placeholders::_1));

    pub_poses_  = create_publisher<geometry_msgs::msg::PoseArray>(
      "/cylinder_landmark_poses", 10);
    // /matched_marker (vehicle_interface/PoseStampedArray)
    //   header.frame_id     = base_link (TF変換済み)
    //   各 pose.header.frame_id = marker_n (IDタグとして使用 = localizer規約)
    pub_matched_ = create_publisher<vehicle_interface::msg::PoseStampedArray>(
      output_topic_, qos);
    // /landmark_ids (std_msgs/String): /matched_marker の配列順と1対1対応する
    //   marker_n のカンマ区切りリスト。`ros2 topic echo` での目視確認用。
    //   例: 検出が array=[pose0(marker_3), pose1(marker_5)] のとき
    //       data="marker_3,marker_5"
    pub_ids_ = create_publisher<std_msgs::msg::String>("/landmark_ids", 10);

    // z_span ゲート許容幅 (実行時スイープ用)。30°視では z_span が圧縮されるため
    // LOW 側を広げて本物の群を回復する。重心方式グリッドが実アンカー位置に適応するので
    // 中程度の圧縮でも誤識別を出さずに識別できる。
    this->declare_parameter("height_tol_low",  (double)Params::HEIGHT_TOLERANCE_LOW);
    this->declare_parameter("height_tol_high", (double)Params::HEIGHT_TOLERANCE_HIGH);
    height_tol_low_param_  = (float)this->get_parameter("height_tol_low").as_double();
    height_tol_high_param_ = (float)this->get_parameter("height_tol_high").as_double();

    this->declare_parameter("bin_min_points", Params::BIN_MIN_POINTS);
    bin_min_points_param_ = (int)this->get_parameter("bin_min_points").as_int();

    // ============================================================
    //  実験中の調整用パラメータ (yaml/launchから上書き可)
    //  デフォルトは Params:: の constexpr を流用
    // ============================================================
    anchor_height_param_       = (float)this->declare_parameter("anchor_height",       (double)Params::ANCHOR_HEIGHT);
    bin_height_param_          = (float)this->declare_parameter("bin_height",          (double)Params::BIN_HEIGHT);
    landmark_width_param_      = (float)this->declare_parameter("landmark_width",      (double)Params::LANDMARK_WIDTH);
    intensity_ratio_param_     = (float)this->declare_parameter("intensity_ratio",     (double)Params::INTENSITY_RATIO);
    intensity_abs_min_param_   = (float)this->declare_parameter("intensity_abs_min",   (double)Params::INTENSITY_ABS_MIN);
    group_xy_threshold_param_  = (float)this->declare_parameter("group_xy_threshold",  (double)Params::GROUP_XY_THRESHOLD);
    min_group_points_param_    = (int)  this->declare_parameter("min_group_points",    Params::MIN_GROUP_POINTS);
    // === 距離適応 (2026-07-15) ===
    // (B) クラスタリング距離: eps(r)=clamp(k·r·Δθ_h, floor, group_xy_threshold)。既定OFF。
    group_xy_adaptive_param_  = this->declare_parameter("group_xy_adaptive", true);
    group_xy_k_param_         = (float)this->declare_parameter("group_xy_k", 3.0);
    group_xy_dtheta_h_param_  = (float)this->declare_parameter("group_xy_dtheta_h", 0.00349);
    group_xy_floor_param_     = (float)this->declare_parameter("group_xy_floor", 0.05);
    cylinder_xy_radius_param_  = (float)this->declare_parameter("cylinder_xy_radius",  (double)Params::CYLINDER_XY_RADIUS);
    // publish 用 XY 中心の既知半径円フィット (known_r)。
    // landmark_radius は物理的な円柱外径半径 (外径413mm→0.20675m)。ゲート用 cylinder_xy_radius
    // (許容半径) とは別物なので専用paramで持つ。
    landmark_radius_param_         = (float)this->declare_parameter("landmark_radius",         0.20675);
    cfit_known_r_min_points_param_ = (int)  this->declare_parameter("cfit_known_r_min_points", 4);
    cfit_known_r_max_rms_param_    = (float)this->declare_parameter("cfit_known_r_max_rms",    0.05);
    cfit_known_r_max_iter_param_   = (int)  this->declare_parameter("cfit_known_r_max_iter",   10);
    bin_inner_margin_param_    = (float)this->declare_parameter("bin_inner_margin",    (double)Params::BIN_INNER_MARGIN);
    hamming_max_dist_param_    = (int)  this->declare_parameter("hamming_max_dist",    Params::HAMMING_MAX_DIST);

    // ---- ビン判定を強度で行う (2026-07-16, 点数→強度の観測量変更) ----
    // 点数は垂直サンプリング密度(仰角依存)を測ってしまい、「疎だから少ない」と
    // 「白だから少ない」を区別できない。OT128は垂直FOV中央が密・端が疎なので、
    // 近距離ではマーカー下端が疎な仰角端に落ち、遠距離では圧縮で上端が疎な端に来る
    // → 固定/距離適応いずれの点数閾値でも根治しない。反射材=強度なので、各ビンを
    // 「窓内の点のうち高強度の割合」で判定する(サンプリング不変)。
    //   bin_read_intensity=true : 割合判定 (新, 既定)
    //   false                   : 旧 点数閾値 (bin_min_points) にフォールバック(A/B用)
    // 割合の分母は bin_intensity_radius 内の全強度点。近距離の背景希釈を避けるため
    // cylinder_xy_radius より絞れるよう独立パラメータ化(既定は同値)。
    bin_read_intensity_param_   = this->declare_parameter("bin_read_intensity", true);
    bin_frac_min_param_         = (float)this->declare_parameter("bin_frac_min", 0.5);
    bin_min_total_param_        = (int)  this->declare_parameter("bin_min_total", 2);
    bin_intensity_radius_param_ = (float)this->declare_parameter(
      "bin_intensity_radius", (double)cylinder_xy_radius_param_);

    // ---- BIN 生成方式 ----
    //  top_anchor_only : 上端アンカのみを参照 (床反射虚像に強い、現行デフォルト)
    //  both_anchors    : 上下両端アンカを参照 (scale = z_span/expected でグリッドを伸縮)
    bin_method_param_ = this->declare_parameter("bin_method", std::string("top_anchor_only"));
    if (bin_method_param_ != "top_anchor_only" && bin_method_param_ != "both_anchors") {
      RCLCPP_WARN(get_logger(),
        "未知の bin_method='%s' → 'top_anchor_only' にフォールバック",
        bin_method_param_.c_str());
      bin_method_param_ = "top_anchor_only";
    }

    // ---- 処理時間の区間計測ログ (性能調査用、default off) ----
    perf_log_enable_param_ = this->declare_parameter("perf_log_enable", false);

    // ---- 上端アンカー検出窓 [m] ----
    // top_anchor_only の z_ta 平均窓高さ。z_high から下ろして上端アンカー中心を推定する。
    // 既定は物理アンカー高 anchor_height_param_ (0.30m)。
    top_anchor_detect_window_param_ = (float)this->declare_parameter(
      "top_anchor_detect_window", (double)anchor_height_param_);

    // ---- 地面下フィルタ (z が下限未満の点を捨てる) ----
    //  床反射虚像や地面点の混入対策。水平化(下記)が有効なときは平面基準、
    //  無効/フィット失敗時は固定 z (ground_filter_z_min) にフォールバックする。
    ground_filter_enable_param_ = this->declare_parameter("ground_filter_enable", true);
    ground_filter_z_min_param_  = (float)this->declare_parameter("ground_filter_z_min", -1.40);

    // ---- 点群水平化 (案C, 2026-08-10) ----
    // 検出器の z ロジック(地面下フィルタ/z幅/上端アンカ/ビン境界)は全て入力フレームの生 z で
    // 走るので、センサが傾いていると地面 z が距離とともに動き、固定 z 閾値が原理的に成立しない。
    // 20260806 の 4bag では傾き 1.30〜1.69°(主に roll)・搭載高 1.95m で、
    // 単一の ground_filter_z_min では全距離 100% が不可能だった (z_min スイープで実測:
    // -1.40 → 10m/5m 全滅、-1.70 → 20m id2 58.8%、無効 → 20m 0%)。
    // ここでは毎フレーム地面平面 z = a·x + b·y + c をロバスト推定し、その法線を +z に合わせる
    // 最小回転を点群に掛けてからビン処理に入る。IMU は使わない([[avia-imu-unreliable]])。
    ground_level_enable_param_    = this->declare_parameter("ground_level_enable", true);
    ground_level_az_bin_deg_param_    = (float)this->declare_parameter("ground_level_az_bin_deg", 2.0);
    ground_level_range_bin_m_param_   = (float)this->declare_parameter("ground_level_range_bin_m", 1.0);
    ground_level_range_min_m_param_   = (float)this->declare_parameter("ground_level_range_min_m", 2.0);
    // 30m だと 35-40m のマーカー足元が外挿域になり、地面カットが数 cm ずれて z幅が 1.70→1.51m と
    // 縮んで z幅ゲート下限 1.55 を割る (20260721 で 18 frame 実測)。フィット範囲を伸ばして解決。
    ground_level_range_max_m_param_   = (float)this->declare_parameter("ground_level_range_max_m", 45.0);
    ground_level_min_cells_param_     = (int)this->declare_parameter("ground_level_min_cells", 60);
    ground_level_iters_param_         = (int)this->declare_parameter("ground_level_iters", 10);
    ground_level_inlier_m_param_      = (float)this->declare_parameter("ground_level_inlier_m", 0.10);
    ground_level_trim_m_param_        = (float)this->declare_parameter("ground_level_trim_m", 0.05);
    ground_level_max_tilt_deg_param_  = (float)this->declare_parameter("ground_level_max_tilt_deg", 10.0);
    // inlier 残差の絶対上限 = サニティのみ (主ゲートは下の相対ゲート)。
    // ★絶対値 0.025 はサイト間で転移しないことが 20260721(屋外草地) で判明した:
    //   健全フレームの残差が 20260806(舗装) 0.016-0.021 / 20260727 0.017-0.020 に対し
    //   20260721 は 0.021-0.028 で、0.025 で切ると健全フレームの 3 割を落とす。
    //   一方 20260727 の「地面が見えていない」フレームは 0.026-0.030 で屋外の健全値と重なる。
    //   → 絶対値では両者を区別できない。相対量で見る。
    ground_level_max_resid_m_param_   = (float)this->declare_parameter("ground_level_max_resid_m", 0.05);
    // 相対ゲート: 直近 window 個の採用フレームを基準に「普段より明らかに悪いか」で判定する。
    //   残差   : resid > k_resid × (窓内の最小残差)   → 却下 (地面が荒れて見えていない)
    //   セル数 : cells < k_cells × (窓内の最大セル数) → 却下 (地面が十分見えていない)
    // 20260727 の不良レジームは cells 94-118 (健全 296-305) / resid 0.026-0.030 (健全 0.017-0.020)
    // で、どちらの相対量でも明確に分離する。20260806・20260721 の健全フレームは全て通る。
    ground_level_resid_rel_k_param_   = (float)this->declare_parameter("ground_level_resid_rel_k", 1.5);
    ground_level_cells_rel_k_param_   = (float)this->declare_parameter("ground_level_cells_rel_k", 0.6);
    ground_level_ref_window_param_    = (int)  this->declare_parameter("ground_level_ref_window", 100);
    // 相対ゲートで連続してこの回数却下されたら参照を段階的に緩める (サイトが変わった場合の
    // デッドロック回避)。20260727 の「地面が見えない」区間は 135 フレーム続くので、
    // それより十分長く取る。緩めるまでの間は直近平面を保持するので出力は止まらない。
    ground_level_reacquire_frames_param_ =
      (int)this->declare_parameter("ground_level_reacquire_frames", 600);
    // 平面係数の時間平滑化 (EMA)。1.0=平滑化なし(毎フレームの生フィット)。
    // ※ 20260727 の σ 悪化は EMA の有無ではなく平面の誤推定が原因だった (α=0.3/0.1 に振っても
    //    σxy 7.56→7.30mm しか動かない。高周波成分は σa=0.0035 しかなく、変動の実体は
    //    「地面が見えている/見えていない」の 2 レジーム)。対策は上の残差ゲート。
    // 地面の傾きは車両ダイナミクスの時定数でしか変わらないので平滑化してよいが、
    // α を小さくすると加減速時のピッチ変化に遅れる (静止データでは検出できない副作用)。
    ground_level_lpf_alpha_param_     = (float)this->declare_parameter("ground_level_lpf_alpha", 0.2);
    // 有効な平面が一度も得られていない間 (= 起動直後) の扱い。true=そのフレームは出力しない。
    // false にすると固定 z フォールバックで検出は出るが、それは「水平化前の系」なので
    // 同じトピックに座標系が 2 つ混ざる。既定は true (座標系を 1 つに保つ)。
    ground_level_require_plane_param_ = this->declare_parameter("ground_level_require_plane", true);
    // 水平化成功時の地面下フィルタ: 推定地面高 + margin より下を捨てる (固定 z を使わない)。
    // 20260806 実測の余裕: 地面 -1.96 / マーカー足元 -1.81 / 床反射虚像の天井 -2.11。
    // ★margin は 0.06 → 0.0 (2026-08-10)。床反射虚像の天井は地面より 15cm 下なので
    //   マージン無しでも分離できる一方、遠方ではこの 6cm がマーカー足元を削り
    //   z幅 1.70→1.5m 台 → z幅ゲート棄却になる (20260721 35-40m で id2 226→181 frame)。
    //   0.0 にすると 35-40m が旧同等 (226 frame) に戻り、20260806/20260727 は不変。
    ground_filter_use_plane_param_ = this->declare_parameter("ground_filter_use_plane", true);
    ground_filter_margin_param_    = (float)this->declare_parameter("ground_filter_margin", 0.0);

    // ---- 形状特徴を復号窓に限定 (案B, 2026-08-10。アブレーション用トグル) ----
    arc_window_limit_enable_param_ = this->declare_parameter("arc_window_limit_enable", true);

    // ---- 群点群 CSV ダンプ (2026-08-10, 解析用。空文字なら無効) ----
    // 検出器が実際にビン判定に使った点をそのまま書き出す。座標は水平化後(検出器内部の系)で、
    // 参考として水平化前(センサ系)も x_raw/y_raw/z_raw に付ける。
    group_points_csv_path_param_ = this->declare_parameter("group_points_csv_path", std::string(""));
    if (!group_points_csv_path_param_.empty()) {
      group_csv_.open(group_points_csv_path_param_);
      if (group_csv_.is_open()) {
        group_csv_ << "frame_idx,stamp_sec,group_idx,cx,cy,dist_xy_m,n_points,z_low,z_high,z_span,"
                      "pattern,id_before,id_after,reject_reason,leveled,ground_z,tilt_deg,"
                      "x,y,z,intensity,x_raw,y_raw,z_raw,r_from_center_m\n";
        group_csv_ << std::fixed << std::setprecision(4);
        RCLCPP_INFO(get_logger(), "[群点群CSV] %s に出力します",
                    group_points_csv_path_param_.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "[群点群CSV] %s を開けませんでした",
                     group_points_csv_path_param_.c_str());
      }
    }

    // ---- 曲率ゲート (2026-07-17: サジッタ置換) ----
    // r_est(Kåsa円フィット半径)=物理シリンダ半径で距離不変(20260703実測: 本物中央値0.182-0.191を
    // 全距離帯で維持)。サジッタ(短軸σ=見かけ弧深さ)は遠方圧縮で縮み距離適応が要ったが、
    // 半径帯は固定1組で足りる。本物2685件の99.96%が[0.15,0.25]内(帯外1件=0.149)→下限0.14でrecall0。
    // 誤BIN_4は63%(近59%)がr≈0.30-0.47で帯外→棄却。残37%(r≈0.184帯内)は他ゲート併用で処理。
    // ★下限 0.14 の見直し (2026-08-10): 上限 0.35 は「誤BIN_4 の 63% が r≈0.30-0.47」という
    //   実測 FP 根拠があるが、下限 0.14 の根拠は「本物の 99.96% が [0.15,0.25] にあるから
    //   ここで切れば recall 0」= 無害だから、であって FP を落とした実測はない。
    //   20260721(屋外40m)で無害でないことが判明した: 30m 以遠で r_est の裾が方位サンプリング
    //   (40m で方位 3〜5 列)のノイズに支配され、下限が切っている候補は**実質 100% 本物**
    //   (前後フレームで同 ID 同位置が成立)。id2 は 35-40m で 226→144 frame に後退する。
    //   → 下限の役目 (微小ノイズ弧の排除) は既知半径フィット残差ゲートに移す。
    curv_gate_enable_param_      = this->declare_parameter("curv_gate_enable", true);
    r_est_min_param_             = (float)this->declare_parameter("r_est_min", 0.14);
    r_est_max_param_             = (float)this->declare_parameter("r_est_max", 0.35);
    // 下限だけ遠方で緩める。r_est の裾が暴れるのは方位サンプリングが粗くなる遠方に限られ
    // (20260721 実測: 35-41m で arc幅が 0.33〜0.48m に散る。近距離は 0.409±)、
    // 近距離で緩めると別種の候補まで通ってしまう(同 bag の 12-16m で BIN_4 が 19 frame 復活)。
    // → r_est_far_range_m 以遠でのみ r_est_min_far を使う。近距離は従来どおり 0.14 で不変。
    r_est_far_range_m_param_     = (float)this->declare_parameter("r_est_far_range_m", 30.0);
    r_est_min_far_param_         = (float)this->declare_parameter("r_est_min_far", 0.10);
    // 既知半径フィット残差ゲート: 「半径いくつに見えるか」ではなく「既知半径 R の円柱として
    // 説明できるか」を問う。半径固定で自由度 2 なので遠方でも劣化しにくい。
    // 本物の実測 (20260721): 5-10m 中央 0.0135 → 35-41m 中央 0.0256 / p90 0.0316。
    knownr_rms_gate_enable_param_ = this->declare_parameter("knownr_rms_gate_enable", true);
    knownr_rms_max_param_         = (float)this->declare_parameter("knownr_rms_max", 0.05);

    // ---- アブレーション用トグル (2026-07-16, 各ゲートの要否検証) ----
    // ハードコードだった3ゲートを個別に無効化して効果を測れるように。全て既定 true (従来動作)。
    same_id_collision_discard_param_ = this->declare_parameter("same_id_collision_discard", true);
    anchor_guard_enable_param_       = this->declare_parameter("anchor_guard_enable", true);
    dedup_enable_param_              = this->declare_parameter("dedup_enable", true);

    // ---- 上端基準・地面フリー復号 (2026-07-17, 既定=これで運用) ----
    // z_high(=最上反射=マーカー上端)から expected_dist(=1.70m 反射高)下までを本体窓とし、
    // それより下(床反射虚像)を z_span 判定から除外。床/絶対高さ/鏡像を使わず物体自身で閉じる。
    // 20260703実測: 全体recall m7 78.8→97.5%/m2 89.3→97.5%, 遠方0→93%, near非回帰。
    decode_top_referenced_param_ = this->declare_parameter("decode_top_referenced", true);
    decode_window_tol_param_     = (float)this->declare_parameter("decode_window_tol", 0.15);

    // ---- 時間ヒステリシス (1フレーム誤識別ブロック) パラメータ ----
    // 検出器はBINパターンが完全に {0,0,1,1} になった瞬間に BIN_4 を確実な
    // ID として出力するが、遠距離+FOV端で z_low が時折上振れすると 1フレーム
    // だけパターンが {0,0,1,1} に化けて誤った marker_4 を出すケースがある。
    // 直近 N フレームの履歴で多数決を取り、現フレームのIDが多数派と異なる
    // 場合は publish を抑止する。安定検出中は素通し。
    hysteresis_enable_param_         = this->declare_parameter("hysteresis_enable", true);
    hysteresis_window_param_         = (int)this->declare_parameter("hysteresis_window", 3);
    hysteresis_min_agree_param_      = (int)this->declare_parameter("hysteresis_min_agree", 2);
    hysteresis_track_radius_param_   = (float)this->declare_parameter("hysteresis_track_radius", 0.5);
    hysteresis_expire_frames_param_  = (int)this->declare_parameter("hysteresis_expire_frames", 5);
    hysteresis_warmup_frames_param_  = (int)this->declare_parameter("hysteresis_warmup_frames", 2);

    // 整合性検証: min_agree > window だと多数決が永遠に成立せず全フレーム
    // 抑止になるため、不正な組合せは警告してクランプする
    if (hysteresis_window_param_ < 1) {
      RCLCPP_WARN(get_logger(), "hysteresis_window=%d は不正 → 1 にクランプ",
                  hysteresis_window_param_);
      hysteresis_window_param_ = 1;
    }
    if (hysteresis_min_agree_param_ < 1) {
      RCLCPP_WARN(get_logger(), "hysteresis_min_agree=%d は不正 → 1 にクランプ",
                  hysteresis_min_agree_param_);
      hysteresis_min_agree_param_ = 1;
    }
    if (hysteresis_min_agree_param_ > hysteresis_window_param_) {
      RCLCPP_WARN(get_logger(),
        "hysteresis_min_agree(%d) > window(%d): 多数決が成立不能 → min_agree=%d にクランプ",
        hysteresis_min_agree_param_, hysteresis_window_param_,
        hysteresis_window_param_);
      hysteresis_min_agree_param_ = hysteresis_window_param_;
    }

    const float expected_dist =
      2.0f * anchor_height_param_ + Params::BIN_COUNT * bin_height_param_
      - 2.0f * bin_inner_margin_param_;

    RCLCPP_INFO(get_logger(), "ランドマーク検出ノード起動 (ビン方式 v4: 2D XY グループ化)");
    RCLCPP_INFO(get_logger(),
      "  [I/O] in=%s (frame=%s) out=%s (frame=%s)",
      input_topic_.c_str(), input_frame_id_.c_str(),
      output_topic_.c_str(), output_frame_id_.c_str());
    RCLCPP_INFO(get_logger(),
      "  [仕様] アンカー高=%.2fm ビン高=%.2fm ビン数=%d 横幅=%.2fm",
      anchor_height_param_, bin_height_param_,
      Params::BIN_COUNT, landmark_width_param_);
    RCLCPP_INFO(get_logger(),
      "  [強度フィルタ] 比率=%.2f 絶対最小=%.1f",
      intensity_ratio_param_, intensity_abs_min_param_);
    RCLCPP_INFO(get_logger(),
      "  [XY群] 結合距離=%.2fm 最小点数=%d",
      group_xy_threshold_param_, min_group_points_param_);
    RCLCPP_INFO(get_logger(),
      "  [ビン判定] 円柱XY半径=%.2fm 最小点数=%d 内側余白=%.3fm 方式=%s",
      cylinder_xy_radius_param_, bin_min_points_param_,
      bin_inner_margin_param_, bin_method_param_.c_str());
    RCLCPP_INFO(get_logger(),
      "  [ビン読取] %s (強度割合下限=%.2f 窓内最小点数=%d 割合半径=%.2fm)",
      bin_read_intensity_param_ ? "強度割合" : "点数閾値(旧)",
      bin_frac_min_param_, bin_min_total_param_, bin_intensity_radius_param_);
    RCLCPP_INFO(get_logger(),
      "  [曲率ゲート] %s r_est帯=[%.2f,%.2f]m (%.0fm 以遠は下限 %.2f) | "
      "[既知半径残差ゲート] %s 上限=%.3fm",
      curv_gate_enable_param_ ? "有効" : "無効", r_est_min_param_, r_est_max_param_,
      r_est_far_range_m_param_, r_est_min_far_param_,
      knownr_rms_gate_enable_param_ ? "有効" : "無効", knownr_rms_max_param_);
    RCLCPP_INFO(get_logger(),
      "  [地面下フィルタ] %s z_min=%.2fm(固定fallback) 平面基準=%s margin=%.2fm | "
      "[上端基準復号] %s tol=%.2fm",
      ground_filter_enable_param_ ? "有効" : "無効",
      ground_filter_z_min_param_,
      ground_filter_use_plane_param_ ? "有効" : "無効",
      ground_filter_margin_param_,
      decode_top_referenced_param_ ? "有効" : "無効",
      decode_window_tol_param_);
    RCLCPP_INFO(get_logger(),
      "  [点群水平化] %s 方位bin=%.1fdeg 距離bin=%.1fm 範囲[%.1f,%.1f]m 最小セル=%d "
      "反復=%d inlier=%.2fm trim=%.2fm LPFα=%.2f 絶対上限(傾き=%.1fdeg 残差=%.3fm) "
      "相対ゲート(残差×%.2f セル×%.2f 窓=%d 再取得=%d) 平面必須=%s",
      ground_level_enable_param_ ? "有効" : "無効",
      ground_level_az_bin_deg_param_, ground_level_range_bin_m_param_,
      ground_level_range_min_m_param_, ground_level_range_max_m_param_,
      ground_level_min_cells_param_, ground_level_iters_param_,
      ground_level_inlier_m_param_, ground_level_trim_m_param_,
      ground_level_lpf_alpha_param_,
      ground_level_max_tilt_deg_param_, ground_level_max_resid_m_param_,
      ground_level_resid_rel_k_param_, ground_level_cells_rel_k_param_,
      ground_level_ref_window_param_, ground_level_reacquire_frames_param_,
      ground_level_require_plane_param_ ? "はい(未取得フレームは出力しない)" : "いいえ(固定zで出力)");
    RCLCPP_INFO(get_logger(),
      "  [形状特徴の窓限定] %s (arc_width/r_est/sagitta を復号窓内の点だけで計算)",
      arc_window_limit_enable_param_ ? "有効" : "無効");
    RCLCPP_INFO(get_logger(),
      "  [Z幅] 期待=%.3fm 許容[%.3f, %.3f]m (非対称, param: tol_low=%.2f tol_high=%.2f)",
      expected_dist,
      expected_dist - height_tol_low_param_,
      expected_dist + height_tol_high_param_,
      height_tol_low_param_, height_tol_high_param_);
    RCLCPP_INFO(get_logger(),
      "  [パターン数] %zu 種類 ハミング距離許容=%d",
      KNOWN_BIN_PATTERNS.size(), hamming_max_dist_param_);
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  float height_tol_low_param_;   // z_spanゲート下側許容(実行時パラメータ)
  float height_tol_high_param_;  // z_spanゲート上側許容(実行時パラメータ)
  int   bin_min_points_param_;   // ビン点数閾値(実行時パラメータ)

  // ============================================================
  //  追加パラメータ群 (yaml/launchから上書き可、デフォルト値は Params:: と同一)
  // ============================================================
  float anchor_height_param_;
  float bin_height_param_;
  float landmark_width_param_;
  float intensity_ratio_param_;
  float intensity_abs_min_param_;
  float group_xy_threshold_param_;
  int   min_group_points_param_;
  float cylinder_xy_radius_param_;
  float landmark_radius_param_;
  int   cfit_known_r_min_points_param_;
  float cfit_known_r_max_rms_param_;
  int   cfit_known_r_max_iter_param_;
  float bin_inner_margin_param_;
  // ---- ビン判定を強度で行う (2026-07-16) ----
  bool  bin_read_intensity_param_;    // true: 高強度割合で判定 / false: 旧点数閾値
  float bin_frac_min_param_;          // 高強度割合の下限 (これ以上で反射=1)
  int   bin_min_total_param_;         // 割合判定に必要な窓内最小点数 (未満は0)
  float bin_intensity_radius_param_;  // 割合の分母を取るXY半径 [m]
  // 距離適応 (2026-07-15): (B)クラスタリング距離 (C)ビン'1'最小点数
  bool  group_xy_adaptive_param_;
  float group_xy_k_param_, group_xy_dtheta_h_param_, group_xy_floor_param_;
  int   hamming_max_dist_param_;

  // ---- BIN 生成方式 & 地面下フィルタ ----
  std::string bin_method_param_;            // "top_anchor_only" | "both_anchors"
  float top_anchor_detect_window_param_;    // top_anchor_only の z_ta 平均窓高さ [m]
  bool  ground_filter_enable_param_;
  float ground_filter_z_min_param_;
  // ---- 点群水平化 (案C) ----
  bool  ground_level_enable_param_;
  float ground_level_az_bin_deg_param_;
  float ground_level_range_bin_m_param_;
  float ground_level_range_min_m_param_;
  float ground_level_range_max_m_param_;
  int   ground_level_min_cells_param_;
  int   ground_level_iters_param_;
  float ground_level_inlier_m_param_;
  float ground_level_trim_m_param_;
  float ground_level_max_tilt_deg_param_;
  float ground_level_max_resid_m_param_;
  float ground_level_lpf_alpha_param_;
  bool  ground_level_require_plane_param_;
  float ground_level_resid_rel_k_param_;
  float ground_level_cells_rel_k_param_;
  int   ground_level_ref_window_param_;
  int   ground_level_reacquire_frames_param_;
  bool  ground_filter_use_plane_param_;
  float ground_filter_margin_param_;
  GroundPlane last_plane_{};            // 直近の有効な平面 (フィット失敗フレームで保持)
  // 相対ゲート用の参照統計 (直近 N 個の採用フレーム)。絶対閾値はサイト間で転移しないので、
  // 「そのサイトで普段どのくらい地面が見えているか」を基準にする (2026-08-10)。
  // 群点群 CSV ダンプ (解析用)
  std::string   group_points_csv_path_param_;
  std::ofstream group_csv_;
  long          frame_idx_ = 0;         // 受信フレーム通し番号 (CSV の突き合わせ用)
  float         cur_R_[9] = {1,0,0, 0,1,0, 0,0,1};  // 今のフレームの水平化回転
  bool          cur_leveled_ = false;
  float         cur_ground_z_ = 0.0f;
  float         cur_tilt_deg_ = 0.0f;
  double        cur_stamp_ = 0.0;

  std::deque<float> plane_ref_resid_;   // 採用フレームの inlier 残差
  std::deque<int>   plane_ref_cells_;   // 採用フレームの inlier セル数
  int plane_rel_reject_run_ = 0;        // 相対ゲートでの連続却下数 (再取得判定用)
  // ---- 形状特徴の窓限定 (案B) ----
  bool  arc_window_limit_enable_param_;

  // ---- 上端基準・地面フリー復号 (2026-07-17, 既定=これで運用) ----
  // マーカーは常に最上反射。z_high=マーカー上端から既知反射高 expected_dist だけ下ろした
  // 窓で本体を切り、窓より下(床反射虚像)は無視する。床/絶対高さ/鏡像を一切使わない。
  bool  decode_top_referenced_param_;
  float decode_window_tol_param_;       // 窓下端の余白 [m] (z_high - expected - tol より下を虚像として除外)
  // ---- 曲率ゲート (2026-07-17: r_est=物理半径は距離不変) ----
  bool  curv_gate_enable_param_;        // true で r_est 帯ゲート適用
  float r_est_min_param_, r_est_max_param_;  // 円柱半径帯 [m]。帯外=平面/非円柱で棄却
  float r_est_far_range_m_param_;            // これ以遠は下限を r_est_min_far に緩める
  float r_est_min_far_param_;                // 遠方での r_est 下限
  bool  knownr_rms_gate_enable_param_;       // 既知半径フィット残差ゲート
  float knownr_rms_max_param_;               // その上限 [m]
  // ---- アブレーション用トグル (2026-07-16) ----
  bool  same_id_collision_discard_param_;  // 同一ID複数箇所→全破棄
  bool  anchor_guard_enable_param_;        // 上端(両端)アンカー欠落で棄却
  bool  dedup_enable_param_;               // 近接(<0.5m)同一円柱の重複除去

  float last_intensity_thr_ = 0.0f;         // 直近フレームの強度フィルタ閾値 (ビン強度割合の分母判定用)
  bool  perf_log_enable_param_;             // 区間処理時間ログ (性能調査用)

  // ---- 時間ヒステリシス ----
  bool  hysteresis_enable_param_;
  int   hysteresis_window_param_;
  int   hysteresis_min_agree_param_;
  float hysteresis_track_radius_param_;
  int   hysteresis_expire_frames_param_;
  int   hysteresis_warmup_frames_param_;

  struct ClusterTrack {
    int                      id;
    Centroid                 last_xy;        // 直近フレームのXY (livox_frame)
    std::deque<std::string>  id_history;     // 直近Nフレームの matched_id ("" 含む)
    int                      missed_frames = 0;
    int                      total_seen = 0;
    bool                     matched_this_frame = false;
  };
  std::vector<ClusterTrack> tracks_;
  int next_track_id_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    pub_poses_;
  rclcpp::Publisher<vehicle_interface::msg::PoseStampedArray>::SharedPtr pub_matched_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_ids_;

  // 入出力トピック / フレーム (parameter)
  std::string input_topic_;
  std::string output_topic_;
  std::string input_frame_id_;   // 点群の元フレーム (livox_frame / hesai_qt128 / hesai_ot128)
  std::string output_frame_id_;  // 自己位置推定が期待するフレーム (base_link)

  // ============================================================
  //  強度フィルター (_gauss2 同等)
  // ============================================================
  std::vector<Point4> intensityFilter(const std::vector<Point4>& pts)
  {
    float mx = 0;
    for (auto& p : pts) mx = std::max(mx, p.intensity);
    float thr = std::max(mx * intensity_ratio_param_, intensity_abs_min_param_);
    last_intensity_thr_ = thr;  // ビン純度計算 (detectLandmarks) が同じ閾値を使う
    std::vector<Point4> res;
    for (auto& p : pts) if (p.intensity > thr) res.push_back(p);
    RCLCPP_DEBUG(get_logger(), "[強度フィルタ] max=%.1f 閾値=%.1f 通過=%zu点",
      mx, thr, res.size());
    return res;
  }

  // ============================================================
  //  ランドマーク検出 (2D XY グループ方式)
  //
  //  手順:
  //    ① XY平面で高強度点を BFS グループ化(Z距離を無視)
  //    ② グループの Z スパン ≈ expected_dist(1.70m) で候補絞り込み
  //    ③ アンカー候補ゾーンから 5cm 物理ギャップを使って bot/top アンカーZを推定
  //    ④ アンカーZからビン境界を算出
  //    ⑤ 円柱XY半径内かつビン[k]のZ範囲内の点数を集計
  //    ⑥ KNOWN_BIN_PATTERNS と照合(ハミング距離)
  //    ⑦ 同一円柱からの重複検出を z_span で除去
  // ============================================================
  struct LandmarkGroup {
    Centroid group_centroid;
    Centroid z_top_anchor;     // 上端アンカー中心
    Centroid z_bot_anchor;     // 下端アンカー中心
    Centroid cylinder_xy;      // 円柱XY中心(ビン判定基準)
    std::vector<int> pattern;
    std::vector<int> bin_point_counts;  // ビンごとの高強度点数(デバッグ用)
    std::string matched_id;
    float z_span = 0.0f;       // 群Z幅(重複除去・信頼性評価用)
    int bot_cnt = 0;           // 下端アンカー候補点数
    int top_cnt = 0;           // 上端アンカー候補点数
  };

  //  pts     : 強度フィルタ通過点 (グループ化・グリッド生成に使用)
  //  all_pts : 強度フィルタ前の全点 (ビン強度割合の分母に使用, 2026-07-16)
  std::vector<LandmarkGroup> detectLandmarks(const std::vector<Point4>& pts,
                                             const std::vector<Point4>& all_pts)
  {
    if (pts.empty()) return {};
    const size_t N = pts.size();

    // ステップ①: 2D XY BFS グループ化 (グリッドハッシュ版)
    //
    // 旧実装は全点対全点の O(N²)。OT128 の強度255飽和で N が数千〜1万点に
    // 膨らむと 10Hz 予算を圧迫するため、セル幅 = 結合距離のグリッドに点を
    // ハッシュし、近傍探索を自セル+8近傍に限定して O(N×局所密度) にする。
    // 結合距離内の点は必ず 3×3 セル内に収まるため、グループ分けの結果は
    // 旧実装と数学的に同一。
    const float xy_d2 = group_xy_threshold_param_ * group_xy_threshold_param_;
    const float inv_cell = 1.0f / group_xy_threshold_param_;
    // (B) 距離適応クラスタリング距離: 各点 eps(r)=clamp(k·r·Δθ_h, floor, group_xy_threshold)。
    // grid セル幅は上限(=group_xy_threshold)のままなので 3x3 近傍探索は有効。接続は min(eps_i,eps_j)。
    std::vector<float> eps2(N, xy_d2);
    if (group_xy_adaptive_param_) {
      for (size_t i = 0; i < N; i++) {
        const float r = std::sqrt(pts[i].x * pts[i].x + pts[i].y * pts[i].y);
        float e = group_xy_k_param_ * r * group_xy_dtheta_h_param_;
        e = std::min(group_xy_threshold_param_, std::max(group_xy_floor_param_, e));
        eps2[i] = e * e;
      }
    }
    auto cell_key = [](int32_t ix, int32_t iy) -> uint64_t {
      return ((uint64_t)(uint32_t)ix << 32) | (uint64_t)(uint32_t)iy;
    };
    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(N * 2);
    for (size_t i = 0; i < N; i++) {
      const int32_t ix = (int32_t)std::floor(pts[i].x * inv_cell);
      const int32_t iy = (int32_t)std::floor(pts[i].y * inv_cell);
      grid[cell_key(ix, iy)].push_back((int)i);
    }

    std::vector<int> gid(N, -1);
    int ng = 0;
    for (size_t i = 0; i < N; i++) {
      if (gid[i] != -1) continue;
      gid[i] = ng;
      std::queue<int> q; q.push((int)i);
      while (!q.empty()) {
        const int cur = q.front(); q.pop();
        const int32_t cix = (int32_t)std::floor(pts[cur].x * inv_cell);
        const int32_t ciy = (int32_t)std::floor(pts[cur].y * inv_cell);
        for (int32_t gx = cix - 1; gx <= cix + 1; gx++) {
          for (int32_t gy = ciy - 1; gy <= ciy + 1; gy++) {
            const auto it = grid.find(cell_key(gx, gy));
            if (it == grid.end()) continue;
            for (int j : it->second) {
              if (gid[j] != -1) continue;
              const float dx = pts[cur].x - pts[j].x, dy = pts[cur].y - pts[j].y;
              const float e2 = std::min(eps2[cur], eps2[j]);
              if (dx * dx + dy * dy <= e2) { gid[j] = ng; q.push(j); }
            }
          }
        }
      }
      ng++;
    }

    std::vector<std::vector<int>> xy_groups(ng);
    for (size_t i = 0; i < N; i++) xy_groups[gid[i]].push_back((int)i);

    // 期待Z幅: ランドマーク全高 1.80m から 2×BIN_INNER_MARGIN(LiDAR が端を取りこぼす経験補正)を差し引いた値
    const float expected_dist =
      2.0f * anchor_height_param_ + Params::BIN_COUNT * bin_height_param_
      - 2.0f * bin_inner_margin_param_;

    std::vector<LandmarkGroup> result;
    const float r2 = cylinder_xy_radius_param_ * cylinder_xy_radius_param_;

    int group_idx_dbg = -1;
    for (const auto& grp : xy_groups) {
      group_idx_dbg++;
      if ((int)grp.size() < min_group_points_param_) continue;

      // ステップ②: Z 分布を解析
      std::vector<float> zv;
      float sx = 0.0f, sy = 0.0f;
      zv.reserve(grp.size());
      for (int idx : grp) {
        zv.push_back(pts[idx].z);
        sx += pts[idx].x; sy += pts[idx].y;
      }
      std::sort(zv.begin(), zv.end());
      const int n = (int)zv.size();

      // 外れ値耐性: 点数が8以上の場合のみ上下各1点を除外
      // 4〜7点の疎な点群では除外するとZ幅が大きく圧縮されるため無効化
      const int k = (n >= 8) ? 1 : 0;
      const float z_high = zv[n - 1 - k];
      // 復号窓の下端。z_low だけでなく形状特徴(computeArcFeatures)にも渡すのでブロック外で持つ。
      const float window_lo = z_high - expected_dist - decode_window_tol_param_;
      float z_low;
      if (decode_top_referenced_param_) {
        // 上端基準・地面フリー: z_high(=マーカー上端)から既知反射高 expected_dist だけ下ろした
        // 窓 [z_high - expected_dist - tol, z_high] の最下反射点を z_low とする。窓より下の
        // 床反射虚像は z_span から除外され、虚像による z_span 肥大での棄却を防ぐ (2026-07-17)。
        z_low = z_high;
        for (int i = k; i <= n - 1 - k; i++) { if (zv[i] >= window_lo) { z_low = zv[i]; break; } }
      } else {
        z_low = zv[k];
      }
      const float z_span = z_high - z_low;

      // 注: top_anchor_only でもこのゲートは z_low 依存の両側条件。
      // 床反射等で z_low が 0.15m 超引き下げられるとグリッド自体は正しく
      // 引けても棄却される (検出率低下)。運用で床汚染が支配的なら
      // ground_filter_enable か height_tol_high (yaml) で調整する。
      // 誤識別ゼロ優先のためデフォルトは両側 ±0.15m のまま。
      // cx,cy,dist_xy を先に計算 (z幅ゲートの距離適応で使うため上に移動, 2026-07-16)
      const float cx = sx / n, cy = sy / n;
      const float dist_xy = std::sqrt(cx * cx + cy * cy);
      if (z_span < expected_dist - height_tol_low_param_ ||
          z_span > expected_dist + height_tol_high_param_) {
        RCLCPP_DEBUG(get_logger(),
          "  [XY群スキップ] (%.2f,%.2f) dist=%.1fm 点数=%d Z幅=%.3fm z=[%.2f,%.2f] "
          "期待=%.3fm 許容[%.3f,%.3f]",
          cx, cy, dist_xy, n, z_span, z_low, z_high,
          expected_dist,
          expected_dist - height_tol_low_param_,
          expected_dist + height_tol_high_param_);
        continue;
      }

      // ============================================================
      //  ステップ③: スケール適応型アンカー・ビン境界推定
      //
      //  【設計方針】 旧実装はギャップ検出で bot_anchor_top を求めていたが、
      //  LiDAR の垂直分解能が物理 5cm ギャップより粗いときアンカー内部の
      //  サンプル間隙を境界と誤認し bot_anchor_z が大きく振れる弱点があった
      //  (記憶: bot_anchor_z 変動 0.36m)。
      //
      //  新実装は「実測 z_span を物理 1.80m の縮尺と見立て、全構造を比例縮小」
      //  する。アンカー候補ゾーンの点数や内部間隙に一切依存しないため、
      //  LiDAR 種別や角度を超えて再現可能な決定論的計算になる。
      //
      //    scale = z_span / (2*ANCHOR_HEIGHT + BIN_COUNT*BIN_HEIGHT)
      //    scaled_anchor = ANCHOR_HEIGHT * scale
      //    scaled_bin    = BIN_HEIGHT    * scale
      //    scaled_margin = BIN_INNER_MARGIN * scale
      //
      //    bot_anchor_z = z_low  + scaled_anchor / 2
      //    top_anchor_z = z_high - scaled_anchor / 2
      //    bin_base     = z_low  + scaled_anchor   (= bin[0] 下端)
      //    bl[k] = bin_base + k     * scaled_bin + scaled_margin
      //    bh[k] = bin_base + (k+1) * scaled_bin - scaled_margin
      //
      //  【誤識別耐性】 シフト誤識別 (BIN_3↔4↔2, BIN_5↔6) が起きるためには
      //  z_low か z_high が「真の物理端」から大きくずれている必要がある。
      //  例: 下端アンカーが見えず z_low が bin[0] 反射底面に乗ったとすると
      //  z_span は約 1.45m に短縮され、HEIGHT_TOLERANCE_LOW 範囲外で
      //  そもそも候補絞り込みで弾かれる(後述の z_span 範囲絞り込み参照)。
      // ============================================================
      // scale はログ出力用に保持 (BIN 計算では未使用)
      const float scale =
        z_span / (2.0f * anchor_height_param_ +
                  Params::BIN_COUNT * bin_height_param_);

      // ── ビングリッド生成 (bin_method で切替) ──
      //  top_anchor_only (デフォルト):
      //    上端アンカのみ参照、物理寸法固定で BIN を切り下げる。
      //    OT128 の床反射虚像 (下側汚染) に強い (2026-07-02)。
      //  both_anchors:
      //    上下両アンカを参照、scale = z_span / expected でグリッドを伸縮。
      //    z_low 汚染がない Avia 系や、遠距離縦圧縮を吸収したい場合に有効。
      std::vector<float> bl(Params::BIN_COUNT), bh(Params::BIN_COUNT);
      float top_anchor_z = 0.0f, bot_anchor_z = 0.0f;
      int   top_cnt = 0, bot_cnt = 0;

      if (bin_method_param_ == "both_anchors") {
        // ---- 上下両端アンカ方式 (scale 適応) ----
        // expected_dist の前提 (LiDAR は両端 margin 分を取りこぼす) に合わせ、
        // z_low ↔ 物理 margin / z_high ↔ 物理 (全高 − margin) を対応付ける。
        // 縮尺 sc = z_span / expected_dist (旧: 全高 1.80 割りで系統 ~3.5cm ずれ)。
        const float sc = z_span / expected_dist;
        // アンカー可視帯 = 物理アンカーから margin 分を除いた高さ
        const float scaled_anchor = (anchor_height_param_ - bin_inner_margin_param_) * sc;

        const float ba_hi = z_low  + scaled_anchor;    // 下端アンカ帯上限
        const float ta_lo = z_high - scaled_anchor;    // 上端アンカ帯下限
        float ba_sum = 0.0f, ta_sum = 0.0f;
        for (int idx : grp) {
          const float z = pts[idx].z;
          // トリム済み外れ値 (z_low 未満 / z_high 超) をアンカー平均から除外
          if (z < z_low || z > z_high) continue;
          if (z <= ba_hi) { ba_sum += z; bot_cnt++; }
          if (z >= ta_lo) { ta_sum += z; top_cnt++; }
        }
        bot_anchor_z = (bot_cnt > 0) ? ba_sum / bot_cnt : z_low  + scaled_anchor * 0.5f;
        top_anchor_z = (top_cnt > 0) ? ta_sum / top_cnt : z_high - scaled_anchor * 0.5f;

        // 物理高さ h (ランドマーク底面基準) → z 変換: z = z_low + (h − margin)·sc
        for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
          const float tape_lo = anchor_height_param_ +  kk      * bin_height_param_
                              + bin_inner_margin_param_;   // 反射材下端 [物理]
          const float tape_hi = anchor_height_param_ + (kk + 1) * bin_height_param_
                              - bin_inner_margin_param_;   // 反射材上端 [物理]
          bl[kk] = z_low + (tape_lo - bin_inner_margin_param_) * sc;
          bh[kk] = z_low + (tape_hi - bin_inner_margin_param_) * sc;
        }
      } else {
        // ---- 上端アンカのみ方式 (デフォルト、床反射虚像対策) ----
        // 検出窓 top_anchor_detect_window_param_ (既定=anchor_height) を z_high から
        // 下ろして上端アンカー中心 z_ta を推定し、そこから物理寸法でグリッドを吊る。
        // 20260703実測で反射柱は遠方27mでもほぼ非圧縮のため縦スケールは 1.0 固定。
        const float detect_window = top_anchor_detect_window_param_;
        // グリッドは幾何 z_high (=最上反射=マーカー上端) から吊る。
        const float ta_lo = z_high - detect_window;
        float ta_sum = 0.0f;
        for (int idx : grp) {
          const float z = pts[idx].z;
          // トリム済み上方外れ値 (z_high 超) を除外。含めると z_ta が
          // 吊り上がり全ビンが上方シフトする (誤識別源)
          if (z < ta_lo || z > z_high) continue;
          ta_sum += z; top_cnt++;
        }
        const float z_ta = (top_cnt > 0) ? ta_sum / top_cnt
                                         : z_high - detect_window * 0.5f;
        top_anchor_z = z_ta;
        bot_anchor_z = z_ta - (anchor_height_param_ + Params::BIN_COUNT * bin_height_param_);
        bot_cnt = 0;  // 下端は参照しないため 0 固定

        const float half_w = bin_height_param_ * 0.5f - bin_inner_margin_param_;
        for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
          const float offset_from_ta = anchor_height_param_ * 0.5f
            + (Params::BIN_COUNT - 1 - kk + 0.5f) * bin_height_param_;
          const float center = z_ta - offset_from_ta;
          bl[kk] = center - half_w;
          bh[kk] = center + half_w;
        }
      }

      // ステップ⑤: ビン別集計
      //  bin_counts       : 高強度点数 (旧点数判定・ログ用)
      //  bin_total/bin_high : bin_intensity_radius 内の全強度点数と高強度点数。
      //    高強度割合 = bin_high/bin_total。反射材=強度なので、これは点数と違い
      //    サンプリング密度(仰角依存)に不変な観測量になる (2026-07-16)。
      std::vector<int> bin_counts(Params::BIN_COUNT, 0);
      for (const auto& p : pts) {
        const float dx = p.x - cx, dy = p.y - cy;
        if (dx * dx + dy * dy > r2) continue;
        for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
          if (p.z >= bl[kk] && p.z < bh[kk]) { bin_counts[kk]++; break; }
        }
      }
      std::vector<int> bin_total(Params::BIN_COUNT, 0);
      std::vector<int> bin_high(Params::BIN_COUNT, 0);
      const float rf2 = bin_intensity_radius_param_ * bin_intensity_radius_param_;
      const float ithr = last_intensity_thr_;
      for (const auto& p : all_pts) {
        const float dx = p.x - cx, dy = p.y - cy;
        if (dx * dx + dy * dy > rf2) continue;
        for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
          if (p.z >= bl[kk] && p.z < bh[kk]) {
            bin_total[kk]++;
            if (p.intensity > ithr) bin_high[kk]++;
            break;
          }
        }
      }
      std::vector<float> bin_frac(Params::BIN_COUNT, 0.0f);
      for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
        bin_frac[kk] = (bin_total[kk] > 0)
          ? (float)bin_high[kk] / (float)bin_total[kk] : 0.0f;
      }

      // ステップ⑥: パターン生成 & マッチング
      std::vector<int> pattern(Params::BIN_COUNT);
      for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
        if (bin_read_intensity_param_) {
          // 強度割合判定: 窓内に十分な点があり、その高強度割合が閾値以上なら反射(=1)。
          // 点が少なくても高強度割合が高ければ反射と読める(遠方・FOV端の疎を救済)。
          pattern[kk] = (bin_total[kk] >= bin_min_total_param_ &&
                         bin_frac[kk] >= bin_frac_min_param_) ? 1 : 0;
        } else {
          pattern[kk] = (bin_counts[kk] >= bin_min_points_param_) ? 1 : 0;
        }
      }

      std::string matched_id = matchPattern(pattern);
      const std::string matched_id_before_guard = matched_id;  // ログ用に保持

      // ============================================================
      //  ステップ⑥-b: 誤識別ガード
      //  【目的】 誤識別ゼロを最優先する。検出率は犠牲にしても構わない。
      //
      //  実LiDAR の垂直分解能では「反射材1枚 = 1点」しか取れないことが
      //  あり (特にアンカーの片端や BIN_2={1,1,0,0} 上半分の薄いゾーン)、
      //  ANCHOR_MIN_POINTS を 2 にすると正常検出も棄却される。
      //
      //  ガード方針:
      //   - 両端アンカーゾーンの **両方とも 0 点** の場合のみ棄却
      //     (LiDAR がランドマーク中央しか捉えていない異常状態を排除)
      //   - 片端だけ 0 点 = 反射材1枚ぶんの単点でも採用
      //   - z_span そのものは HEIGHT_TOLERANCE_LOW/HIGH (±0.15m) で既に
      //     厳格にガードされているので、シフト誤識別リスクは構造的に低い
      // ============================================================
      // 形状計測 (r_est=曲率ゲート用。arc_width/sagitta は BINCSV 分析用)
      // 復号窓と同じ点集合で計算する (案B, 2026-08-10)。窓を使わない設定 or
      // アブレーション時は -inf を渡してグループ全点の従来動作に戻す。
      const float arc_window_lo =
        (arc_window_limit_enable_param_ && decode_top_referenced_param_)
          ? window_lo : -std::numeric_limits<float>::infinity();
      const ArcFeat arc_feat = computeArcFeatures(grp, pts, arc_window_lo,
                                                  landmark_radius_param_);
      const char* reject_reason = nullptr;
      if (!matched_id.empty()) {
        if (anchor_guard_enable_param_) {
          if (bin_method_param_ == "both_anchors") {
            // 上下両アンカ方式: 両端とも 0 点のときのみ棄却
            if (top_cnt == 0 && bot_cnt == 0) reject_reason = "両端アンカー欠落";
          } else {
            // 上端アンカのみ参照方式: top_cnt==0 → 棄却
            if (top_cnt == 0) reject_reason = "上端アンカー欠落";
          }
        }
        // ---- 曲率ゲート (2026-07-17: 平面反射板 vs 円柱前面弧) ----
        // r_est=物理シリンダ半径で距離不変なので固定帯 [r_est_min, r_est_max] 1組で足りる。
        // 平面反射板 → 半径∞側(r_est 大) / 微小ノイズ弧 → r_est 小、どちらも帯外で棄却。
        const float r_min_here = (dist_xy >= r_est_far_range_m_param_)
                                   ? r_est_min_far_param_ : r_est_min_param_;
        if (!reject_reason && curv_gate_enable_param_ &&
            (arc_feat.r_est < r_min_here || arc_feat.r_est > r_est_max_param_)) {
          reject_reason = "曲率帯外(平面/非円柱)";
        }
        // ---- 既知半径フィット残差ゲート (2026-08-10) ----
        // 下限 0.14 の役目 (微小ノイズ弧の排除) をこちらに移すためのゲート。
        // 半径を固定するので遠方の方位サンプリング不足に強い。
        if (!reject_reason && knownr_rms_gate_enable_param_ &&
            arc_feat.knownr_rms >= 0.0f && arc_feat.knownr_rms > knownr_rms_max_param_) {
          reject_reason = "既知半径フィット残差過大";
        }
        if (reject_reason) {
          matched_id.clear();
        }
      }

      // デバッグログ
      std::string ps, cs;
      for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
        ps += std::to_string(pattern[kk]);
        cs += std::to_string(bin_counts[kk]);
        if (kk + 1 < Params::BIN_COUNT) cs += ",";
      }
      RCLCPP_DEBUG(get_logger(),
        "[XY群] (%.2f,%.2f) dist=%.1fm 点%d Z幅=%.3f scale=%.3f "
        "アンカーZ[下=%.3f(%d点) 上=%.3f(%d点)] "
        "bl=%.3f bh=%.3f パターン=%s 点数=[%s] → %s%s%s%s",
        cx, cy, dist_xy, n, z_span, scale,
        bot_anchor_z, bot_cnt,
        top_anchor_z, top_cnt,
        bl[0], bh[Params::BIN_COUNT-1],
        ps.c_str(), cs.c_str(),
        matched_id_before_guard.empty() ? "不一致" : matched_id_before_guard.c_str(),
        reject_reason ? " [棄却:" : "",
        reject_reason ? reject_reason : "",
        reject_reason ? "]" : "");

      // パース可能なCSVデバッグ行 (原因解析用、log-level=debug で出力)
      //  列: tag,cx,cy,dist,n,z_low,z_high,z_span,scale,bin_height,z_ba(phys),z_ta,
      //      bl0,bh0,bl1,bh1,bl2,bh2,bl3,bh3,c0,c1,c2,c3,bot_cnt,top_cnt,
      //      pattern,id_before,id_after,
      //      arc_width,r_est,arc_npts (2026-07-08 形状計測列、棄却なし)
      //      ,sagitta (2026-07-14 半径方向奥行き=平面vs円柱弁別。棄却なし)
      //  上端アンカのみ方式: z_ba は物理推定値 (bot_anchor_z)、s は bin_height 固定
      //  2026-07-16: ビン純度列(p0-3,zbin_pur)はビン純度ゲート削除に伴い廃止。
      //  2026-07-16: 末尾に bt0-3(bin_total 全強度点数), f0-3(高強度割合) を追加
      //             (点数→強度の観測量変更のA/B・閾値決め用)。
      //  形状計測を BINCSV に出力 (arc_feat は上の高さ/サジッタゲートで計算済を再利用)。
      RCLCPP_DEBUG(get_logger(),
        "BINCSV,%.4f,%.4f,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%d,%d,%d,%d,%d,%d,%s,%s,%s,%.4f,%.4f,%d,%.4f,"
        "%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.4f,%d",
        cx, cy, dist_xy, n, z_low, z_high, z_span, scale,
        bin_height_param_, bot_anchor_z, top_anchor_z,
        bl[0], bh[0], bl[1], bh[1], bl[2], bh[2], bl[3], bh[3],
        bin_counts[0], bin_counts[1], bin_counts[2], bin_counts[3],
        bot_cnt, top_cnt,
        ps.c_str(),
        matched_id_before_guard.empty() ? "NONE" : matched_id_before_guard.c_str(),
        matched_id.empty() ? "NONE" : matched_id.c_str(),
        arc_feat.arc_width, arc_feat.r_est, arc_feat.arc_npts,
        arc_feat.sagitta,
        bin_total[0], bin_total[1], bin_total[2], bin_total[3],
        bin_frac[0], bin_frac[1], bin_frac[2], bin_frac[3],
        arc_feat.knownr_rms, arc_feat.knownr_n);

      // ---- 群点群 CSV: この群でビン判定に使った点をそのまま書き出す ----
      // 座標は検出器内部の系(水平化後)。x_raw/y_raw/z_raw は回転を戻したセンサ系。
      if (group_csv_.is_open()) {
        const std::string ps_id_b =
          matched_id_before_guard.empty() ? "NONE" : matched_id_before_guard;
        const std::string ps_id_a = matched_id.empty() ? "NONE" : matched_id;
        const char* rr = reject_reason ? reject_reason : "";
        for (int idx : grp) {
          const Point4& p = pts[idx];
          // 水平化前 = R^T · p (R は回転行列なので転置が逆行列)
          const float xr = cur_R_[0]*p.x + cur_R_[3]*p.y + cur_R_[6]*p.z;
          const float yr = cur_R_[1]*p.x + cur_R_[4]*p.y + cur_R_[7]*p.z;
          const float zr = cur_R_[2]*p.x + cur_R_[5]*p.y + cur_R_[8]*p.z;
          group_csv_ << frame_idx_ << ',' << cur_stamp_ << ',' << group_idx_dbg << ','
                     << cx << ',' << cy << ',' << dist_xy << ',' << n << ','
                     << z_low << ',' << z_high << ',' << z_span << ','
                     << ps << ',' << ps_id_b << ',' << ps_id_a << ',' << rr << ','
                     << (cur_leveled_ ? 1 : 0) << ',' << cur_ground_z_ << ','
                     << cur_tilt_deg_ << ','
                     << p.x << ',' << p.y << ',' << p.z << ',' << p.intensity << ','
                     << xr << ',' << yr << ',' << zr << ','
                     << std::sqrt((p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy)) << '\n';
        }
      }

      // publish 用 XY 中心 = 既知半径円フィット (known_r)。ビン判定は cx,cy のまま不変で、
      // 出力位置のみ精緻化する。初期値/フォールバックは los_offset (重心をR押し出した種)。
      // los_offset 自体は出力方式として露出せず、known_r の内部種としてのみ使う。
      // 点数不足/RMS超過/未収束なら los_offset にフォールバック。既知半径は
      // landmark_radius_param_ (物理外径半径)。
      float cx_los, cy_los;
      losOffset(cx, cy, landmark_radius_param_, cx_los, cy_los);
      float out_cx = cx_los, out_cy = cy_los;
      if ((int)grp.size() >= cfit_known_r_min_points_param_) {
        float kx, ky, krms;
        const bool kr_ok = fitCircleKnownR(grp, pts, cx_los, cy_los,
                                           landmark_radius_param_,
                                           cfit_known_r_max_iter_param_,
                                           kx, ky, krms);
        if (kr_ok && krms <= cfit_known_r_max_rms_param_) { out_cx = kx; out_cy = ky; }
      }

      LandmarkGroup g;
      g.pattern         = pattern;
      g.bin_point_counts = bin_counts;
      g.matched_id      = matched_id;
      g.group_centroid.x = out_cx;
      g.group_centroid.y = out_cy;
      // z はトリム済み端点 (z_low/z_high) で一貫させる (グリッド計算と同基準)
      g.group_centroid.z = (z_low + z_high) * 0.5f;
      g.cylinder_xy     = g.group_centroid;
      g.z_bot_anchor    = g.group_centroid; g.z_bot_anchor.z = bot_anchor_z;
      g.z_top_anchor    = g.group_centroid; g.z_top_anchor.z = top_anchor_z;
      g.z_span          = z_span;
      g.bot_cnt         = bot_cnt;
      g.top_cnt         = top_cnt;
      result.push_back(g);
    }

    // ============================================================
    // ステップ⑦: 同一円柱からの重複検出を除去
    //
    // 30°/遠距離視で、同じ物理円柱から複数のXYグループが生成され
    // 異なるIDにマッチすることがある。両方がpublishされると
    // 自己位置推定が混乱する。
    //
    // 処理: 近接XY (< DEDUP_XY_DIST) の検出結果を「同一円柱」とみなし、
    //       Z幅が大きい方 (= より完全な円柱視野) を採用する。
    //
    // 採用基準:
    //   z_span大 ⇔ 上下アンカー両方の点が含まれる完全な視野
    //          ⇔ アンカー推定が信頼できる
    //          ⇔ ビン境界が正しい
    // ============================================================
    constexpr float DEDUP_XY_DIST_SQ = 0.50f * 0.50f;
    std::vector<bool> active(result.size(), true);
    for (size_t i = 0; dedup_enable_param_ && i < result.size(); i++) {
      if (!active[i] || result[i].matched_id.empty()) continue;
      for (size_t j = i + 1; j < result.size(); j++) {
        if (!active[j] || result[j].matched_id.empty()) continue;
        const float dx = result[i].group_centroid.x - result[j].group_centroid.x;
        const float dy = result[i].group_centroid.y - result[j].group_centroid.y;
        if (dx * dx + dy * dy >= DEDUP_XY_DIST_SQ) continue;
        // 同一円柱と判定: z_spanが大きい方を採用
        const bool keep_i = result[i].z_span >= result[j].z_span;
        const auto& winner = keep_i ? result[i] : result[j];
        const auto& loser  = keep_i ? result[j] : result[i];
        RCLCPP_DEBUG(get_logger(),
          "  [重複除去] %s(z幅=%.3f)を採用 / %s(z幅=%.3f)を破棄",
          winner.matched_id.c_str(), winner.z_span,
          loser.matched_id.c_str(),  loser.z_span);
        if (keep_i) active[j] = false;
        else { active[i] = false; break; }
      }
    }
    std::vector<LandmarkGroup> deduped;
    deduped.reserve(result.size());
    for (size_t i = 0; i < result.size(); i++) {
      if (active[i]) deduped.push_back(std::move(result[i]));
    }

    return deduped;
  }

  // ============================================================
  //  パターンマッチング (ハミング距離)
  //
  //  注: BIN_1/BIN_8 削除後の6語辞書 (偶パリティ) では、1ビット誤り
  //  (奇パリティ) の入力は必ず距離1の辞書語を複数持つ → ambiguous 棄却。
  //  そのため hamming_max_dist=1 でも実質「完全一致のみ」が通る。
  //  (辞書を増減した場合はこの性質が崩れるので再確認すること)
  // ============================================================
  std::string matchPattern(const std::vector<int>& pattern)
  {
    int best_dist = 999;
    std::string best_id = "";
    bool ambiguous = false;

    for (const auto& [id, known] : KNOWN_BIN_PATTERNS) {
      if (known.size() != pattern.size()) continue;
      int d = 0;
      for (size_t k = 0; k < pattern.size(); k++) {
        if (pattern[k] != known[k]) d++;
      }
      if (d < best_dist) {
        best_dist = d;
        best_id = id;
        ambiguous = false;
      } else if (d == best_dist) {
        ambiguous = true;
      }
    }

    if (best_dist <= hamming_max_dist_param_ && !ambiguous) {
      return best_id;
    }
    return "";
  }

  // ============================================================
  //  時間ヒステリシス (1フレーム誤識別ブロック)
  //
  //  検出フレーム間で物理クラスタを XY 近接で追跡し、各トラックの
  //  ID 履歴 (直近 hysteresis_window_param_ フレーム) を保持する。
  //
  //  現フレームの matched_id が「履歴の多数決ID」と異なる場合、
  //  matched_id を "" にクリアして publish 側で破棄させる。
  //
  //  追跡は livox_frame での XY 近接 (半径 hysteresis_track_radius_param_)。
  //  期限切れ (hysteresis_expire_frames_param_ 連続未検出) トラックは削除。
  //  ウォームアップ (hysteresis_warmup_frames_param_ 未満) は publish 保留。
  // ============================================================
  void applyHysteresis(std::vector<LandmarkGroup>& landmarks)
  {
    // Step 1: 既存トラックを「今フレーム未対応」に初期化
    for (auto& t : tracks_) t.matched_this_frame = false;

    // Step 2: 各検出 → 最近傍トラックへ greedy 関連付け
    //         (XY 距離が hysteresis_track_radius 以下のペアを距離昇順で確定)
    const float r2 = hysteresis_track_radius_param_ * hysteresis_track_radius_param_;
    std::vector<int> det_to_track(landmarks.size(), -1);

    std::vector<std::tuple<float, int, int>> pairs;  // (d2, det_idx, track_idx)
    for (size_t di = 0; di < landmarks.size(); di++) {
      const auto& xy = landmarks[di].group_centroid;
      for (size_t ti = 0; ti < tracks_.size(); ti++) {
        const auto& txy = tracks_[ti].last_xy;
        float dx = xy.x - txy.x, dy = xy.y - txy.y;
        float d2 = dx * dx + dy * dy;
        if (d2 <= r2) pairs.emplace_back(d2, (int)di, (int)ti);
      }
    }
    std::sort(pairs.begin(), pairs.end());
    std::vector<bool> det_used(landmarks.size(), false);
    std::vector<bool> track_used(tracks_.size(), false);
    for (const auto& [d2, di, ti] : pairs) {
      if (det_used[di] || track_used[ti]) continue;
      det_used[di] = true;
      track_used[ti] = true;
      det_to_track[di] = ti;
    }

    // Step 3: 未対応検出 → 新規トラック誕生
    for (size_t di = 0; di < landmarks.size(); di++) {
      if (det_to_track[di] != -1) continue;
      ClusterTrack t;
      t.id = next_track_id_++;
      t.last_xy = landmarks[di].group_centroid;
      tracks_.push_back(t);
      det_to_track[di] = (int)tracks_.size() - 1;
    }

    // Step 4: 各トラックの履歴更新
    for (size_t di = 0; di < landmarks.size(); di++) {
      auto& tr = tracks_[det_to_track[di]];
      tr.last_xy = landmarks[di].group_centroid;
      tr.id_history.push_back(landmarks[di].matched_id);
      while ((int)tr.id_history.size() > hysteresis_window_param_) {
        tr.id_history.pop_front();
      }
      tr.missed_frames = 0;
      tr.total_seen++;
      tr.matched_this_frame = true;
    }
    for (auto& tr : tracks_) {
      if (tr.matched_this_frame) continue;
      tr.missed_frames++;
      tr.id_history.push_back("");  // miss も履歴に残す
      while ((int)tr.id_history.size() > hysteresis_window_param_) {
        tr.id_history.pop_front();
      }
    }

    // Step 5: 期限切れトラック削除 (det_to_track の参照ずれを避けるため
    //         判定とindex補正は landmarks ループ前に確定済み)
    //         publish判定後にまとめて削除する
    auto majority_of = [](const std::deque<std::string>& hist)
        -> std::pair<std::string, int> {
      std::map<std::string, int> cnt;
      for (const auto& s : hist) if (!s.empty()) cnt[s]++;
      std::string best;
      int best_cnt = 0;
      for (const auto& [k, v] : cnt) {
        if (v > best_cnt) { best_cnt = v; best = k; }
      }
      return {best, best_cnt};
    };

    // Step 6: publish 判定 (matched_id を "" に書き換えることで下流で破棄)
    for (size_t di = 0; di < landmarks.size(); di++) {
      auto& g = landmarks[di];
      if (g.matched_id.empty()) continue;  // そもそも未マッチは何もしない
      auto& tr = tracks_[det_to_track[di]];
      if (tr.total_seen < hysteresis_warmup_frames_param_) {
        RCLCPP_DEBUG(get_logger(),
          "[hysteresis] WARMUP track=%d total=%d id=%s xy=(%.2f,%.2f) → 保留",
          tr.id, tr.total_seen, g.matched_id.c_str(),
          g.group_centroid.x, g.group_centroid.y);
        g.matched_id.clear();
        continue;
      }
      auto [maj_id, maj_cnt] = majority_of(tr.id_history);
      if (maj_cnt < hysteresis_min_agree_param_) {
        RCLCPP_DEBUG(get_logger(),
          "[hysteresis] NO_MAJORITY track=%d id=%s maj=%s(%d) → 保留",
          tr.id, g.matched_id.c_str(), maj_id.c_str(), maj_cnt);
        g.matched_id.clear();
        continue;
      }
      if (g.matched_id != maj_id) {
        std::string hist_str;
        for (const auto& s : tr.id_history) {
          if (!hist_str.empty()) hist_str += ",";
          hist_str += s.empty() ? "-" : s;
        }
        RCLCPP_INFO(get_logger(),
          "[hysteresis] SUPPRESSED track=%d xy=(%.2f,%.2f) raw=%s majority=%s(%d/%d) hist=[%s]",
          tr.id, g.group_centroid.x, g.group_centroid.y,
          g.matched_id.c_str(), maj_id.c_str(), maj_cnt,
          hysteresis_window_param_, hist_str.c_str());
        g.matched_id.clear();
        // 抑止した誤IDを履歴に残すと2フレーム連続の誤識別で多数派が
        // 乗っ取られるため、今フレームのエントリを miss("") に置換する。
        // (Step 4 で push した末尾 = 今フレーム分)
        tr.id_history.back() = "";
      }
    }

    // Step 7: 期限切れトラック削除
    tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const ClusterTrack& t) {
          return t.missed_frames >= hysteresis_expire_frames_param_;
        }),
      tracks_.end());
  }

  geometry_msgs::msg::PoseArray makePoseArray(
    const std_msgs::msg::Header& header,
    const std::map<std::string, Centroid>& detected_positions)
  {
    geometry_msgs::msg::PoseArray pa;
    pa.header = header;
    for (const auto& [id, pos] : detected_positions) {
      geometry_msgs::msg::Pose p;
      p.position.x = pos.x; p.position.y = pos.y; p.position.z = pos.z;
      p.orientation.w = 1.0;
      pa.poses.push_back(p);
    }
    return pa;
  }

  // ============================================================
  //  /matched_marker (vehicle_interface/PoseStampedArray) 生成
  //
  //  入力フレーム(input_frame_id_ = livox_frame / hesai_qt128 / hesai_ot128) で得た
  //  検出位置を base_link (output_frame_id_) に TF 変換してから格納する。
  //  さらに BIN_n を marker_n に名前変換して localizer 規約に合わせる。
  //
  //  メッセージ規約 (NEDO_ROS_2026_Lidar/localizer.cpp と一致):
  //    msg.header.frame_id          = output_frame_id_ (base_link)
  //    msg.array[i].header.frame_id = ID 文字列 (marker_n)
  //    msg.array[i].pose.position   = base_link 系の位置 (m)
  // ============================================================
  vehicle_interface::msg::PoseStampedArray makeMatchedMarkerArray(
    const std_msgs::msg::Header& header,
    const std::map<std::string, Centroid>& detected_positions)
  {
    vehicle_interface::msg::PoseStampedArray arr;
    arr.header.stamp = header.stamp;
    arr.header.frame_id = output_frame_id_;

    // input_frame_id_ -> output_frame_id_ の TF を一度だけ取得
    geometry_msgs::msg::TransformStamped tf_stamped;
    if (input_frame_id_ != output_frame_id_) {
      try {
        // tf2::TimePointZero = 最新の有効な変換を使用 (静的TFで十分)
        tf_stamped = tf_buffer_->lookupTransform(
          output_frame_id_, input_frame_id_, tf2::TimePointZero,
          tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "TF lookup失敗 (%s -> %s): %s",
          input_frame_id_.c_str(), output_frame_id_.c_str(), ex.what());
        return arr;  // 空配列を返す (publishしてもlocalizer側で無視される)
      }
    }

    for (const auto& [bin_id, pos] : detected_positions) {
      // BIN_n → marker_n 変換
      const auto map_it = BIN_TO_MARKER_ID.find(bin_id);
      if (map_it == BIN_TO_MARKER_ID.end()) {
        RCLCPP_WARN(get_logger(), "未マップのBIN ID: %s", bin_id.c_str());
        continue;
      }
      const std::string& marker_id = map_it->second;

      // input_frame_id_ 系の検出位置を base_link 系へ変換
      geometry_msgs::msg::PoseStamped src;
      src.header.stamp = header.stamp;
      src.header.frame_id = input_frame_id_;
      src.pose.position.x = pos.x;
      src.pose.position.y = pos.y;
      src.pose.position.z = pos.z;
      src.pose.orientation.w = 1.0;

      geometry_msgs::msg::PoseStamped dst;
      if (input_frame_id_ == output_frame_id_) {
        dst = src;
      } else {
        tf2::doTransform(src, dst, tf_stamped);
      }
      // header.frame_id を ID タグとして上書き (localizer 規約)
      dst.header.frame_id = marker_id;
      arr.array.push_back(dst);
    }
    return arr;
  }

  // ============================================================
  //  メインコールバック
  // ============================================================
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    cur_stamp_ = static_cast<double>(msg->header.stamp.sec) +
                 static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    // ① パース: 地面フィルタはまだ掛けない。地面平面の推定に地面点そのものが要るため。
    std::vector<Point4> all;
    sensor_msgs::PointCloud2Iterator<float> ix(*msg, "x"), iy(*msg, "y"),
                                             iz(*msg, "z"), ii(*msg, "intensity");
    for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ii) {
      if (!std::isnan(*ix) && !std::isnan(*iy) && !std::isnan(*iz)) {
        all.push_back({*ix, *iy, *iz, *ii});
      }
    }
    if (all.empty()) return;
    const auto t1 = Clock::now();

    // ② 水平化 (案C): 地面平面を推定し、法線を +z に合わせる回転を全点に掛ける。
    //    ビン処理より前に行うので、以降の z ロジック全体が搭載高・傾きに非依存になる。
    float ground_z_level = 0.0f;   // 水平化後の地面 z (= c/|n|)
    bool  leveled = false;
    frame_idx_++;
    cur_leveled_ = false;
    cur_tilt_deg_ = 0.0f;
    cur_ground_z_ = 0.0f;
    std::fill(cur_R_, cur_R_ + 9, 0.0f);
    cur_R_[0] = cur_R_[4] = cur_R_[8] = 1.0f;
    if (ground_level_enable_param_) {
      GroundPlane gp = fitGroundPlane(
        all, ground_level_az_bin_deg_param_, ground_level_range_bin_m_param_,
        ground_level_range_min_m_param_, ground_level_range_max_m_param_,
        ground_level_min_cells_param_, ground_level_iters_param_,
        ground_level_inlier_m_param_, ground_level_trim_m_param_,
        ground_level_max_tilt_deg_param_, ground_level_max_resid_m_param_);

      // ── 相対ゲート: 直近の採用フレームを基準に「普段より明らかに悪いか」で却下する ──
      bool rel_reject = false;
      if (gp.valid && !plane_ref_resid_.empty()) {
        const float resid_ref =
          *std::min_element(plane_ref_resid_.begin(), plane_ref_resid_.end());
        const int cells_ref =
          *std::max_element(plane_ref_cells_.begin(), plane_ref_cells_.end());
        const bool resid_bad =
          gp.resid_m > ground_level_resid_rel_k_param_ * resid_ref;
        const bool cells_bad =
          gp.n_cells < ground_level_cells_rel_k_param_ * static_cast<float>(cells_ref);
        if (resid_bad || cells_bad) {
          rel_reject = true;
          gp.valid = false;
          RCLCPP_DEBUG(get_logger(),
            "[水平化] 相対ゲート却下 (残差 %.4f vs 基準 %.4f×%.2f / セル %d vs 基準 %d×%.2f)",
            gp.resid_m, resid_ref, ground_level_resid_rel_k_param_,
            gp.n_cells, cells_ref, ground_level_cells_rel_k_param_);
        }
      }
      if (rel_reject) {
        // 連続却下が続いたら基準自体が古い (サイトが変わった) 可能性がある。
        // ただし一気にリセットすると「一時的に地面が見えない区間」を全部通してしまうので
        // (20260727 の前半 135 フレームで実測)、参照を段階的に緩めるだけにする。
        // 直近平面の保持は続くので、緩むまでの間も出力は止まらない。
        if (++plane_rel_reject_run_ >= ground_level_reacquire_frames_param_ &&
            !plane_ref_resid_.empty()) {
          const float relaxed_resid =
            *std::min_element(plane_ref_resid_.begin(), plane_ref_resid_.end()) * 1.15f;
          const int relaxed_cells = static_cast<int>(
            *std::max_element(plane_ref_cells_.begin(), plane_ref_cells_.end()) * 0.85f);
          RCLCPP_WARN(get_logger(),
            "[水平化] 相対ゲートで %d フレーム連続却下 → 基準を緩める (残差 %.4f / セル %d)",
            plane_rel_reject_run_, relaxed_resid, relaxed_cells);
          plane_ref_resid_.assign(1, relaxed_resid);
          plane_ref_cells_.assign(1, relaxed_cells);
          plane_rel_reject_run_ = 0;
        }
      } else if (gp.valid) {
        plane_rel_reject_run_ = 0;
        plane_ref_resid_.push_back(gp.resid_m);
        plane_ref_cells_.push_back(gp.n_cells);
        const size_t w = static_cast<size_t>(std::max(1, ground_level_ref_window_param_));
        while (plane_ref_resid_.size() > w) { plane_ref_resid_.pop_front(); }
        while (plane_ref_cells_.size() > w) { plane_ref_cells_.pop_front(); }
      }

      if (!gp.valid) {
        // 却下理由を残す (セル不足 / 傾き超過 / 残差超過のどれで落ちたかを後から切り分けるため)
        const GroundPlane rej = gp;
        // 地面が見えないフレームは直近の有効な平面を保持する (0.5rpm 回転や一時遮蔽対策)
        gp = last_plane_;
        if (gp.valid) {
          RCLCPP_DEBUG(get_logger(),
            "[水平化] 平面フィット却下 (セル=%d 傾き=%.2fdeg 残差σ=%.3fm) → 直近値を保持",
            rej.n_cells, rej.tilt_deg, rej.resid_m);
        } else if (ground_level_require_plane_param_) {
          // 直近平面も無い = 出力すべき座標系がまだ確定していない。ここで固定 z に
          // フォールバックすると「水平化前の系」で publish してしまい、同じトピックに
          // 2 つの座標系が混ざる (実測 z の段差: 20260806 15m で 7cm、20260727 で 19cm)。
          // → このフレームは丸ごと捨てる。捨てたことが分母から消えないよう PERF は出す。
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[水平化] 平面フィット却下 (セル=%d 傾き=%.2fdeg 残差σ=%.3fm) かつ直近値なし "
            "→ 座標系が確定しないのでこのフレームは出力しない",
            rej.n_cells, rej.tilt_deg, rej.resid_m);
          if (perf_log_enable_param_) {
            const auto t_skip = Clock::now();
            auto ms = [](auto a, auto b) {
              return std::chrono::duration<double, std::milli>(b - a).count();
            };
            RCLCPP_INFO(get_logger(),
              "PERF raw=%zu filt=0 parse=%.1fms intens=0.0ms vizclust=0.0ms detect=0.0ms "
              "level=%.1fms(skip)",
              all.size(), ms(t0, t1), ms(t1, t_skip));
          }
          return;
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[水平化] 平面フィット却下 (セル=%d 傾き=%.2fdeg 残差σ=%.3fm) かつ直近値なし → 固定 z フィルタにフォールバック "
            "(★出力座標系が水平化前の系に戻る: ground_level_require_plane=false 指定時のみ)",
            rej.n_cells, rej.tilt_deg, rej.resid_m);
        }
      } else if (last_plane_.valid && ground_level_lpf_alpha_param_ < 1.0f) {
        // 平面係数を EMA で平滑化 (推定ノイズが回転ジッタ → 検出位置ジッタになるのを抑える)
        const float al = std::max(0.0f, ground_level_lpf_alpha_param_);
        gp.a = al * gp.a + (1.0f - al) * last_plane_.a;
        gp.b = al * gp.b + (1.0f - al) * last_plane_.b;
        gp.c = al * gp.c + (1.0f - al) * last_plane_.c;
        last_plane_ = gp;
      } else {
        last_plane_ = gp;
      }
      if (gp.valid) {
        float R[9];
        levelRotation(gp.a, gp.b, R);
        std::copy(R, R + 9, cur_R_);
        cur_tilt_deg_ = gp.tilt_deg;
        for (auto& p : all) {
          const float x = p.x, y = p.y, z = p.z;
          p.x = R[0]*x + R[1]*y + R[2]*z;
          p.y = R[3]*x + R[4]*y + R[5]*z;
          p.z = R[6]*x + R[7]*y + R[8]*z;
        }
        ground_z_level = gp.c / std::sqrt(gp.a*gp.a + gp.b*gp.b + 1.0f);
        leveled = true;
        cur_leveled_ = true;
        cur_ground_z_ = ground_z_level;
        RCLCPP_DEBUG(get_logger(),
          "[水平化] a=%+.5f b=%+.5f 傾き=%.2fdeg 残差σ=%.3fm セル=%d 地面z=%.3fm",
          gp.a, gp.b, gp.tilt_deg, gp.resid_m, gp.n_cells, ground_z_level);
      }
    }

    // ③ 地面下フィルタ: 床/床反射虚像を除去。上端側は decode_top_referenced
    //    (z_high 基準 + 既知高窓) が扱うため上端クリップは無し。
    //    水平化が効いていれば推定地面 + margin、そうでなければ従来の固定 z。
    if (ground_filter_enable_param_) {
      const float z_cut = (leveled && ground_filter_use_plane_param_)
                            ? ground_z_level + ground_filter_margin_param_
                            : ground_filter_z_min_param_;
      all.erase(std::remove_if(all.begin(), all.end(),
                               [z_cut](const Point4& p) { return p.z < z_cut; }),
                all.end());
      if (all.empty()) return;
    }
    const auto t1b = Clock::now();

    auto filtered  = intensityFilter(all);
    if (filtered.empty()) return;
    const auto t2 = Clock::now();
    const auto t3 = Clock::now();
    auto landmarks = detectLandmarks(filtered, all);
    const auto t4 = Clock::now();

    if (perf_log_enable_param_) {
      auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
      };
      RCLCPP_INFO(get_logger(),
        "PERF raw=%zu filt=%zu parse=%.1fms intens=%.1fms vizclust=%.1fms detect=%.1fms "
        "level=%.1fms(%s)",
        all.size(), filtered.size(),
        ms(t0, t1), ms(t1b, t2), ms(t2, t3), ms(t3, t4),
        ms(t1, t1b), leveled ? "on" : "off");
    }

    // 時間ヒステリシス: 1フレーム誤識別 (z_low 上振れ → BIN_4 化け等)
    // を多数決で抑制する。matched_id を "" にクリアすることで以下の
    // detected_positions 構築段で破棄される。
    if (hysteresis_enable_param_) {
      applyHysteresis(landmarks);
    }

    // ── per-frame 検出結果の集約 ──
    // 同一IDが複数グループ (dedup 対象外の 0.5m 超離れた位置) でマッチした
    // 場合、少なくとも一方は誤識別なので両方破棄する (誤識別ゼロ優先)。
    std::map<std::string, int> id_counts;
    for (const auto& lm : landmarks) {
      if (!lm.matched_id.empty()) id_counts[lm.matched_id]++;
    }
    std::map<std::string, Centroid> detected_positions;
    for (const auto& lm : landmarks) {
      if (lm.matched_id.empty()) continue;
      if (same_id_collision_discard_param_ && id_counts[lm.matched_id] > 1) {
        RCLCPP_WARN(get_logger(),
          "同一ID %s が %d 箇所で検出 xy=(%.2f,%.2f) → 全て破棄 (誤識別疑い)",
          lm.matched_id.c_str(), id_counts[lm.matched_id],
          lm.group_centroid.x, lm.group_centroid.y);
        continue;
      }
      detected_positions[lm.matched_id] = lm.group_centroid;
    }

    // ID-座標の対応をターミナル1行で表示 (検出ゼロの時は出力しない)
    if (!detected_positions.empty()) {
      std::string line = "[detections]";
      for (const auto& [bin_id, p] : detected_positions) {
        const auto map_it = BIN_TO_MARKER_ID.find(bin_id);
        const std::string& marker_id =
          (map_it != BIN_TO_MARKER_ID.end()) ? map_it->second : bin_id;
        char buf[96];
        std::snprintf(buf, sizeof(buf), " %s(%.3f, %.3f, %.3f)",
                      marker_id.c_str(), p.x, p.y, p.z);
        line += buf;
      }
      RCLCPP_INFO(get_logger(), "%s", line.c_str());
    }

    pub_poses_->publish(makePoseArray(msg->header, detected_positions));
    const auto matched = makeMatchedMarkerArray(msg->header, detected_positions);
    pub_matched_->publish(matched);

    // /landmark_ids: PoseStampedArray の配列順と1対1対応するID列 (CSV)
    std_msgs::msg::String ids_msg;
    for (size_t i = 0; i < matched.array.size(); i++) {
      if (i > 0) ids_msg.data += ",";
      ids_msg.data += matched.array[i].header.frame_id;
    }
    pub_ids_->publish(ids_msg);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CylinderLandmarkDetectorBin>());
  rclcpp::shutdown();
  return 0;
}
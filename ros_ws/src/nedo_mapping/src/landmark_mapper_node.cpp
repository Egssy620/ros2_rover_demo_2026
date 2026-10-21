/**
 * landmark_mapper_node.cpp
 *
 * 処理フロー:
 *   全点群
 *     ↓
 *   強度フィルタ
 *     ↓
 *   LM識別（2つ目のコードのビン方式に差し替え）:
 *     2D XY グループ化（Z無視）→ Z幅ゲート
 *       → スケール適応型アンカー/ビン境界推定
 *       → 円柱XY半径内のビン別点数カウント
 *       → 4ビットBINパターン生成 → ハミング距離照合 → 誤識別ガード
 *       → 同一円柱の重複除去（z_span大を採用）
 *     ↓
 *   [--circle-fit]                        [--no-circle-fit]
 *   グループ点群を統合                       グループ重心(XY)をそのまま使用
 *   → RANSAC で円フィット → 中心(cx,cy)
 *     ↓
 *   CSV出力
 *
 * ※ RANSAC（fitCircle2D / fitCircleRANSAC）は変更していません。
 *
 * ビルド:
 *   g++ -O2 -std=c++17 landmark_mapper_node.cpp -o landmark_mapper_node
 *
 * 使い方:
 *   ./landmark_mapper_node <keyframes_dir> <transformations_pcd> <output_csv>
 */

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <cstring>
#include <array>
#include <numeric>
#include <random>

namespace fs = std::filesystem;

// =============================================================================
// パラメータ
// =============================================================================
namespace Params
{
  // --- 強度フィルタ（LM識別の前処理。2つ目のコードに合わせて 0.80）---
  constexpr float INTENSITY_RATIO        = 0.80f;
  constexpr float INTENSITY_ABS_MIN      = 160.0f;

  // =====================================================================
  //  ↓↓↓ ここから LM識別（2つ目のコード cylinder_landmark_detector_bin の方式）
  // =====================================================================

  // --- ランドマーク物理仕様 ---
  //   上端アンカー30cm + 中間ビン30cm×4 + 下端アンカー30cm = 全高 1.80m
  //   expected_dist = 1.80 − 2×BIN_INNER_MARGIN(0.05) = 1.70m
  constexpr float ANCHOR_HEIGHT          = 0.30f;   // 上下端アンカーの高さ [m]
  constexpr float BIN_HEIGHT             = 0.30f;   // 中間ビンの高さ      [m]
  constexpr int   BIN_COUNT              = 4;       // 中間ビンの数
  constexpr float LANDMARK_WIDTH         = 0.40f;   // 横幅 [m]

  // 高さ判定の許容幅（非対称）
  constexpr float HEIGHT_TOLERANCE_LOW   = 0.15f;
  constexpr float HEIGHT_TOLERANCE_HIGH  = 0.15f;

  // --- 2D XY グループ化 ---
  constexpr float GROUP_XY_THRESHOLD     = 0.30f;   // XY BFS マージ距離 [m]
  constexpr int   MIN_GROUP_POINTS       = 3;       // グループ最小点数

  // --- ビン判定（円柱XY半径内の点数カウント方式）---
  constexpr float CYLINDER_XY_RADIUS     = 0.20f;   // 円柱表面までのXY許容 [m]
  constexpr int   BIN_MIN_POINTS         = 1;       // 反射材ありと判定する最小点数
  constexpr float BIN_INNER_MARGIN       = 0.05f;   // ビン両端の内側余白（漏れ込み対策）[m]

  // --- パターンマッチング ---
  constexpr int   HAMMING_MAX_DIST       = 1;

  // =====================================================================
  //  ↑↑↑ ここまで LM識別
  // =====================================================================

  // --- 円フィッティング（RANSAC）--- ※変更なし
  // ON/OFF は --circle-fit / --no-circle-fit で切り替え（デフォルトON）
  constexpr int   CIRCLE_RANSAC_ITERATIONS  = 1000;
  constexpr float CIRCLE_RANSAC_THRESHOLD   = 0.02f;   // インライア距離閾値 [m]
  constexpr int   CIRCLE_RANSAC_MIN_INLIERS = 4;
  constexpr float CIRCLE_RADIUS_MIN         = 0.02f;   // 円柱半径の下限 [m]
  constexpr float CIRCLE_RADIUS_MAX         = 0.20f;   // 円柱半径の上限 [m]
}

// =============================================================================
// IDパターン定義（2つ目のコードのビン方式辞書）
//   偶数パリティコード: 全28ペアでハミング距離 ≥ 2 を保証
// =============================================================================
const std::map<std::string, std::vector<int>> KNOWN_BIN_PATTERNS = {
  {"BIN_1", {0, 0, 0, 0}},
  {"BIN_2", {1, 1, 0, 0}},
  {"BIN_3", {0, 1, 1, 0}},
  {"BIN_4", {0, 0, 1, 1}},
  {"BIN_5", {1, 0, 1, 0}},
  {"BIN_6", {0, 1, 0, 1}},
  {"BIN_7", {1, 0, 0, 1}},
  {"BIN_8", {1, 1, 1, 1}},
};

// =============================================================================
// 基本構造体
// =============================================================================
struct Point4  { float x, y, z, intensity; };
struct Centroid{ float x, y, z; };
struct Pose6D  { float x,y,z,roll,pitch,yaw; double time; };
using  Mat4 = std::array<std::array<float,4>,4>;

// =============================================================================
// 姿勢 → 変換行列
// =============================================================================
static Mat4 poseToMatrix(const Pose6D& p)
{
  float cr=std::cos(p.roll), sr=std::sin(p.roll);
  float cp=std::cos(p.pitch),sp=std::sin(p.pitch);
  float cy=std::cos(p.yaw),  sy=std::sin(p.yaw);
  Mat4 T{};
  T[0][0]=cy*cp; T[0][1]=cy*sp*sr-sy*cr; T[0][2]=cy*sp*cr+sy*sr; T[0][3]=p.x;
  T[1][0]=sy*cp; T[1][1]=sy*sp*sr+cy*cr; T[1][2]=sy*sp*cr-cy*sr; T[1][3]=p.y;
  T[2][0]=-sp;   T[2][1]=cp*sr;          T[2][2]=cp*cr;           T[2][3]=p.z;
  T[3][3]=1;
  return T;
}

static Centroid transformPoint(const Mat4& T, float x, float y, float z)
{
  return {
    T[0][0]*x+T[0][1]*y+T[0][2]*z+T[0][3],
    T[1][0]*x+T[1][1]*y+T[1][2]*z+T[1][3],
    T[2][0]*x+T[2][1]*y+T[2][2]*z+T[2][3]
  };
}

// =============================================================================
// transformations.pcd 読み込み
// =============================================================================
static std::vector<Pose6D> loadTransformations(const std::string& path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("開けません: "+path);
  std::string line;
  while (std::getline(f,line)) if (line.rfind("DATA",0)==0) break;
  std::vector<Pose6D> poses;
  while (true) {
    char buf[36]; if (!f.read(buf,36)) break;
    float x,y,z,intensity,roll,pitch,yaw; double time;
    std::memcpy(&x,buf,4);   std::memcpy(&y,buf+4,4);  std::memcpy(&z,buf+8,4);
    std::memcpy(&intensity,buf+12,4);
    std::memcpy(&roll,buf+16,4); std::memcpy(&pitch,buf+20,4); std::memcpy(&yaw,buf+24,4);
    std::memcpy(&time,buf+28,8);
    poses.push_back({x,y,z,roll,pitch,yaw,time});
  }
  std::cout<<"  姿勢読み込み: "<<poses.size()<<" フレーム\n";
  return poses;
}

// =============================================================================
// pcd 読み込み（ASCII / Binary 両対応）
// =============================================================================
static std::vector<Point4> loadPCD(const std::string& path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) return {};
  std::vector<std::string> fields; std::vector<int> sizes;
  int n_points=0; bool is_binary=false; std::string line;
  while (std::getline(f,line)) {
    if      (line.rfind("FIELDS",0)==0){std::istringstream ss(line.substr(7));std::string t;while(ss>>t)fields.push_back(t);}
    else if (line.rfind("SIZE",0)==0)  {std::istringstream ss(line.substr(5));int v;while(ss>>v)sizes.push_back(v);}
    else if (line.rfind("POINTS",0)==0){std::istringstream ss(line.substr(7));ss>>n_points;}
    else if (line.rfind("DATA",0)==0)  {is_binary=(line.find("binary")!=std::string::npos);break;}
  }
  int xi=-1,yi=-1,zi=-1,ii=-1;
  for (int k=0;k<(int)fields.size();k++){
    if(fields[k]=="x")xi=k; else if(fields[k]=="y")yi=k;
    else if(fields[k]=="z")zi=k; else if(fields[k]=="intensity")ii=k;
  }
  if (xi<0||yi<0||zi<0) return {};
  std::vector<Point4> pts; pts.reserve(n_points);
  if (is_binary) {
    int psz=0; for(int s:sizes)psz+=s;
    std::vector<int> off(sizes.size(),0);
    for(int k=1;k<(int)sizes.size();k++) off[k]=off[k-1]+sizes[k-1];
    std::vector<char> buf(psz);
    for(int i=0;i<n_points;i++){
      if(!f.read(buf.data(),psz))break;
      float x,y,z,intensity=0;
      std::memcpy(&x,buf.data()+off[xi],4); std::memcpy(&y,buf.data()+off[yi],4);
      std::memcpy(&z,buf.data()+off[zi],4);
      if(ii>=0)std::memcpy(&intensity,buf.data()+off[ii],4);
      if(!std::isnan(x)&&!std::isnan(y)&&!std::isnan(z))
        pts.push_back({x,y,z,intensity});
    }
  } else {
    for(int i=0;i<n_points;i++){
      if(!std::getline(f,line))break;
      std::istringstream ss(line); std::vector<float> v; float t;
      while(ss>>t)v.push_back(t);
      if((int)v.size()<=std::max({xi,yi,zi}))continue;
      float intensity=(ii>=0&&ii<(int)v.size())?v[ii]:0.0f;
      pts.push_back({v[xi],v[yi],v[zi],intensity});
    }
  }
  return pts;
}

// =============================================================================
// 円フィッティング（最小二乗）  ※変更なし
// =============================================================================
// 戻り値: true=成功, cx/cy/r に結果を格納
static bool fitCircle2D(const std::vector<std::pair<float,float>>& pts,
                        float& cx, float& cy, float& r)
{
  const int n = static_cast<int>(pts.size());
  if (n < 3) return false;

  // Ax = b  (A=[2x, 2y, 1], b=[x²+y²])
  double A00=0,A01=0,A02=0,A11=0,A12=0,A22=n;
  double b0=0,b1=0,b2=0;
  for (auto& [x,y] : pts) {
    double x2=x*x, y2=y*y, xy=x*y, s=x2+y2;
    A00+=x2*4; A01+=xy*4; A02+=x*2;
    A11+=y2*4; A12+=y*2;
    b0+=s*x*2; b1+=s*y*2; b2+=s;
  }
  // 3×3対称行列の逆行列（クラメルの公式）
  double det = A00*(A11*A22-A12*A12)
             - A01*(A01*A22-A12*A02)
             + A02*(A01*A12-A11*A02);
  if (std::abs(det) < 1e-10) return false;

  double inv_det = 1.0/det;
  double cx_ = inv_det*( b0*(A11*A22-A12*A12)
                        -b1*(A01*A22-A12*A02)
                        +b2*(A01*A12-A11*A02));
  double cy_ = inv_det*(-b0*(A01*A22-A02*A12)
                        +b1*(A00*A22-A02*A02)
                        -b2*(A00*A12-A01*A02));
  double c_  = inv_det*( b0*(A01*A12-A02*A11)
                        -b1*(A00*A12-A01*A02)
                        +b2*(A00*A11-A01*A01));
  double r2 = cx_*cx_ + cy_*cy_ + c_;
  if (r2 < 0) return false;
  cx = static_cast<float>(cx_);
  cy = static_cast<float>(cy_);
  r  = static_cast<float>(std::sqrt(r2));
  return true;
}

// =============================================================================
// 円フィッティング RANSAC  ※変更なし
// 入力: グループ点群（センサ座標系）
// 出力: 成功時 true, cx/cy に円柱中心
// =============================================================================
static bool fitCircleRANSAC(const std::vector<int>& cluster,
                            const std::vector<Point4>& pts,
                            float& cx, float& cy, float& r_out)
{
  const int n = static_cast<int>(cluster.size());
  if (n < Params::CIRCLE_RANSAC_MIN_INLIERS) return false;

  std::vector<std::pair<float,float>> xy;
  xy.reserve(n);
  for (int i : cluster) xy.push_back({pts[i].x, pts[i].y});

  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist(0, n-1);

  int   best_inliers = 0;
  float best_cx=0, best_cy=0, best_r=0;

  for (int iter=0; iter<Params::CIRCLE_RANSAC_ITERATIONS; iter++) {
    // 3点をランダムサンプリング
    int i0=dist(rng), i1=dist(rng), i2=dist(rng);
    if (i0==i1||i1==i2||i0==i2) continue;

    std::vector<std::pair<float,float>> sample = {xy[i0], xy[i1], xy[i2]};
    float tcx, tcy, tr;
    if (!fitCircle2D(sample, tcx, tcy, tr)) continue;
    if (tr < Params::CIRCLE_RADIUS_MIN || tr > Params::CIRCLE_RADIUS_MAX) continue;

    // インライア数をカウント
    int inliers = 0;
    for (auto& [x,y] : xy) {
      float d = std::abs(std::sqrt((x-tcx)*(x-tcx)+(y-tcy)*(y-tcy)) - tr);
      if (d < Params::CIRCLE_RANSAC_THRESHOLD) inliers++;
    }
    if (inliers > best_inliers) {
      best_inliers = inliers; best_cx=tcx; best_cy=tcy; best_r=tr;
    }
  }

  if (best_inliers < Params::CIRCLE_RANSAC_MIN_INLIERS) return false;

  // インライア全点で最終フィッティング
  std::vector<std::pair<float,float>> inlier_pts;
  for (auto& [x,y] : xy) {
    float d = std::abs(std::sqrt((x-best_cx)*(x-best_cx)+(y-best_cy)*(y-best_cy))-best_r);
    if (d < Params::CIRCLE_RANSAC_THRESHOLD) inlier_pts.push_back({x,y});
  }
  float fcx, fcy, fr;
  if (!fitCircle2D(inlier_pts, fcx, fcy, fr)) { cx=best_cx; cy=best_cy; r_out=best_r; }
  else { cx=fcx; cy=fcy; r_out=fr; }

  return true;
}

// =============================================================================
// 強度フィルタ
// =============================================================================
static std::vector<Point4> intensityFilter(const std::vector<Point4>& pts)
{
  float mx=0;
  for (auto& p:pts) mx=std::max(mx,p.intensity);
  float thr=std::max(mx*Params::INTENSITY_RATIO,Params::INTENSITY_ABS_MIN);
  std::vector<Point4> res;
  for (auto& p:pts) if(p.intensity>thr) res.push_back(p);
  return res;
}

// =============================================================================
// パターンマッチング（ハミング距離）
// =============================================================================
static std::string matchPattern(const std::vector<int>& pattern)
{
  int best_dist=999; std::string best_id; bool ambiguous=false;
  for (const auto& [id,known]:KNOWN_BIN_PATTERNS){
    if (known.size()!=pattern.size()) continue;
    int d=0;
    for (size_t k=0;k<pattern.size();k++) if(pattern[k]!=known[k]) d++;
    if (d<best_dist){best_dist=d;best_id=id;ambiguous=false;}
    else if(d==best_dist) ambiguous=true;
  }
  if (best_dist<=Params::HAMMING_MAX_DIST && !ambiguous) return best_id;
  return "";
}

// =============================================================================
// LandmarkGroup
// =============================================================================
struct LandmarkGroup {
  Centroid          group_centroid; // 最終的な中心座標（円フィット or 重心XY）
  std::vector<int>  pattern;        // 4ビットBINパターン
  std::string       matched_id;     // BIN_x（未識別なら空）
  float             z_span;         // 群Z幅（重複除去・信頼性評価用）
  bool              circle_fit_ok;  // 円フィッティング成功フラグ
  float             circle_radius;  // 推定半径（参考値）
};

// =============================================================================
// ランドマーク検出（2つ目のコードのビン方式に差し替え）
//
//  手順:
//    ① XY平面で高強度点を BFS グループ化（Z距離を無視）
//    ② グループの Z スパン ≈ expected_dist(1.70m) で候補絞り込み
//    ③ スケール適応型アンカー（帯内z重心）/ ビン境界推定
//    ④ アンカー中心間を (BIN_COUNT+1) 等分して各ビン中心を内挿
//    ⑤ 円柱XY半径内かつビン[k]のZ範囲内の点数を集計
//    ⑥ KNOWN_BIN_PATTERNS と照合（ハミング距離）→ 誤識別ガード
//    ── 中心座標の決定（RANSAC部分は変更なし）──
//    ⑦ 同一円柱からの重複検出を z_span で除去
//
// circle_fit: true=RANSACで円フィット, false=重心(XY)ベース
// =============================================================================
static std::vector<LandmarkGroup> detectLandmarks(
  const std::vector<Point4>& pts,
  bool circle_fit)
{
  if (pts.empty()) return {};
  const size_t N = pts.size();

  // --- ステップ①: 2D XY BFS グループ化 ---
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
        float dx = pts[cur].x - pts[j].x, dy = pts[cur].y - pts[j].y;
        if (dx * dx + dy * dy <= xy_d2) { gid[j] = ng; q.push((int)j); }
      }
    }
    ng++;
  }
  std::vector<std::vector<int>> xy_groups(ng);
  for (size_t i = 0; i < N; i++) xy_groups[gid[i]].push_back((int)i);

  // 期待Z幅: 全高 1.80m − 2×BIN_INNER_MARGIN（端の取りこぼし経験補正）
  const float expected_dist =
    2.0f * Params::ANCHOR_HEIGHT + Params::BIN_COUNT * Params::BIN_HEIGHT
    - 2.0f * Params::BIN_INNER_MARGIN;

  std::vector<LandmarkGroup> result;
  const float r2 = Params::CYLINDER_XY_RADIUS * Params::CYLINDER_XY_RADIUS;

  for (const auto& grp : xy_groups) {
    if ((int)grp.size() < Params::MIN_GROUP_POINTS) continue;

    // --- ステップ②: Z 分布を解析 ---
    std::vector<float> zv; zv.reserve(grp.size());
    float sx = 0.0f, sy = 0.0f;
    for (int idx : grp) { zv.push_back(pts[idx].z); sx += pts[idx].x; sy += pts[idx].y; }
    std::sort(zv.begin(), zv.end());
    const int n = (int)zv.size();

    // 外れ値耐性: 点数8以上の場合のみ上下各1点を除外
    const int k = (n >= 8) ? 1 : 0;
    const float z_low  = zv[k];
    const float z_high = zv[n - 1 - k];
    const float z_span = z_high - z_low;

    if (z_span < expected_dist - Params::HEIGHT_TOLERANCE_LOW ||
        z_span > expected_dist + Params::HEIGHT_TOLERANCE_HIGH)
      continue;

    const float cx = sx / n, cy = sy / n;

    // --- ステップ③: スケール適応型アンカー・ビン境界推定 ---
    const float scale =
      z_span / (2.0f * Params::ANCHOR_HEIGHT +
                Params::BIN_COUNT * Params::BIN_HEIGHT);
    const float scaled_anchor = Params::ANCHOR_HEIGHT * scale;

    const float anchor_frac =
      Params::ANCHOR_HEIGHT /
      (2.0f * Params::ANCHOR_HEIGHT + Params::BIN_COUNT * Params::BIN_HEIGHT);
    const float ba_hi = z_low  + z_span * anchor_frac;   // 下端アンカー帯 上限
    const float ta_lo = z_high - z_span * anchor_frac;   // 上端アンカー帯 下限

    float ba_sum = 0.0f, ta_sum = 0.0f;
    int bot_cnt = 0, top_cnt = 0;
    for (int idx : grp) {
      const float z = pts[idx].z;
      if (z <= ba_hi) { ba_sum += z; bot_cnt++; }
      if (z >= ta_lo) { ta_sum += z; top_cnt++; }
    }
    // アンカー中心 = 帯内点の z 重心（帯が空なら幾何推定にフォールバック）
    const float z_ba = (bot_cnt > 0) ? ba_sum / bot_cnt
                                     : z_low  + scaled_anchor * 0.5f;
    const float z_ta = (top_cnt > 0) ? ta_sum / top_cnt
                                     : z_high - scaled_anchor * 0.5f;

    // アンカー中心間隔を (BIN_COUNT+1) 等分 = 隣接バンド中心間隔 s
    const float s = (z_ta - z_ba) / (Params::BIN_COUNT + 1);
    const float scale_local = s / Params::BIN_HEIGHT;
    const float half_w =
      (Params::BIN_HEIGHT * 0.5f - Params::BIN_INNER_MARGIN) * scale_local;

    // --- ステップ④: ビン有効Z範囲（アンカー中心からの内挿）---
    std::vector<float> bl(Params::BIN_COUNT), bh(Params::BIN_COUNT);
    for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
      const float center = z_ba + s * (kk + 1);
      bl[kk] = center - half_w;
      bh[kk] = center + half_w;
    }

    // --- ステップ⑤: 全高強度点をビン別にカウント ---
    std::vector<int> bin_counts(Params::BIN_COUNT, 0);
    for (const auto& p : pts) {
      const float dx = p.x - cx, dy = p.y - cy;
      if (dx * dx + dy * dy > r2) continue;
      for (int kk = 0; kk < Params::BIN_COUNT; kk++) {
        if (p.z >= bl[kk] && p.z < bh[kk]) { bin_counts[kk]++; break; }
      }
    }

    // --- ステップ⑥: パターン生成 & マッチング ---
    std::vector<int> pattern(Params::BIN_COUNT);
    for (int kk = 0; kk < Params::BIN_COUNT; kk++)
      pattern[kk] = (bin_counts[kk] >= Params::BIN_MIN_POINTS) ? 1 : 0;

    std::string matched_id = matchPattern(pattern);

    // ステップ⑥-b: 誤識別ガード（両端アンカーが両方とも0点なら棄却）
    if (!matched_id.empty() && bot_cnt == 0 && top_cnt == 0)
      matched_id.clear();

    // --- 中心座標の決定（LM識別の後）--- ※RANSAC部分は変更なし ---
    LandmarkGroup g;
    g.pattern       = pattern;
    g.matched_id    = matched_id;
    g.z_span        = z_span;
    g.circle_fit_ok = false;
    g.circle_radius = 0.0f;
    const float gz  = (zv.front() + zv.back()) * 0.5f;

    if (circle_fit) {
      // グループ内の全点群を統合してRANSAC
      // 観測角度のカバレッジが広がり精度向上
      float fcx, fcy, fr;
      if (fitCircleRANSAC(grp, pts, fcx, fcy, fr)) {
        g.group_centroid = {fcx, fcy, gz};
        g.circle_fit_ok  = true;
        g.circle_radius  = fr;
      } else {
        // フォールバック: 重心(XY)ベース
        g.group_centroid = {cx, cy, gz};
      }
    } else {
      // 重心(XY)ベース
      g.group_centroid = {cx, cy, gz};
    }

    result.push_back(g);
  }

  // --- ステップ⑦: 同一円柱からの重複検出を除去（z_span大を採用）---
  constexpr float DEDUP_XY_DIST_SQ = 0.50f * 0.50f;
  std::vector<bool> active(result.size(), true);
  for (size_t i = 0; i < result.size(); i++) {
    if (!active[i] || result[i].matched_id.empty()) continue;
    for (size_t j = i + 1; j < result.size(); j++) {
      if (!active[j] || result[j].matched_id.empty()) continue;
      const float dx = result[i].group_centroid.x - result[j].group_centroid.x;
      const float dy = result[i].group_centroid.y - result[j].group_centroid.y;
      if (dx * dx + dy * dy >= DEDUP_XY_DIST_SQ) continue;
      // 同一円柱と判定: z_spanが大きい方を採用
      const bool keep_i = result[i].z_span >= result[j].z_span;
      if (keep_i) active[j] = false;
      else { active[i] = false; break; }
    }
  }
  std::vector<LandmarkGroup> deduped;
  deduped.reserve(result.size());
  for (size_t i = 0; i < result.size(); i++)
    if (active[i]) deduped.push_back(result[i]);

  return deduped;
}

// =============================================================================
// メイン
// =============================================================================
int main(int argc, char* argv[])
{
  // --- 引数パース ---
  // 必須: <keyframes_dir> <transformations_pcd> <output_csv>
  // オプション: --circle-fit (デフォルト) / --no-circle-fit

  std::string keyframes_dir, transforms_path, output_csv;
  bool circle_fit = true;  // デフォルトON

  std::vector<std::string> args(argv+1, argv+argc);
  std::vector<std::string> positional;
  for (auto& a : args) {
    if      (a == "--circle-fit")    circle_fit = true;
    else if (a == "--no-circle-fit") circle_fit = false;
    else                             positional.push_back(a);
  }

  if (positional.size() < 3) {
    std::cerr
      << "使い方: " << argv[0]
      << " <keyframes_dir> <transformations_pcd> <output_csv>"
      << " [--circle-fit|--no-circle-fit]\n"
      << "例:\n"
      << "  " << argv[0]
      << " /tmp/keyframes /tmp/transformations.pcd /tmp/obs.csv\n"
      << "  " << argv[0]
      << " /tmp/keyframes /tmp/transformations.pcd /tmp/obs.csv --no-circle-fit\n";
    return 1;
  }
  keyframes_dir   = positional[0];
  transforms_path = positional[1];
  output_csv      = positional[2];

  std::cout << "================================================\n"
            << " ランドマークマッピング（LM識別: ビン方式）\n"
            << "  円フィッティング: " << (circle_fit?"ON (RANSAC)":"OFF (重心ベース)") << "\n"
            << "================================================\n";

  // --- 姿勢読み込み ---
  std::cout << "\n[1/3] 姿勢データ読み込み中...\n";
  std::vector<Pose6D> poses;
  try { poses = loadTransformations(transforms_path); }
  catch (const std::exception& e) { std::cerr << "ERROR: " << e.what() << "\n"; return 1; }

  // --- CSV出力準備 ---
  // circle_fit_ok列: 円フィッティングOFFの場合は常に0
  std::ofstream csv(output_csv);
  if (!csv) { std::cerr << "ERROR: CSV を開けません: " << output_csv << "\n"; return 1; }
  csv << "frame_index,lm_id,"
      << "x_world,y_world,z_world,"
      << "x_sensor,y_sensor,z_sensor,"
      << "distance,circle_fit_ok\n";

  // --- キーフレームを順番に処理 ---
  std::cout << "\n[2/3] キーフレーム処理中...\n";
  int total_obs = 0;

  for (int i=0; i<(int)poses.size(); i++) {
    std::string pcd_path = keyframes_dir+"/raw_frame_"+std::to_string(i)+".pcd";
    if (!fs::exists(pcd_path)) continue;

    auto pts = loadPCD(pcd_path);
    if (pts.empty()) continue;

    auto filtered  = intensityFilter(pts);
    if (filtered.empty()) continue;

    // ビン方式は内部で 2D XY グループ化を行うため、3D クラスタリングは不要
    auto landmarks = detectLandmarks(filtered, circle_fit);

    const Mat4 T = poseToMatrix(poses[i]);

    for (auto& lm : landmarks) {
      if (lm.matched_id.empty()) continue;

      const Centroid& sc = lm.group_centroid;
      const Centroid  wc = transformPoint(T, sc.x, sc.y, sc.z);
      const float dist   = std::sqrt(sc.x*sc.x+sc.y*sc.y+sc.z*sc.z);

      csv << i << ","
          << lm.matched_id << ","
          << std::fixed << std::setprecision(6)
          << wc.x << "," << wc.y << "," << wc.z << ","
          << sc.x << "," << sc.y << "," << sc.z << ","
          << dist << ","
          << (lm.circle_fit_ok ? 1 : 0) << "\n";
      total_obs++;
    }

    if ((i+1)%20==0||(i+1)==(int)poses.size())
      std::cout << "  " << (i+1) << "/" << poses.size()
                << " フレーム完了 (観測数: " << total_obs << ")\n";
  }

  std::cout << "\n[3/3] 完了\n"
            << "  総観測数: " << total_obs << "\n"
            << "  出力:     " << output_csv << "\n";
  return 0;
}
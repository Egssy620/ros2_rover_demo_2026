/**
 * landmark_bundle_adjustment.cpp
 *
 * 目的:
 *   LIO-SAM が出力する自己位置（transformations.pcd）と
 *   landmark_mapper_node.cpp が出力するランドマーク観測（CSV）を入力に、
 *   バンドル調整によって高精度なランドマーク地図を生成する。
 *
 * 方針:
 *   - LIO-SAM の自己位置（各キーフレーム姿勢）は「完全に信頼」して定数扱い。
 *     最適化する変数はランドマークの世界座標 p_L（3次元）のみ。
 *   - 各観測の残差は、最適化中のランドマーク世界座標を「固定された姿勢」で
 *     センサ座標系へ逆投影し、観測されたセンサ座標との差で定義する
 *     （姿勢が固定なので世界座標系で差を取るのと数値的に等価だが、
 *      観測ノイズが乗るセンサ座標系で残差を定義する正攻法を採用）。
 *
 * 外れ値対策:
 *   1) 距離による重み付け
 *        観測距離 d に応じてノイズ標準偏差を σ(d)=σ0 + k·d とモデル化し、
 *        残差を 1/σ(d) で正規化する。遠方（=ノイズ大）の観測を相対的に減衰。
 *   2) Huber 損失
 *        正規化後の残差（σ 単位）に Huber 損失を適用し、
 *        大きく外れた観測の影響を線形に抑える。
 *
 * 入力:
 *   <observations_csv>   landmark_mapper_node の出力 CSV
 *                        ヘッダ: frame_index,lm_id,x_world,y_world,z_world,
 *                                x_sensor,y_sensor,z_sensor,distance,circle_fit_ok
 *   <transformations_pcd> LIO-SAM の姿勢列（landmark_mapper_node と同形式）
 *
 * 出力:
 *   <output_map_csv>     最適化後のランドマーク地図
 *                        ヘッダ: lm_id,x,y,z,num_obs,num_inliers,
 *                                rms_before,rms_after
 *
 * ビルド（単体）:
 *   g++ -O2 -std=c++17 landmark_bundle_adjustment.cpp -o landmark_bundle_adjustment \
 *       -I/usr/include/eigen3 -lceres -lglog
 *
 * 使い方:
 *   ./landmark_bundle_adjustment <observations_csv> <transformations_pcd> <output_map_csv>
 */

#include <ceres/ceres.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// =============================================================================
// パラメータ（ノイズモデル / ロバスト化）
// =============================================================================
namespace Params
{
  // --- 距離による重み付け ---
  // 観測ノイズ標準偏差 σ(d) = SIGMA0 + SIGMA_SLOPE * d   [m]
  //   SIGMA0     : 近距離での基準ノイズ（測距分解能・中心推定誤差など）
  //   SIGMA_SLOPE: 距離あたりのノイズ増加率（≒測距の距離比例誤差）
  // 残差はこの σ(d) で正規化されるため、遠方の観測ほど重みが小さくなる。
  constexpr double SIGMA0      = 0.02;   // [m]
  constexpr double SIGMA_SLOPE = 0.01;   // [m/m]  (距離の 1%)
  constexpr double SIGMA_MIN   = 1e-3;   // σ の下限（ゼロ割回避）[m]

  // --- Huber 損失 ---
  // 残差は σ 単位（無次元）に正規化済みなので、delta は「何σで外れ値とみなすか」。
  constexpr double HUBER_DELTA = 1.5;    // [σ]

  // --- 最適化 ---
  constexpr int    MAX_ITERATIONS = 100;

  // --- インライア判定（出力統計用）---
  // 最終残差（σ 単位）がこの値以下なら inlier とカウント
  constexpr double INLIER_SIGMA = 2.5;   // [σ]

  // --- 観測数による足切り（CLI で 有効/無効・閾値 を選択）---
  // 観測数がこの値未満のランドマークは信頼できないとして出力から除外する。
  // 既定では無効（--cutoff / --min-obs で有効化）。
  constexpr int    MIN_OBS_DEFAULT = 3;
}

// =============================================================================
// 基本構造体
// =============================================================================
struct Pose6D { float x, y, z, roll, pitch, yaw; double time; };
using Mat3 = std::array<double, 9>;   // row-major 3x3

struct Observation
{
  int    frame_index;
  std::string lm_id;
  double sensor[3];   // 観測されたセンサ座標（x_sensor,y_sensor,z_sensor）
  double world[3];    // 参考: landmark_mapper が変換した世界座標
  double distance;    // センサからの距離 [m]
};

// =============================================================================
// 姿勢 → 回転行列（landmark_mapper_node.cpp と同一の RPY 規約）
// =============================================================================
static Mat3 poseToRotation(const Pose6D& p)
{
  double cr = std::cos(p.roll),  sr = std::sin(p.roll);
  double cp = std::cos(p.pitch), sp = std::sin(p.pitch);
  double cy = std::cos(p.yaw),   sy = std::sin(p.yaw);
  Mat3 R{};
  R[0] = cy*cp; R[1] = cy*sp*sr - sy*cr; R[2] = cy*sp*cr + sy*sr;
  R[3] = sy*cp; R[4] = sy*sp*sr + cy*cr; R[5] = sy*sp*cr - cy*sr;
  R[6] = -sp;   R[7] = cp*sr;            R[8] = cp*cr;
  return R;
}

// =============================================================================
// transformations.pcd 読み込み（landmark_mapper_node.cpp と同一フォーマット）
// =============================================================================
static std::vector<Pose6D> loadTransformations(const std::string& path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("開けません: " + path);
  std::string line;
  while (std::getline(f, line)) if (line.rfind("DATA", 0) == 0) break;
  std::vector<Pose6D> poses;
  while (true) {
    char buf[36];
    if (!f.read(buf, 36)) break;
    float x, y, z, intensity, roll, pitch, yaw; double time;
    std::memcpy(&x, buf, 4);     std::memcpy(&y, buf + 4, 4);   std::memcpy(&z, buf + 8, 4);
    std::memcpy(&intensity, buf + 12, 4);
    std::memcpy(&roll, buf + 16, 4); std::memcpy(&pitch, buf + 20, 4); std::memcpy(&yaw, buf + 24, 4);
    std::memcpy(&time, buf + 28, 8);
    (void)intensity;
    poses.push_back({x, y, z, roll, pitch, yaw, time});
  }
  return poses;
}

// =============================================================================
// 観測 CSV 読み込み
// =============================================================================
static std::vector<Observation> loadObservations(const std::string& path)
{
  std::ifstream f(path);
  if (!f) throw std::runtime_error("開けません: " + path);

  std::string line;
  // ヘッダ行をスキップ
  std::getline(f, line);

  std::vector<Observation> obs;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tok;
    std::vector<std::string> cols;
    while (std::getline(ss, tok, ',')) cols.push_back(tok);
    // frame_index,lm_id,x_world,y_world,z_world,x_sensor,y_sensor,z_sensor,distance,circle_fit_ok
    if (cols.size() < 9) continue;

    Observation o;
    o.frame_index = std::stoi(cols[0]);
    o.lm_id       = cols[1];
    o.world[0]    = std::stod(cols[2]);
    o.world[1]    = std::stod(cols[3]);
    o.world[2]    = std::stod(cols[4]);
    o.sensor[0]   = std::stod(cols[5]);
    o.sensor[1]   = std::stod(cols[6]);
    o.sensor[2]   = std::stod(cols[7]);
    o.distance    = std::stod(cols[8]);
    obs.push_back(o);
  }
  return obs;
}

// =============================================================================
// 距離による重み（= 1/σ(d)）
// =============================================================================
static double distanceWeight(double d)
{
  double sigma = Params::SIGMA0 + Params::SIGMA_SLOPE * d;
  if (sigma < Params::SIGMA_MIN) sigma = Params::SIGMA_MIN;
  return 1.0 / sigma;
}

// =============================================================================
// 残差: 固定姿勢でランドマーク世界座標をセンサ座標系へ逆投影し、観測との差を取る
//
//   sensor_pred = R^T * (p_L - t)          （R, t は固定された LIO-SAM 姿勢）
//   residual    = weight * (sensor_pred - sensor_obs)
//
//   weight = 1/σ(d) を残差に掛けることで距離重み付けを実現。
//   姿勢は定数としてファンクタ内に取り込み、最適化変数は p_L のみ。
// =============================================================================
struct LandmarkReprojectionResidual
{
  // R は world<-sensor の回転。逆投影では R^T を使うので転置を保持。
  double Rt[9];        // R^T (row-major)
  double t[3];         // 姿勢の並進
  double obs[3];       // 観測センサ座標
  double weight;       // 1/σ(d)

  LandmarkReprojectionResidual(const Mat3& R, const Pose6D& pose,
                               const double sensor_obs[3], double w)
  {
    // R^T を作る（R は row-major なので転置 = 添字入れ替え）
    Rt[0] = R[0]; Rt[1] = R[3]; Rt[2] = R[6];
    Rt[3] = R[1]; Rt[4] = R[4]; Rt[5] = R[7];
    Rt[6] = R[2]; Rt[7] = R[5]; Rt[8] = R[8];
    t[0] = pose.x; t[1] = pose.y; t[2] = pose.z;
    obs[0] = sensor_obs[0]; obs[1] = sensor_obs[1]; obs[2] = sensor_obs[2];
    weight = w;
  }

  template <typename T>
  bool operator()(const T* const p, T* residual) const
  {
    // d = p - t
    T d0 = p[0] - T(t[0]);
    T d1 = p[1] - T(t[1]);
    T d2 = p[2] - T(t[2]);
    // sensor_pred = R^T * d
    T sx = T(Rt[0]) * d0 + T(Rt[1]) * d1 + T(Rt[2]) * d2;
    T sy = T(Rt[3]) * d0 + T(Rt[4]) * d1 + T(Rt[5]) * d2;
    T sz = T(Rt[6]) * d0 + T(Rt[7]) * d1 + T(Rt[8]) * d2;

    residual[0] = T(weight) * (sx - T(obs[0]));
    residual[1] = T(weight) * (sy - T(obs[1]));
    residual[2] = T(weight) * (sz - T(obs[2]));
    return true;
  }

  static ceres::CostFunction* Create(const Mat3& R, const Pose6D& pose,
                                     const double sensor_obs[3], double w)
  {
    return new ceres::AutoDiffCostFunction<LandmarkReprojectionResidual, 3, 3>(
        new LandmarkReprojectionResidual(R, pose, sensor_obs, w));
  }
};

// =============================================================================
// 各軸メディアン（ロバストな初期値）
// =============================================================================
static void axisMedian(std::vector<double> v, double& out)
{
  if (v.empty()) { out = 0.0; return; }
  std::sort(v.begin(), v.end());
  out = v[v.size() / 2];
}

// =============================================================================
// メイン
// =============================================================================
int main(int argc, char* argv[])
{
  // --- 引数パース ---
  // 必須(位置引数): <observations_csv> <transformations_pcd> <output_map_csv>
  // オプション:
  //   --cutoff            観測数による足切りを有効化（閾値は MIN_OBS_DEFAULT）
  //   --no-cutoff         足切りを無効化（既定）
  //   --min-obs <N>       足切り閾値を N に設定し、足切りを有効化
  bool cutoff_enabled = false;
  int  min_obs        = Params::MIN_OBS_DEFAULT;

  std::vector<std::string> positional;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--cutoff") {
      cutoff_enabled = true;
    } else if (a == "--no-cutoff") {
      cutoff_enabled = false;
    } else if (a == "--min-obs") {
      if (i + 1 >= argc) {
        std::cerr << "ERROR: --min-obs の後に整数が必要です。\n";
        return 1;
      }
      min_obs = std::stoi(argv[++i]);
      cutoff_enabled = true;
    } else {
      positional.push_back(a);
    }
  }

  if (positional.size() < 3) {
    std::cerr << "使い方: " << argv[0]
              << " <observations_csv> <transformations_pcd> <output_map_csv>"
              << " [--cutoff|--no-cutoff] [--min-obs <N>]\n"
              << "例:\n  " << argv[0]
              << " /tmp/landmarks.csv /tmp/transformations.pcd /tmp/landmark_map.csv\n"
              << "  " << argv[0]
              << " /tmp/landmarks.csv /tmp/transformations.pcd /tmp/landmark_map.csv --min-obs 5\n";
    return 1;
  }
  const std::string obs_csv   = positional[0];
  const std::string poses_pcd = positional[1];
  const std::string out_csv   = positional[2];

  std::cout << "================================================\n"
            << " ランドマーク バンドル調整\n"
            << "  姿勢: 固定（LIO-SAM を完全信頼）\n"
            << "  外れ値対策: Huber損失(delta=" << Params::HUBER_DELTA
            << "σ) + 距離重み(σ=" << Params::SIGMA0 << "+" << Params::SIGMA_SLOPE << "·d)\n"
            << "  観測数による足切り: "
            << (cutoff_enabled ? ("ON (min_obs=" + std::to_string(min_obs) + ")") : std::string("OFF"))
            << "\n"
            << "================================================\n";

  // --- 入力読み込み ---
  std::vector<Pose6D> poses;
  std::vector<Observation> obs;
  try {
    poses = loadTransformations(poses_pcd);
    obs   = loadObservations(obs_csv);
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
  std::cout << "  姿勢: " << poses.size() << " フレーム\n"
            << "  観測: " << obs.size() << " 点\n";

  if (poses.empty() || obs.empty()) {
    std::cerr << "ERROR: 姿勢または観測が空です。\n";
    return 1;
  }

  // 各フレームの回転行列を事前計算
  std::vector<Mat3> R_all(poses.size());
  for (size_t i = 0; i < poses.size(); i++) R_all[i] = poseToRotation(poses[i]);

  // --- ランドマークごとに観測をグループ化 ---
  // lm_id -> 観測インデックス列
  std::map<std::string, std::vector<int>> lm_obs;
  for (int k = 0; k < (int)obs.size(); k++) {
    if (obs[k].frame_index < 0 || obs[k].frame_index >= (int)poses.size()) {
      std::cerr << "  警告: frame_index=" << obs[k].frame_index
                << " は姿勢範囲外。スキップ。\n";
      continue;
    }
    lm_obs[obs[k].lm_id].push_back(k);
  }
  std::cout << "  ランドマーク種類: " << lm_obs.size() << "\n";

  // --- 各ランドマークのパラメータ（世界座標）を初期化 ---
  // 初期値は世界座標観測の各軸メディアン（外れ値に頑健）
  std::map<std::string, std::array<double, 3>> lm_param;
  for (auto& [id, idxs] : lm_obs) {
    std::vector<double> xs, ys, zs;
    xs.reserve(idxs.size()); ys.reserve(idxs.size()); zs.reserve(idxs.size());
    for (int k : idxs) {
      xs.push_back(obs[k].world[0]);
      ys.push_back(obs[k].world[1]);
      zs.push_back(obs[k].world[2]);
    }
    std::array<double, 3> p{};
    axisMedian(xs, p[0]); axisMedian(ys, p[1]); axisMedian(zs, p[2]);
    lm_param[id] = p;
  }

  // --- 最適化前の RMS（世界座標, σ非正規化, 初期値基準）を記録 ---
  std::map<std::string, double> rms_before;
  for (auto& [id, idxs] : lm_obs) {
    const auto& p = lm_param[id];
    double se = 0.0;
    for (int k : idxs) {
      double dx = obs[k].world[0] - p[0];
      double dy = obs[k].world[1] - p[1];
      double dz = obs[k].world[2] - p[2];
      se += dx*dx + dy*dy + dz*dz;
    }
    rms_before[id] = std::sqrt(se / idxs.size());
  }

  // --- Ceres 問題の構築 ---
  ceres::Problem problem;
  for (auto& [id, idxs] : lm_obs) {
    double* p = lm_param[id].data();
    problem.AddParameterBlock(p, 3);
    for (int k : idxs) {
      const Observation& o = obs[k];
      double w = distanceWeight(o.distance);
      ceres::CostFunction* cost =
          LandmarkReprojectionResidual::Create(R_all[o.frame_index],
                                               poses[o.frame_index],
                                               o.sensor, w);
      // Huber 損失（残差は σ 単位に正規化済み）
      ceres::LossFunction* loss = new ceres::HuberLoss(Params::HUBER_DELTA);
      problem.AddResidualBlock(cost, loss, p);
    }
  }

  // --- 求解 ---
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;  // 各ランドマーク独立・小規模
  options.max_num_iterations = Params::MAX_ITERATIONS;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;

  std::cout << "\n[最適化中...]\n";
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  // --- 最適化後の統計（世界座標 RMS / インライア数）---
  // インライアは σ 単位の残差ノルムで判定
  std::map<std::string, double> rms_after;
  std::map<std::string, int>    inliers;
  for (auto& [id, idxs] : lm_obs) {
    const auto& p = lm_param[id];
    double se = 0.0;
    int    nin = 0;
    for (int k : idxs) {
      const Observation& o = obs[k];
      const Mat3& R = R_all[o.frame_index];
      const Pose6D& ps = poses[o.frame_index];
      // sensor_pred = R^T (p - t)
      double d0 = p[0] - ps.x, d1 = p[1] - ps.y, d2 = p[2] - ps.z;
      double sx = R[0]*d0 + R[3]*d1 + R[6]*d2;
      double sy = R[1]*d0 + R[4]*d1 + R[7]*d2;
      double sz = R[2]*d0 + R[5]*d1 + R[8]*d2;
      double ex = sx - o.sensor[0];
      double ey = sy - o.sensor[1];
      double ez = sz - o.sensor[2];
      // 世界座標 RMS（重みなしの幾何誤差。回転は直交なのでノルム不変）
      se += ex*ex + ey*ey + ez*ez;
      // σ 単位の残差ノルムでインライア判定
      double w = distanceWeight(o.distance);
      double rn = w * std::sqrt(ex*ex + ey*ey + ez*ez);
      if (rn <= Params::INLIER_SIGMA) nin++;
    }
    rms_after[id] = std::sqrt(se / idxs.size());
    inliers[id]   = nin;
  }

  // --- 出力 ---
  // uncertainty: 推定位置の標準誤差 ≒ rms_after / sqrt(num_inliers)。
  //   観測数が多いほど（= 平均化が効くほど）小さくなる「不確かさ」[m]。
  //   インライアが 0 の場合は定義できないため rms_after をそのまま用いる。
  std::ofstream csv(out_csv);
  if (!csv) { std::cerr << "ERROR: CSV を開けません: " << out_csv << "\n"; return 1; }
  csv << "lm_id,x,y,z,num_obs,num_inliers,rms_before,rms_after,uncertainty\n";
  csv << std::fixed << std::setprecision(6);

  std::cout << "\n[結果]\n";
  int written = 0, dropped = 0;
  for (auto& [id, idxs] : lm_obs) {
    const int n_obs = static_cast<int>(idxs.size());

    // 観測数による足切り（有効時のみ）
    if (cutoff_enabled && n_obs < min_obs) {
      dropped++;
      std::cout << "  " << id << "  [除外] obs=" << n_obs
                << " < min_obs=" << min_obs << "\n";
      continue;
    }

    const auto& p = lm_param[id];
    const int   nin = inliers[id];
    const double uncertainty =
        (nin > 0) ? rms_after[id] / std::sqrt(static_cast<double>(nin))
                  : rms_after[id];

    csv << id << ","
        << p[0] << "," << p[1] << "," << p[2] << ","
        << n_obs << "," << nin << ","
        << rms_before[id] << "," << rms_after[id] << ","
        << uncertainty << "\n";
    written++;

    std::cout << "  " << id
              << "  (" << std::fixed << std::setprecision(3)
              << p[0] << ", " << p[1] << ", " << p[2] << ")"
              << "  obs=" << n_obs
              << " inlier=" << nin
              << "  RMS " << rms_before[id] << " -> " << rms_after[id]
              << "  不確かさ=" << uncertainty << " [m]\n";
  }

  std::cout << "\n[完了] 出力: " << out_csv
            << "  (出力 " << written << " 件";
  if (cutoff_enabled) std::cout << " / 除外 " << dropped << " 件";
  std::cout << ")\n";
  return 0;
}

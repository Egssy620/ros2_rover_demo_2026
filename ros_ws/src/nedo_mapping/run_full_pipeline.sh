#!/bin/bash
# LIO-SAM マッピング + ランドマーク検出 一括パイプライン
#
# 処理フロー:
#   0. landmark_mapper_node をビルド（バイナリが古い/存在しない場合のみ）
#   1. LIO-SAM 起動
#   2. rosbag 再生（終了まで待機）
#   3. グラフ最適化の収束待ち
#   4. /lio_sam/save_map サービスでキーフレーム・姿勢を保存
#   5. LIO-SAM 停止
#   6. landmark_mapper_node でランドマーク検出 → 観測 CSV 出力
#   7. landmark_bundle_adjustment でバンドル調整 → ランドマーク地図 CSV 出力
#
# 使い方:
#   bash run_full_pipeline.sh [destination] [output_csv] [map_csv] \
#        [--circle-fit|--no-circle-fit] [--cutoff|--no-cutoff] [--min-obs N]
#
# 引数:
#   destination   地図の保存先（$HOME からの相対パス、LIO-SAM が内部で $HOME を補完）
#                 デフォルト: /Downloads/LOAM
#   output_csv    ランドマーク観測 CSV の保存先（絶対パス）
#                 デフォルト: $HOME/Downloads/landmarks.csv
#   map_csv       バンドル調整後のランドマーク地図 CSV の保存先（絶対パス）
#                 デフォルト: output_csv の末尾を _map.csv に置換
#   --circle-fit  RANSAC 円フィッティング ON（デフォルト）
#   --no-circle-fit  重心ベースで中心座標を決定
#   --cutoff / --no-cutoff  観測数によるランドマーク足切り（既定 OFF）
#   --min-obs N   足切り閾値を N に設定（指定すると足切り ON）
#
# 例:
#   bash run_full_pipeline.sh \
#       /Downloads/LOAM_run1 \
#       /root/Downloads/landmarks_run1.csv \
#       /root/Downloads/landmarks_run1_map.csv \
#       --min-obs 5

set -euo pipefail

# ── 引数パース ────────────────────────────────────────────────────────────────
# CIRCLE_FIT_FLAG : landmark_mapper_node 用（円フィット）
# BA_ARGS         : landmark_bundle_adjustment 用（足切り等）をそのまま転送
CIRCLE_FIT_FLAG="--circle-fit"
BA_ARGS=()
POSITIONAL=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --circle-fit|--no-circle-fit) CIRCLE_FIT_FLAG="$1"; shift ;;
        --cutoff|--no-cutoff)         BA_ARGS+=("$1"); shift ;;
        --min-obs)                    BA_ARGS+=("$1" "${2:?--min-obs には整数が必要}"); shift 2 ;;
        *)                            POSITIONAL+=("$1"); shift ;;
    esac
done

BAG_FOLDER="/NEDO_ROS/bags/bag_for_mapping"
DESTINATION="${POSITIONAL[0]:-/NEDO_ROS/bags/Output/LOAM}"
OUTPUT_CSV="${POSITIONAL[1]:-${HOME}/NEDO_ROS/bags/Output/landmarks.csv}"
# バンドル調整後のランドマーク地図（省略時は観測CSVと同じ場所に _map を付けて出力）
MAP_CSV="${POSITIONAL[2]:-${OUTPUT_CSV%.csv}_map.csv}"
LAUNCH_FILE="nedo_mapping mapping.launch.py"
SETTLE_WAIT=10

ACTUAL_BAG_FOLDER="${HOME}${BAG_FOLDER}"
ACTUAL_MAP_DIR="${HOME}${DESTINATION}"
KEYFRAMES_DIR="${ACTUAL_MAP_DIR}/keyframes"
TRANSFORMS_PCD="${ACTUAL_MAP_DIR}/transformations.pcd"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
LM_SRC="${SCRIPT_DIR}/src/landmark_mapper_node.cpp"
LM_BIN_INSTALLED="${WS_ROOT}/install/lib/nedo_mapping/landmark_mapper_node"
LM_BIN_LOCAL="${SCRIPT_DIR}/landmark_mapper_node"

# バンドル調整バイナリ（colcon インストール済みを優先、なければ g++ でその場ビルド）
BA_SRC="${SCRIPT_DIR}/src/landmark_bundle_adjustment.cpp"
BA_BIN_INSTALLED="${WS_ROOT}/install/lib/nedo_mapping/landmark_bundle_adjustment"
BA_BIN_LOCAL="${SCRIPT_DIR}/landmark_bundle_adjustment"

# ── バリデーション ─────────────────────────────────────────────────────────────
if [[ ! -d "$ACTUAL_BAG_FOLDER" ]]; then
    echo "Error: フォルダが見つかりません: $ACTUAL_BAG_FOLDER" >&2
    exit 1
fi

# bag_for_mapping フォルダ内の bag を自動検出（metadata.yaml を含むディレクトリ）
mapfile -t BAG_CANDIDATES < <(find "$ACTUAL_BAG_FOLDER" -maxdepth 2 -name "metadata.yaml" -exec dirname {} \; | sort)
if [[ ${#BAG_CANDIDATES[@]} -eq 0 ]]; then
    echo "Error: $ACTUAL_BAG_FOLDER 内に bag が見つかりません（metadata.yaml が存在するディレクトリなし）。" >&2
    exit 1
elif [[ ${#BAG_CANDIDATES[@]} -gt 1 ]]; then
    echo "Error: $ACTUAL_BAG_FOLDER 内に複数の bag が見つかりました。1つに絞ってください。" >&2
    for b in "${BAG_CANDIDATES[@]}"; do echo "  $b" >&2; done
    exit 1
fi
ACTUAL_BAG_PATH="${BAG_CANDIDATES[0]}"

if [[ ! -f "$LM_SRC" ]]; then
    echo "Error: ソースが見つかりません: $LM_SRC" >&2
    exit 1
fi
if [[ ! -f "$BA_BIN_INSTALLED" && ! -f "$BA_SRC" ]]; then
    echo "Error: ソースが見つかりません: $BA_SRC" >&2
    exit 1
fi

# ── 0. landmark_mapper_node バイナリ選択 ─────────────────────────────────────
# colcon ビルド済みバイナリを優先。なければ g++ でその場コンパイル。
echo "================================================"
echo " LIO-SAM + ランドマーク検出 パイプライン"
echo "  bag       : $ACTUAL_BAG_PATH"
echo "  地図保存先 : $ACTUAL_MAP_DIR"
echo "  観測CSV    : $OUTPUT_CSV"
echo "  地図CSV    : $MAP_CSV"
echo "  円フィット : $CIRCLE_FIT_FLAG"
echo "  BAオプション : ${BA_ARGS[*]:-(なし)}"
echo "================================================"
echo ""

if [[ -f "$LM_BIN_INSTALLED" ]]; then
    LM_BIN="$LM_BIN_INSTALLED"
    echo "[0/7] colcon インストール済みバイナリを使用: $LM_BIN"
else
    LM_BIN="$LM_BIN_LOCAL"
    if [[ ! -f "$LM_BIN" ]] || [[ "$LM_SRC" -nt "$LM_BIN" ]]; then
        echo "[0/7] landmark_mapper_node をビルド中（g++）..."
        g++ -O2 -std=c++17 "$LM_SRC" -o "$LM_BIN"
        echo "[0/7] ビルド完了: $LM_BIN"
    else
        echo "[0/7] landmark_mapper_node は最新です（ローカルビルド）。スキップ。"
    fi
fi

# landmark_bundle_adjustment バイナリ選択（Ceres リンクが必要）
if [[ -f "$BA_BIN_INSTALLED" ]]; then
    BA_BIN="$BA_BIN_INSTALLED"
    echo "[0/7] BA: colcon インストール済みバイナリを使用: $BA_BIN"
else
    BA_BIN="$BA_BIN_LOCAL"
    if [[ ! -f "$BA_BIN" ]] || [[ "$BA_SRC" -nt "$BA_BIN" ]]; then
        echo "[0/7] landmark_bundle_adjustment をビルド中（g++ + Ceres）..."
        g++ -O2 -std=c++17 "$BA_SRC" -o "$BA_BIN" -I/usr/include/eigen3 -lceres -lglog
        echo "[0/7] ビルド完了: $BA_BIN"
    else
        echo "[0/7] landmark_bundle_adjustment は最新です（ローカルビルド）。スキップ。"
    fi
fi

# ── クリーンアップ（スクリプト終了時に LIO-SAM を確実に停止）────────────────
LIO_SAM_PID=""

# ros2 launch の子ノードは launch 親が死んでも残ることがある。
# PGID kill の後に名前指定 pkill でゾンビ残存を確実に除去する。
lio_sam_kill() {
    local pid="$1"
    local pgid
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [[ -n "$pgid" ]] && [[ "$pgid" != "0" ]]; then
        kill -SIGINT -- "-$pgid" 2>/dev/null || true
    fi
    kill -SIGINT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true

    # launch が終了しても子ノードが残る場合があるため名前で追加終了
    local deadline=$(( $(date +%s) + 10 ))
    while pkill -0 -f "lio_sam_" 2>/dev/null || \
          pkill -0 -f "livox_converter" 2>/dev/null || \
          pkill -0 -f "odom_imu_fuser" 2>/dev/null || \
          pkill -0 -f "ekf_filter_node" 2>/dev/null; do
        if (( $(date +%s) >= deadline )); then
            echo "[cleanup] 残存プロセスを強制終了..."
            pkill -SIGKILL -f "lio_sam_"      2>/dev/null || true
            pkill -SIGKILL -f "livox_converter" 2>/dev/null || true
            pkill -SIGKILL -f "odom_imu_fuser"  2>/dev/null || true
            pkill -SIGKILL -f "ekf_filter_node" 2>/dev/null || true
            break
        fi
        pkill -SIGINT -f "lio_sam_"      2>/dev/null || true
        pkill -SIGINT -f "livox_converter" 2>/dev/null || true
        pkill -SIGINT -f "odom_imu_fuser"  2>/dev/null || true
        pkill -SIGINT -f "ekf_filter_node" 2>/dev/null || true
        sleep 1
    done
}
cleanup() {
    if [[ -n "$LIO_SAM_PID" ]] && kill -0 "$LIO_SAM_PID" 2>/dev/null; then
        echo "[cleanup] LIO-SAM を停止中 (PID=$LIO_SAM_PID)..."
        lio_sam_kill "$LIO_SAM_PID"
    else
        # LIO_SAM_PID が空でも念のため残存ノードを確認・終了
        pkill -SIGINT -f "lio_sam_"      2>/dev/null || true
        pkill -SIGINT -f "livox_converter" 2>/dev/null || true
        pkill -SIGINT -f "odom_imu_fuser"  2>/dev/null || true
        pkill -SIGINT -f "ekf_filter_node" 2>/dev/null || true
        sleep 2
        pkill -SIGKILL -f "lio_sam_"      2>/dev/null || true
        pkill -SIGKILL -f "livox_converter" 2>/dev/null || true
        pkill -SIGKILL -f "odom_imu_fuser"  2>/dev/null || true
        pkill -SIGKILL -f "ekf_filter_node" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# ── 1. LIO-SAM 起動（rviz2 なし：メモリ節約）────────────────────────────────
# setsid で別プロセスグループに隔離。
# ros2 bag play は終了時に同一グループへ SIGINT を送るため、
# 同グループにいると save_map の前に LIO-SAM が落ちてしまう。
echo "[1/7] LIO-SAM 起動中..."
setsid ros2 launch $LAUNCH_FILE use_rviz:=true &
LIO_SAM_PID=$!

echo "[1/7] /lio_sam/save_map サービス待機中..."
until ros2 service list 2>/dev/null | grep -q "/lio_sam/save_map"; do
    sleep 1
    if ! kill -0 "$LIO_SAM_PID" 2>/dev/null; then
        echo "Error: LIO-SAM が予期せず終了しました。" >&2
        exit 1
    fi
done

# /lio_sam/save_map が出現しても imuPreintegration の GTSAM 初期化はまだ続いている。
# imuPreintegration が最初の GTSAM 最適化を完了すると /odometry/imu_incremental を
# 発行し始めるので、そのトピックを待つことで「本当に準備完了」を確認する。
# これにより初期化未完了のまま IMU データが流れ込んで引き起こされる
# "Large velocity, reset IMU-preintegration!" を防ぐ。
echo "[1/7] imuPreintegration の初期化完了を待機中 (/odometry/imu_incremental)..."
IMU_READY_DEADLINE=$(( $(date +%s) + 60 ))
until ros2 topic list 2>/dev/null | grep -q "^/odometry/imu_incremental$"; do
    sleep 1
    if (( $(date +%s) >= IMU_READY_DEADLINE )); then
        echo "[1/7] 警告: 60s 待機してもimuPreintegrationが準備完了しませんでした。続行します。" >&2
        break
    fi
    if ! kill -0 "$LIO_SAM_PID" 2>/dev/null; then
        echo "Error: LIO-SAM が予期せず終了しました。" >&2
        exit 1
    fi
done
echo "[1/7] LIO-SAM 準備完了。"

# ── 2. rosbag 再生（終了まで待機）───────────────────────────────────────────
echo "[2/7] rosbag 再生中: $ACTUAL_BAG_PATH"
ros2 bag play "$ACTUAL_BAG_PATH"
echo "[2/7] rosbag 再生完了。"

# ── 3. グラフ最適化 収束待ち ─────────────────────────────────────────────────
echo "[3/7] グラフ最適化の収束待ち (${SETTLE_WAIT}s)..."
sleep "$SETTLE_WAIT"

# ── 4. 地図保存 ──────────────────────────────────────────────────────────────
echo "[4/7] 地図を保存中: $ACTUAL_MAP_DIR"
if ! ros2 service call /lio_sam/save_map \
        lio_sam/srv/SaveMap \
        "{resolution: 0.0, destination: '${DESTINATION}'}"; then
    echo "Error: save_map サービス呼び出しに失敗しました。" >&2
    exit 1
fi

# ── 5. LIO-SAM 停止 ──────────────────────────────────────────────────────────
echo "[5/7] LIO-SAM を停止中..."
lio_sam_kill "$LIO_SAM_PID"
LIO_SAM_PID=""
echo "[5/7] LIO-SAM 停止完了。"

# ── 6. ランドマーク検出 ───────────────────────────────────────────────────────
echo "[6/7] ランドマーク検出を実行中..."

if [[ ! -d "$KEYFRAMES_DIR" ]]; then
    echo "Error: キーフレームディレクトリが見つかりません: $KEYFRAMES_DIR" >&2
    exit 1
fi
if [[ ! -f "$TRANSFORMS_PCD" ]]; then
    echo "Error: transformations.pcd が見つかりません: $TRANSFORMS_PCD" >&2
    exit 1
fi

mkdir -p "$(dirname "$OUTPUT_CSV")"
"$LM_BIN" "$KEYFRAMES_DIR" "$TRANSFORMS_PCD" "$OUTPUT_CSV" "$CIRCLE_FIT_FLAG"

# ── 7. バンドル調整（ランドマーク地図の高精度化）─────────────────────────────
echo "[7/7] バンドル調整を実行中（姿勢固定 + Huber + 距離重み）..."
mkdir -p "$(dirname "$MAP_CSV")"
"$BA_BIN" "$OUTPUT_CSV" "$TRANSFORMS_PCD" "$MAP_CSV" ${BA_ARGS[@]+"${BA_ARGS[@]}"}

echo ""
echo "================================================"
echo " パイプライン完了！"
echo "  地図（PCD）: $ACTUAL_MAP_DIR"
echo "    GlobalMap.pcd"
echo "    transformations.pcd"
echo "    keyframes/raw_frame_N.pcd"
echo "  ランドマーク観測 CSV : $OUTPUT_CSV"
echo "  ランドマーク地図 CSV : $MAP_CSV  (バンドル調整後)"
echo "================================================"

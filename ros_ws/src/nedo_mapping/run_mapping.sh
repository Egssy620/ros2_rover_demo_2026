#!/bin/bash
# Run LIO-SAM mapping pipeline:
#   1. Launch LIO-SAM
#   2. Play a rosbag (blocking until finished)
#   3. Wait for graph optimization to settle
#   4. Call save_map service
#   5. Shut down LIO-SAM
#
# Usage:
#   bash run_mapping.sh <bag_path> [destination] [launch_file]
#
# Arguments:
#   bag_path      Path to the rosbag directory/file (required)
#   destination   Save path relative to $HOME (default: /Downloads/LOAM)
#                 NOTE: LIO-SAM prepends $HOME internally, so /Downloads/LOAM
#                       → $HOME/Downloads/LOAM = /root/Downloads/LOAM in Docker
#   launch_file   ROS2 launch file to use (default: lio_sam/run.launch.py)
#
# Example:
#   bash run_mapping.sh /data/bags/run1 /Downloads/LOAM

set -euo pipefail

# ── Arguments ────────────────────────────────────────────────────────────────
BAG_PATH="${1:-/NEDO_ROS/bags/20260527/rosbag2_2026_05_27-07_55_20}"
DESTINATION="${2:-/Downloads/LOAM}"
LAUNCH_FILE="${3:-nedo_mapping mapping.launch.py}"
SETTLE_WAIT=10          # seconds to wait after bag ends for iSAM to stabilize

ACTUAL_BAG_PATH="${HOME}${BAG_PATH}"
if [[ -z "$ACTUAL_BAG_PATH" ]]; then
    echo "Usage: $0 <bag_path> [destination] [launch_file]" >&2
    exit 1
fi

if [[ ! -e "$ACTUAL_BAG_PATH" ]]; then
    echo "Error: bag path not found: $ACTUAL_BAG_PATH" >&2
    exit 1
fi

# ── Cleanup on exit ───────────────────────────────────────────────────────────
LIO_SAM_PID=""
cleanup() {
    if [[ -n "$LIO_SAM_PID" ]] && kill -0 "$LIO_SAM_PID" 2>/dev/null; then
        echo "[cleanup] Killing LIO-SAM (PID=$LIO_SAM_PID)..."
        kill "$LIO_SAM_PID"
        wait "$LIO_SAM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# ── 1. Launch LIO-SAM ────────────────────────────────────────────────────────
echo "[1/4] Launching LIO-SAM: $LAUNCH_FILE"
ros2 launch $LAUNCH_FILE &
LIO_SAM_PID=$!

# Wait until the save_map service is available (= node is fully up)
echo "[1/4] Waiting for /lio_sam/save_map service..."
until ros2 service list 2>/dev/null | grep -q "/lio_sam/save_map"; do
    sleep 1
    if ! kill -0 "$LIO_SAM_PID" 2>/dev/null; then
        echo "Error: LIO-SAM exited unexpectedly." >&2
        exit 1
    fi
done
echo "[1/4] LIO-SAM is ready."

# ── 2. Play rosbag (blocks until finished) ───────────────────────────────────
echo "[2/4] Playing bag: $ACTUAL_BAG_PATH"
ros2 bag play "$ACTUAL_BAG_PATH"
echo "[2/4] Bag finished."

# ── 3. Wait for iSAM graph to settle ─────────────────────────────────────────
echo "[3/4] Waiting ${SETTLE_WAIT}s for graph optimization to settle..."
sleep "$SETTLE_WAIT"

# ── 4. Call save_map service ─────────────────────────────────────────────────
ACTUAL_PATH="${HOME}${DESTINATION}"
echo "[4/4] Saving map to: $ACTUAL_PATH"

if ! ros2 service call /lio_sam/save_map \
        lio_sam/srv/SaveMap \
        "{resolution: 0.0, destination: '${DESTINATION}'}"; then
    echo "Error: save_map service call failed." >&2
    exit 1
fi

echo ""
echo "Done. Map saved to: $ACTUAL_PATH"
echo "  GlobalMap.pcd"
echo "  trajectory.pcd / transformations.pcd"
echo "  keyframes/raw_frame_N.pcd"

#!/usr/bin/env bash
# ============================================================
# FILE:    start__drivers_for_tune_ekf.sh
# PURPOSE: Launch omni driver + IMU + EKF in one terminal
#          for covariance tuning. Ctrl+C stops all nodes.
#
# USAGE:
#   ./start__drivers_for_tune_ekf.sh [robot_namespace]
#   ./start__drivers_for_tune_ekf.sh ausra_1
# ============================================================

set -euo pipefail

# ── Colour helpers ────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'
CYN='\033[0;36m'; NC='\033[0m'
info() { echo -e "${CYN}[INFO ]${NC}  $*"; }
ok()   { echo -e "${GRN}[OK   ]${NC}  $*"; }
die()  { echo -e "${RED}[ERROR]${NC}  $*" >&2; exit 1; }

# ── Robot namespace ───────────────────────────────────────────
ROBOT_NS="${1:-ausra_1}"
info "Robot namespace: ${ROBOT_NS}"

# ── Source workspace ─────────────────────────────────────────
WS_SETUP="/home/ausranano/ausra_NM_ws/install/setup.bash"
[[ -f "$WS_SETUP" ]] || die "Workspace not found: ${WS_SETUP}. Run colcon build first."
# shellcheck disable=SC1090
set +u; source "$WS_SETUP"; set -u
ok "Workspace sourced."

# ── Generate hardware params (replace <robot_namespace>) ──────
HW_PARAMS_SRC="/home/ausranano/ausra_NM_ws/install/ausrabot_description/share/ausrabot_description/config/hardware_params.yaml"
HW_PARAMS_TMP="/tmp/${ROBOT_NS}_hw.yaml"
[[ -f "$HW_PARAMS_SRC" ]] || die "hardware_params.yaml not found: ${HW_PARAMS_SRC}"
sed "s/<robot_namespace>/${ROBOT_NS}/g" "$HW_PARAMS_SRC" > "$HW_PARAMS_TMP"
ok "Hardware params generated → ${HW_PARAMS_TMP}"

# ── Config paths ──────────────────────────────────────────────
EKF_PARAMS="/home/ausranano/ausra_NM_ws/install/ausra_localization/share/ausra_localization/config/ekf.yaml"
IMU_PARAMS="/home/ausranano/ausra_NM_ws/install/mpu6050driver/share/mpu6050driver/params/mpu6050.yaml"

[[ -f "$EKF_PARAMS" ]] || die "EKF params not found: ${EKF_PARAMS}"
[[ -f "$IMU_PARAMS"  ]] || die "IMU params not found: ${IMU_PARAMS}"

# ── Track PIDs for clean shutdown ─────────────────────────────
PIDS=()

cleanup() {
    echo ""
    info "Shutting down all nodes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGTERM "$pid" 2>/dev/null || true
        fi
    done
    # Wait up to 3s for all to exit
    sleep 1
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGKILL "$pid" 2>/dev/null || true
        fi
    done
    ok "All nodes stopped."
}
trap cleanup SIGINT SIGTERM EXIT

# ── 1. IMU Driver ─────────────────────────────────────────────
info "Starting IMU driver (mpu6050)..."
ros2 run mpu6050driver mpu6050driver \
    --ros-args \
    -r __ns:=/${ROBOT_NS} \
    -r __node:=mpu6050publisher \
    --params-file "$IMU_PARAMS" \
    -p frame_id:=${ROBOT_NS}_imu_link \
    &
PIDS+=($!)
ok "IMU driver started (PID ${PIDS[-1]})"
sleep 1

# ── 2. Omni Driver ────────────────────────────────────────────
info "Starting Omni driver..."
ros2 run omnidirectional_driver omni_driver \
    --ros-args \
    -r __ns:=/${ROBOT_NS} \
    -r __node:=omnidirectional_driver \
    --params-file "$HW_PARAMS_TMP" \
    -r /cmd_vel:=/${ROBOT_NS}/cmd_vel \
    -r /joint_states:=/${ROBOT_NS}/joint_states \
    -r /odom:=/${ROBOT_NS}/odom \
    -r /joint_group_velocity_controller/commands:=/${ROBOT_NS}/joint_group_velocity_controller/commands \
    -r /poseWithCovariance:=/${ROBOT_NS}/poseWithCovariance \
    -r /twistWithCovariance:=/${ROBOT_NS}/twistWithCovariance \
    &
PIDS+=($!)
ok "Omni driver started (PID ${PIDS[-1]})"
sleep 1

# ── 3. EKF Node ───────────────────────────────────────────────
info "Starting EKF node..."
ros2 run robot_localization ekf_node \
    --ros-args \
    -r __ns:=/${ROBOT_NS} \
    --params-file "$EKF_PARAMS" \
    -p odom_frame:=${ROBOT_NS}_odom \
    -p base_link_frame:=${ROBOT_NS}_robot_footprint \
    -p world_frame:=${ROBOT_NS}_odom \
    -p odom0:=odom \
    -p imu0:=imu \
    -r odometry/filtered:=filtered_odometry \
    &
PIDS+=($!)
ok "EKF node started (PID ${PIDS[-1]})"

echo ""
echo -e "${GRN}========================================${NC}"
echo -e "${GRN}  All 3 nodes running. Press Ctrl+C to stop all.${NC}"
echo -e "${GRN}========================================${NC}"
echo ""
echo "  Topics to monitor:"
echo "    ros2 topic echo /${ROBOT_NS}/odom --field pose.pose.position"
echo "    ros2 topic echo /${ROBOT_NS}/filtered_odometry --field pose.pose.position"
echo ""

# ── Keep alive until Ctrl+C ───────────────────────────────────
wait

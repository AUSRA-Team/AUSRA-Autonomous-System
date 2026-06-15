#!/usr/bin/env bash
# ============================================================
# FILE:    start_micro_ros.sh
# RUNS ON: Jetson (systemd — ausra-micro-ros.service)
# PURPOSE: Complete micro-ROS agent lifecycle:
#          1. Kill any stale micro-ROS agent processes
#          2. Launch the micro-ROS agent (which opens the serial
#             port and triggers ESP32 DTR reset automatically)
#          3. Wait for the agent to claim the serial port
#          4. Send the auto-detected namespace to the ESP32
#          5. Wait for XRCE-DDS session to be established
#          6. Signal systemd READY (Type=notify)
#          7. Forward the agent's exit code
#
# The namespace is read from /etc/ausra/namespace, which was
# written by ns_resolver.py during the previous boot stage.
# This script does NOT require any user interaction.
#
# BASED ON: start_micro_ros_agent.sh (the proven working script)
# ============================================================
set -eo pipefail

# ── Colour helpers ───────────────────────────────────────────
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${CYN}[MICRO-ROS]${NC}  $*"; }
ok()    { echo -e "${GRN}[MICRO-ROS]${NC}  $*"; }
warn()  { echo -e "${YLW}[MICRO-ROS]${NC}  $*"; }
die()   { echo -e "${RED}[MICRO-ROS]${NC}  $*" >&2; exit 1; }

# ── Configuration ────────────────────────────────────────────
SERIAL_DEV="${MICRO_ROS_DEV:-/dev/ttyACM0}"
BAUD_RATE="${MICRO_ROS_BAUD:-115200}"
NS_FILE="/etc/ausra/namespace"
AUSRA_WS="${AUSRA_WS:-/home/ausranano/ausra_NM_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SESSION_TIMEOUT="${SESSION_READY_TIMEOUT:-30}"
AGENT_EXTRA_ARGS="${AGENT_EXTRA_ARGS:-}"
PORT_READY_TIMEOUT="${PORT_READY_TIMEOUT:-10}"
PORT_POLL_INTERVAL="${PORT_POLL_INTERVAL:-0.2}"
POST_OPEN_DELAY="${POST_OPEN_DELAY:-1.0}"

# ── DDS environment ─────────────────────────────────────────
# Must match the hardware stack — see jetson_ros_env.sh for rationale.
# Do NOT set ROS_LOCALHOST_ONLY — micro_ros_agent uses FastDDS directly
# and needs WiFi interface discovery with CycloneDDS hardware nodes.
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>500</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"

# ── Source ROS 2 ─────────────────────────────────────────────
# ROS 2 setup.bash references $AMENT_TRACE_SETUP_FILES and $COLCON_TRACE
# without defaults, which trips bash's nounset (-u) flag. Disable it briefly.
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
else
    die "ROS 2 ${ROS_DISTRO} setup.bash not found at /opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f "${AUSRA_WS}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${AUSRA_WS}/install/setup.bash"
    set -u
    info "Sourced workspace overlay: ${AUSRA_WS}/install/setup.bash"
fi

# micro_ros_agent lives in a separate workspace
MICROROS_WS="${MICROROS_WS:-/home/ausranano/microros_ws}"
if [[ -f "${MICROROS_WS}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${MICROROS_WS}/install/setup.bash"
    set -u
    info "Sourced micro-ROS workspace: ${MICROROS_WS}/install/setup.bash"
fi

# ── Read namespace ───────────────────────────────────────────
if [[ ! -f "$NS_FILE" ]]; then
    die "Namespace file '$NS_FILE' not found. Is ausra-ns-resolver.service running?"
fi

ROBOT_NAMESPACE=$(cat "$NS_FILE")
if [[ -z "$ROBOT_NAMESPACE" ]]; then
    die "Namespace file is empty."
fi

info "Robot namespace: ${ROBOT_NAMESPACE}"
info "Serial device:   ${SERIAL_DEV} (baud: ${BAUD_RATE})"
info "DDS: domain=${ROS_DOMAIN_ID}  (no ROS_LOCALHOST_ONLY — FastDDS+CycloneDDS both on WiFi)"
echo ""

# ── Trap: clean shutdown ─────────────────────────────────────
AGENT_PID=""
AGENT_LOG=""

cleanup() {
    [[ -z "$AGENT_PID" ]] && return 0
    echo ""
    info "Shutting down micro-ROS agent (PID ${AGENT_PID})..."
    if kill -0 "$AGENT_PID" 2>/dev/null; then
        kill -SIGTERM "$AGENT_PID" 2>/dev/null || true
        wait "$AGENT_PID" 2>/dev/null || true
    fi
    ok "micro-ROS agent stopped."
    AGENT_PID=""

    # Kill any remaining agent processes
    if pkill -f "micro_ros_agent serial" 2>/dev/null; then
        ok "Remaining micro_ros_agent process(es) cleaned up."
    fi

    # Remove temp log
    if [[ -n "$AGENT_LOG" && -f "$AGENT_LOG" ]]; then
        rm -f "$AGENT_LOG"
    fi
}

trap cleanup SIGINT SIGTERM EXIT

# ── Step 1: Kill stale micro-ROS agents ──────────────────────
STALE_PIDS=$(pgrep -f "micro_ros_agent serial" 2>/dev/null || true)
if [[ -n "$STALE_PIDS" ]]; then
    info "Found stale micro_ros_agent: ${STALE_PIDS}"
    pkill -f "micro_ros_agent serial" 2>/dev/null || true
    sleep 0.5
    ok "Stale agents killed."
else
    info "No stale micro_ros_agent processes found."
fi
echo ""

# ── Step 2: Verify device ────────────────────────────────────
if [[ ! -e "$SERIAL_DEV" ]]; then
    die "Serial device '${SERIAL_DEV}' not found. Is the ESP32 plugged in?"
fi

# ── Step 3: Launch micro-ROS agent ───────────────────────────
# The agent opens the serial port, which toggles DTR and triggers
# an ESP32 hardware reset automatically. This is the correct order:
# agent starts → opens port → ESP32 resets → ESP32 waits for namespace.
info "Starting micro-ROS agent on ${SERIAL_DEV} @ ${BAUD_RATE} baud..."

AGENT_CMD=(
    ros2 run micro_ros_agent micro_ros_agent
    serial
    --dev "${SERIAL_DEV}"
    -b "${BAUD_RATE}"
)

if [[ -n "$AGENT_EXTRA_ARGS" ]]; then
    # shellcheck disable=SC2206
    AGENT_CMD+=( $AGENT_EXTRA_ARGS )
fi

AGENT_LOG=$(mktemp /tmp/micro_ros_agent_XXXXXX.log)
"${AGENT_CMD[@]}" > >(tee "${AGENT_LOG}") 2>&1 &
AGENT_PID=$!
ok "micro-ROS agent launched (PID ${AGENT_PID})"

# ── Step 4: Wait for agent to claim the serial port ──────────
# Poll until the agent actually holds the device open, so the
# namespace write doesn't race with port initialization.
info "Waiting for agent (PID ${AGENT_PID}) to open ${SERIAL_DEV}..."

PORT_READY=false
ELAPSED=0
while (( $(echo "$ELAPSED < $PORT_READY_TIMEOUT" | bc -l) )); do
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        die "micro-ROS agent (PID ${AGENT_PID}) exited prematurely."
    fi

    # fuser is more reliable than /proc/<pid>/fd because
    # micro_ros_agent opens the port from an internal thread.
    if fuser "${SERIAL_DEV}" >/dev/null 2>&1; then
        PORT_READY=true
        break
    fi

    sleep "${PORT_POLL_INTERVAL}"
    ELAPSED=$(echo "$ELAPSED + $PORT_POLL_INTERVAL" | bc)
done

if [[ "$PORT_READY" == "false" ]]; then
    warn "Timed out waiting for agent to open ${SERIAL_DEV} after ${PORT_READY_TIMEOUT}s."
    warn "Proceeding anyway — the namespace send may race."
else
    # The agent opening the port toggles DTR, which triggers the ESP32
    # hardware reset. Wait for the ESP32 to reboot before sending the namespace.
    info "Agent has opened ${SERIAL_DEV} — waiting ${POST_OPEN_DELAY}s for ESP32 reboot after DTR reset..."
    sleep "${POST_OPEN_DELAY}"
    ok "ESP32 should be ready."
fi

# ── Step 5: Reset ESP32-S3 via esptool ───────────────────────
# Apply a clean reset so the ESP32 boots fresh and enters namespace-read mode.
# '|| true' keeps the script running even if esptool reports a serial exception
# — the reset pulse is still issued.
info "Resetting ESP32-S3 via esptool.py..."
esptool.py --port "${SERIAL_DEV}" --chip esp32s3 --no-stub --before usb_reset run || true
ok "ESP32-S3 reset step done."
echo ""

# ── Step 6: Wait for ESP32 to boot ───────────────────────────
info "Waiting 3s for ESP32 to boot and enter namespace-read mode..."
sleep 3

# ── Step 7: Send namespace to ESP32 ──────────────────────────
# Write the namespace directly to the serial device — no stty baud change needed.
# The agent already has the port open at the configured baud rate.
info "Sending namespace 'ns:${ROBOT_NAMESPACE}' → ${SERIAL_DEV}..."
echo "ns:${ROBOT_NAMESPACE}" > "${SERIAL_DEV}"
ok "Namespace '${ROBOT_NAMESPACE}' sent to ESP32."
echo ""

# ── Step 8: Wait for XRCE-DDS session ────────────────────────
info "Waiting for ESP32 to establish XRCE-DDS session (timeout: ${SESSION_TIMEOUT}s)..."
ELAPSED=0
SESSION_OK=false

while (( $(echo "$ELAPSED < $SESSION_TIMEOUT" | bc -l) )); do
    if grep -q "session established" "$AGENT_LOG" 2>/dev/null; then
        SESSION_OK=true
        break
    fi
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        die "micro-ROS agent died while waiting for ESP32 session."
    fi
    sleep 0.5
    ELAPSED=$(echo "$ELAPSED + 0.5" | bc)
done

if [[ "$SESSION_OK" == "true" ]]; then
    ok "Session established! (${ELAPSED}s)"
    systemd-notify --ready --status="Session established for ${ROBOT_NAMESPACE}" 2>/dev/null || true
else
    warn "Timed out waiting for session after ${SESSION_TIMEOUT}s."
    warn "Agent is running — session may establish later."
    # Still signal ready so the chain continues (agent is running)
    systemd-notify --ready --status="Agent running for ${ROBOT_NAMESPACE} (no session yet)" 2>/dev/null || true
fi
echo ""

info "micro-ROS agent is running. Service will stay alive."

# ── Step 9: Forward agent exit code ──────────────────────────
wait "$AGENT_PID"
AGENT_EXIT=$?
AGENT_PID=""  # prevent cleanup from trying to kill again

if [[ $AGENT_EXIT -ne 0 ]]; then
    warn "micro-ROS agent exited with code ${AGENT_EXIT}."
fi

exit "$AGENT_EXIT"

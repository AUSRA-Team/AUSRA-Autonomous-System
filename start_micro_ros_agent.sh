#!/usr/bin/env bash
# ============================================================
# FILE:    start_micro_ros_agent.sh
# RUNS ON: Jetson / Host PC
# PURPOSE: Start the micro-ROS serial agent and send the robot
#          namespace string to the ESP32 over /dev/ttyACM0 so
#          the firmware can self-configure its ROS namespace.
#
# USAGE:
#   ./start_micro_ros_agent.sh <robot_namespace>
#
# EXAMPLE:
#   ./start_micro_ros_agent.sh ausra_1
#   ./start_micro_ros_agent.sh ausra_2
#
# The script will:
#   1. Validate arguments and device presence
#   2. Launch the micro-ROS agent in the background
#   3. Wait for the serial port to be opened by the agent
#   4. Write <robot_namespace>\n to /dev/ttyACM0
#   5. Keep running, forwarding SIGINT/SIGTERM to the agent
# ============================================================

set -euo pipefail

# ── Colour helpers ───────────────────────────────────────────
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
NC='\033[0m'   # No Color

info()  { echo -e "${CYN}[INFO ]${NC}  $*"; }
ok()    { echo -e "${GRN}[OK   ]${NC}  $*"; }
warn()  { echo -e "${YLW}[WARN ]${NC}  $*"; }
die()   { echo -e "${RED}[ERROR]${NC}  $*" >&2; exit 1; }

# ── Defaults ─────────────────────────────────────────────────
SERIAL_DEV="${MICRO_ROS_DEV:-/dev/ttyACM0}"
BAUD_RATE="${MICRO_ROS_BAUD:-115200}"
# Max seconds to wait for the agent to claim the port before writing
PORT_READY_TIMEOUT="${PORT_READY_TIMEOUT:-10}"
# Seconds between readiness poll attempts
PORT_POLL_INTERVAL="${PORT_POLL_INTERVAL:-0.2}"
# Seconds to wait after the agent opens the port before sending the namespace.
# When the agent opens /dev/ttyACM0 the DTR line toggles, which triggers the
# ESP32 hardware reset circuit.  The ESP32 needs ~1-3 s to reboot before it
# is ready to receive the namespace string on Serial.
POST_OPEN_DELAY="${POST_OPEN_DELAY:-1.0}"
# Extra optional args forwarded to the agent (e.g. -v6 for verbose)
AGENT_EXTRA_ARGS="${AGENT_EXTRA_ARGS:-}"
# Max seconds to wait for the ESP32 to connect and establish an XRCE-DDS session
SESSION_READY_TIMEOUT="${SESSION_READY_TIMEOUT:-30}"

# ── Argument validation ──────────────────────────────────────
if [[ $# -lt 1 ]]; then
    die "Usage: $0 <robot_namespace> [serial_device]
    robot_namespace  e.g. ausra_1 or ausra_2
    serial_device    (optional, default: /dev/ttyACM0 or \$MICRO_ROS_DEV)"
fi

ROBOT_NAMESPACE="$1"
# Allow overriding device as second positional arg
if [[ $# -ge 2 ]]; then
    SERIAL_DEV="$2"
fi

# Basic namespace sanity check (ROS names: alphanumeric + underscore, no leading /)
if [[ ! "$ROBOT_NAMESPACE" =~ ^[a-zA-Z][a-zA-Z0-9_]*$ ]]; then
    die "Invalid robot_namespace '${ROBOT_NAMESPACE}'.
    Must start with a letter and contain only [a-zA-Z0-9_]."
fi

# ── Device check ─────────────────────────────────────────────
if [[ ! -e "$SERIAL_DEV" ]]; then
    die "Serial device '${SERIAL_DEV}' not found.
    Make sure the ESP32 is plugged in and recognised by the OS."
fi

if [[ ! -w "$SERIAL_DEV" ]]; then
    warn "No write permission on '${SERIAL_DEV}'.  Trying with sudo..."
    NEED_SUDO=true
else
    NEED_SUDO=false
fi

# ── ROS 2 environment ────────────────────────────────────────
# Source the workspace if not already sourced
if ! command -v ros2 &>/dev/null; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    else
        die "ros2 not found in PATH and /opt/ros/humble/setup.bash does not exist.
    Please source your ROS 2 installation before running this script."
    fi
fi

# Optionally source the local workspace install overlay
WS_INSTALL="$(cd "$(dirname "$0")/../.." && pwd)/install/setup.bash"
if [[ -f "$WS_INSTALL" ]]; then
    # colcon's setup.bash references $COLCON_TRACE without a default, which
    # trips bash's nounset (-u) flag from set -euo pipefail. Disable it briefly.
    set +u
    # shellcheck disable=SC1090
    source "$WS_INSTALL"
    set -u
    info "Sourced workspace overlay: ${WS_INSTALL}"
fi

# ── Trap: clean shutdown ─────────────────────────────────────
AGENT_PID=""
AGENT_LOG=""  # temp file that captures agent output for session polling

cleanup() {
    # Guard against being called twice (SIGINT fires cleanup, then EXIT fires it again)
    [[ -z "$AGENT_PID" ]] && return 0
    echo ""
    info "Shutting down micro-ROS agent (PID ${AGENT_PID})..."
    if kill -0 "$AGENT_PID" 2>/dev/null; then
        kill -SIGTERM "$AGENT_PID" 2>/dev/null || true
        wait "$AGENT_PID" 2>/dev/null || true
    fi
    ok "micro-ROS agent stopped."
    AGENT_PID=""  # prevent double-run on EXIT after SIGINT

    # Kill any remaining micro_ros_agent processes (threads, child processes, etc.)
    # "micro_ros_agent serial" is used instead of the bare pattern to avoid matching
    # this script's own filename (start_micro_ros_agent.sh) during teardown.
    if pkill -f "micro_ros_agent serial" 2>/dev/null; then
        ok "All remaining micro_ros_agent process(es) killed."
    fi

    # Remove the temporary agent log file
    if [[ -n "$AGENT_LOG" && -f "$AGENT_LOG" ]]; then
        rm -f "$AGENT_LOG"
    fi
}

trap cleanup SIGINT SIGTERM EXIT

# ── Kill any stale micro-ROS agents ─────────────────────────
# Done here, before launching a new agent, so pkill cannot
# accidentally target the process we are about to start.
# Pattern "micro_ros_agent serial" matches only the ROS agent binary
# (always invoked with the 'serial' sub-command) and not this script
# whose filename also contains 'micro_ros_agent'.
STALE_PIDS=$(pgrep -f "micro_ros_agent serial" 2>/dev/null || true)
if [[ -n "$STALE_PIDS" ]]; then
    info "Found existing micro_ros_agent process(es): ${STALE_PIDS}"
    pkill -f "micro_ros_agent serial" 2>/dev/null || true
    sleep 0.5   # let the port fully release before esptool touches it
    ok "Stale micro_ros_agent process(es) killed."
else
    info "No existing micro_ros_agent processes found."
fi
echo ""


# ── Launch the micro-ROS agent ────────────────────────────────
info "Starting micro-ROS agent on ${SERIAL_DEV} @ ${BAUD_RATE} baud..."
info "Robot namespace will be: '${ROBOT_NAMESPACE}'"
echo ""

# Build the agent command
AGENT_CMD=(
    ros2 run micro_ros_agent micro_ros_agent
    serial
    --dev "${SERIAL_DEV}"
    -b "${BAUD_RATE}"
)
# Append any extra flags the caller passed in via env
if [[ -n "$AGENT_EXTRA_ARGS" ]]; then
    # Word-split intentional here
    # shellcheck disable=SC2206
    AGENT_CMD+=( $AGENT_EXTRA_ARGS )
fi

# Tee agent output to a temp log AND the terminal so we can detect
# the "establish_session" line without losing any console output.
AGENT_LOG=$(mktemp /tmp/micro_ros_agent_XXXXXX.log)
"${AGENT_CMD[@]}" > >(tee "${AGENT_LOG}") 2>&1 &
AGENT_PID=$!
ok "micro-ROS agent launched (PID ${AGENT_PID})"

# ── Wait for the agent to claim the serial port ──────────────
# Poll /proc/<pid>/fd until the agent actually holds the device open.
# This avoids a race where the namespace is written before the agent
# is in its XRCE-DDS listening state.
info "Waiting for agent (PID ${AGENT_PID}) to open ${SERIAL_DEV} ..."

PORT_READY=false
ELAPSED=0
while (( $(echo "$ELAPSED < $PORT_READY_TIMEOUT" | bc -l) )); do
    # Check the agent is still alive
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        die "micro-ROS agent (PID ${AGENT_PID}) exited prematurely.
    Check that micro_ros_agent is installed and that '${SERIAL_DEV}' is accessible."
    fi

    # Check whether ANY process (including agent's child threads) has the
    # serial device open.  fuser is more reliable than /proc/<pid>/fd
    # because micro_ros_agent opens the port from an internal thread.
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
    # hardware reset.  Wait for the ESP32 to reboot before sending the namespace.
    info "Agent has opened ${SERIAL_DEV} — waiting ${POST_OPEN_DELAY}s for ESP32 reboot after DTR reset..."
    sleep "${POST_OPEN_DELAY}"
    ok "ESP32 should be ready."
fi

# ── Reset ESP32-S3 ───────────────────────────────────────────
# Now that the namespace is saved, we reset the ESP32 so it boots up,
# reads the new namespace, connects to the agent, and starts XRCE-DDS.
# '|| true' keeps the script running even if esptool reports a
# serial exception — the reset pulse is still issued in that case.
info "Resetting ESP32-S3 via esptool.py (applying new namespace)..."
esptool.py --port "${SERIAL_DEV}" --chip esp32s3 --no-stub --before usb_reset run || true
ok "ESP32-S3 reset step done."
echo ""

# ── Wait for ESP32 to boot up ──────────────────────────────────
# Give the ESP32 a few seconds to finish booting after the esptool reset
# and enter the Serial reading loop.
info "Waiting 3 seconds for ESP32 to be ready to receive namespace..."
sleep 3

# ── Send the namespace to the ESP32 ──────────────────────────
info "Sending namespace '${ROBOT_NAMESPACE}' → ${SERIAL_DEV} ..."

_send_namespace() {
    if [[ "$NEED_SUDO" == "true" ]]; then
        echo "ns:${ROBOT_NAMESPACE}" | sudo tee "${SERIAL_DEV}" > /dev/null
    else
        echo "ns:${ROBOT_NAMESPACE}" > "${SERIAL_DEV}"
    fi
}

_send_namespace
ok "Namespace '${ROBOT_NAMESPACE}' sent to ESP32 successfully."
echo ""

# ── Wait for ESP32 to establish session ────────────────────────
info "Waiting for ESP32 to initialize and establish XRCE-DDS session..."
READY_TIMEOUT="${SESSION_READY_TIMEOUT:-30}"
ELAPSED=0
SUCCESS=false
while (( $(echo "$ELAPSED < $READY_TIMEOUT" | bc -l) )); do
    # 'session established' is printed by micro-ROS agent when XRCE-DDS connection is made
    if grep -q "session established" "$AGENT_LOG" 2>/dev/null; then
        SUCCESS=true
        break
    fi
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        die "micro-ROS agent died while waiting for ESP32 session."
    fi
    sleep 0.5
    ELAPSED=$(echo "$ELAPSED + 0.5" | bc)
done

if [[ "$SUCCESS" == "true" ]]; then
    ok "Session established!"
else
    warn "Timed out waiting for session established after ${READY_TIMEOUT}s."
fi
echo ""
info "micro-ROS agent is running.  Press Ctrl+C to stop."

# ── Keep script alive, forwarding the agent's exit code ───────
wait "$AGENT_PID"
AGENT_EXIT=$?

if [[ $AGENT_EXIT -ne 0 ]]; then
    warn "micro-ROS agent exited with code ${AGENT_EXIT}."
fi

exit "$AGENT_EXIT"

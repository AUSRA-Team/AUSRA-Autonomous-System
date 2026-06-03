#!/bin/bash
# ============================================================
# FILE: start_single_robot_test.sh
# RUNS ON: Laptop ONLY — no Jetsons required
# PURPOSE: Full end-to-end comms pipeline test on one machine.
#          Launches two fake robot publishers (ausra_1 + ausra_2)
#          and map_merge. Verifies that topics flow and
#          map_merge produces /map_merged — all without any
#          real hardware or network.
#
#          NOTE: The bridge is skipped in this mode because all
#          topics are local. The bridge is only needed for
#          cross-machine communication.
#
# USAGE: ./start_single_robot_test.sh
#
# PLACEHOLDERS: None — runs entirely on localhost
# ============================================================
set -e

# --- Source ROS2 Humble and workspace ---
source /opt/ros/humble/setup.bash
source ~/ausra_ws/install/setup.bash

# --- ROS2 environment ---
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "============================================"
echo "[Test] Single-machine end-to-end comms test"
echo "[Test] No hardware or network required"
echo "============================================"

# --- Launch fake ausra_1 publisher ---
echo "[Test] Starting fake ausra_1..."
ros2 run ausra_comms fake_robot_pub --ros-args \
    -p robot_name:=ausra_1 \
    -p robot_index:=1 \
    -p map_interval_sec:=10.0 &
FAKE1_PID=$!
sleep 1

# --- Launch fake ausra_2 publisher ---
echo "[Test] Starting fake ausra_2..."
ros2 run ausra_comms fake_robot_pub --ros-args \
    -p robot_name:=ausra_2 \
    -p robot_index:=2 \
    -p map_interval_sec:=10.0 &
FAKE2_PID=$!
sleep 1

# --- Launch map merge ---
echo "[Test] Starting map merge..."
ros2 launch ausra_comms map_merge.launch.py &
MERGE_PID=$!
sleep 3

echo "============================================"
echo "[Test] All components running:"
echo "  Fake ausra_1:   PID $FAKE1_PID"
echo "  Fake ausra_2:   PID $FAKE2_PID"
echo "  Map Merge:      PID $MERGE_PID"
echo "============================================"
echo ""
echo "[Test] Verifying topics..."
echo ""

# --- Verify topics exist ---
sleep 3
TOPICS=$(ros2 topic list 2>/dev/null)

PASS=true
for TOPIC in /ausra_1/heartbeat /ausra_1/pose /ausra_1/map \
             /ausra_2/heartbeat /ausra_2/pose /ausra_2/map; do
    if echo "$TOPICS" | grep -q "^${TOPIC}$"; then
        echo "[Test] ✓ $TOPIC — present"
    else
        echo "[Test] ✗ $TOPIC — MISSING"
        PASS=false
    fi
done

echo ""
if [ "$PASS" = true ]; then
    echo "[Test] ✓ All expected topics are present."
else
    echo "[Test] ✗ Some topics are missing — check logs above."
fi

# --- Wait for first maps to be published, then check /map_merged ---
echo ""
echo "[Test] Waiting for fake maps to be published (~10s)..."
sleep 12

TOPICS=$(ros2 topic list 2>/dev/null)
if echo "$TOPICS" | grep -q "^/map_merged$"; then
    echo "[Test] ✓ /map_merged — present (map merge is working!)"
else
    echo "[Test] ✗ /map_merged — not yet published"
    echo "[Test]   This may take a few more seconds. Check manually:"
    echo "[Test]   ros2 topic echo /map_merged --no-arr"
fi

echo ""
echo "[Test] System is running. Press Ctrl+C to stop."
echo ""
echo "  To verify manually in another terminal:"
echo "    source /opt/ros/humble/setup.bash && source ~/ausra_ws/install/setup.bash"
echo "    export ROS_DOMAIN_ID=0"
echo "    ros2 topic echo /ausra_1/heartbeat"
echo "    ros2 topic hz /ausra_1/pose"
echo "    ros2 topic echo /map_merged --no-arr"
echo ""

# --- Wait and clean up on Ctrl+C ---
cleanup() {
    echo ""
    echo "[Test] Shutting down..."
    kill $MERGE_PID $FAKE2_PID $FAKE1_PID 2>/dev/null
    wait 2>/dev/null
    echo "[Test] Done."
}
trap cleanup SIGINT SIGTERM

wait

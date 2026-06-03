#!/bin/bash
# ============================================================
# FILE: start_comms_2robots.sh
# RUNS ON: Jetson (ausra_1) — 2-robot mode
# PURPOSE: Starts relay_node to namespace SLAM topics as
#          /ausra_1/map, /ausra_1/pose, /ausra_1/heartbeat.
#          DDS carries them to the laptop automatically.
#
# USAGE: ./start_comms_2robots.sh
#
# PREREQUISITES:
#   - hardware_full_stack.launch.py already running (SLAM active)
#   - Both machines on the same WiFi network
#
# PLACEHOLDER: INSERT_LAPTOP_IP_HERE
# ============================================================
set -e

ROBOT_NAME=ausra_1

# --- Source ROS2 Humble and workspace ---
source /opt/ros/humble/setup.bash
source ~/swarm_ws/install/setup.bash

# --- ROS2 environment ---
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "============================================"
echo "[AUSRA] ${ROBOT_NAME} — 2-robot mode"
echo "[AUSRA] Checking laptop connectivity..."
echo "============================================"

# ┌──────────────────────────────────────────────┐
# │  REPLACE THIS with your laptop's WiFi IP     │
# │  Find it on laptop with: ip addr show wlan0  │
# └──────────────────────────────────────────────┘
LAPTOP_IP="192.168.1.34"

if [ "$LAPTOP_IP" = "INSERT_LAPTOP_IP_HERE" ]; then
    echo "[AUSRA] ⚠  You haven't set LAPTOP_IP yet!"
    echo "[AUSRA]    Edit this file: $(readlink -f $0)"
    echo "[AUSRA]    Replace INSERT_LAPTOP_IP_HERE with your laptop's WiFi IP"
    exit 1
fi

if ping -c 1 -W 2 "$LAPTOP_IP" > /dev/null 2>&1; then
    echo "[AUSRA] ✓ Laptop (${LAPTOP_IP}) — reachable"
else
    echo "[AUSRA] ✗ Laptop (${LAPTOP_IP}) — NOT reachable"
    echo "[AUSRA] Check WiFi connection. Both machines must be on same network."
    exit 1
fi

echo "============================================"
echo "[AUSRA] Starting relay_node (robot_name=${ROBOT_NAME})"
echo "[AUSRA] /map → /${ROBOT_NAME}/map"
echo "[AUSRA] /pose → /${ROBOT_NAME}/pose"
echo "[AUSRA] /${ROBOT_NAME}/heartbeat at 1 Hz"
echo "[AUSRA] DDS carries topics to laptop automatically"
echo "============================================"

ros2 launch ausra_comms robot_comms.launch.py robot_name:="$ROBOT_NAME"

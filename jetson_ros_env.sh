#!/usr/bin/env bash
# ============================================================
# FILE:    jetson_ros_env.bash
# PURPOSE: Source this file to configure the ROS 2 environment
#          for the AUSRA Jetson robot.  Add to ~/.bashrc:
#
#            source ~/ausra_NM_ws/src/AUSRA-Autonomous-System/jetson_ros_env.bash
#
# This file is kept in the repo so all Jetsons use identical
# settings without each developer touching their own ~/.bashrc.
# ============================================================

# ── Workspace overlay ────────────────────────────────────────
# Source the workspace if not already sourced
if [[ -z "${AUSRA_WS_SOURCED:-}" ]]; then
    _WS="${HOME}/ausra_NM_ws"
    if [[ -f "${_WS}/install/setup.bash" ]]; then
        # colcon setup.bash may reference $COLCON_TRACE; disable nounset briefly
        set +u
        # shellcheck disable=SC1090
        source "${_WS}/install/setup.bash"
        set -u
    fi
    export AUSRA_WS_SOURCED=1
fi

# ── DDS / Discovery settings ─────────────────────────────────
# ROS_DOMAIN_ID=0  — shared by all Jetson nodes and the Zenoh bridge.
# The laptop uses the same domain; cross-machine isolation is handled
# by the Zenoh bridge's allowlist, not by domain separation.
#
# CYCLONEDDS_URI raises MaxAutoParticipantIndex from the default of ~9
# to 500.  The full hardware stack (slam, nav2 ×6, ekf, lidar, imu,
# omni_driver, relay_node, zenoh…) easily exceeds 9 participants; without
# this, newly started nodes cannot register and become invisible.
#
# Do NOT set ROS_LOCALHOST_ONLY=1 — the micro_ros_agent binary uses
# FastDDS directly (bypassing the RMW layer) and ignores that variable,
# staying on the WiFi interface regardless.  CycloneDDS with
# ROS_LOCALHOST_ONLY=1 would then be on loopback while FastDDS is on
# WiFi, making them invisible to each other.
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>500</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

# ── ROS 2 daemon ─────────────────────────────────────────────
# The daemon is a shared background process — all terminals use the same one.
# Restarting it is harmless to running nodes (they are independent processes),
# but it does cause a brief gap in ros2 CLI commands across ALL terminals.
#
# Strategy: only restart if the daemon is NOT currently running.
# If it is running (started by a previous terminal with correct settings),
# leave it alone — it already has the right env and disrupting it would
# momentarily affect other open terminals.
if ! ros2 daemon status 2>/dev/null | grep -q "running"; then
    ros2 daemon start 2>/dev/null || true
fi
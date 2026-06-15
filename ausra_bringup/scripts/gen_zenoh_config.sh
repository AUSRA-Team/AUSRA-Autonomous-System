#!/bin/bash
# ============================================================
# FILE:    gen_zenoh_config.sh
# PURPOSE: Template the Zenoh bridge config by substituting
#          __ROBOT_NS__ with the actual namespace.
#
# The bridge uses peer mode with multicast scouting, so no
# router IP substitution is needed — peers auto-discover.
#
# Input:   ${AUSRA_WS}/src/AUSRA-Autonomous-System/ausra_bringup/zenoh/robot_bridge.json5
# Output:  /tmp/ausra_zenoh_bridge.json5
# Called by: ausra-zenoh-bridge.service as ExecStartPre=
# ============================================================
set -e

NS_FILE="/etc/ausra/namespace"
AUSRA_WS="${AUSRA_WS:-/home/ausranano/ausra_NM_ws}"

TEMPLATE="${AUSRA_WS}/src/AUSRA-Autonomous-System/ausra_bringup/zenoh/robot_bridge.json5"
OUTPUT="/tmp/ausra_zenoh_bridge.json5"

if [ ! -f "$NS_FILE" ]; then
    echo "[gen_zenoh_config] ERROR: $NS_FILE not found." >&2
    exit 1
fi

if [ ! -f "$TEMPLATE" ]; then
    echo "[gen_zenoh_config] ERROR: Template not found at $TEMPLATE" >&2
    exit 1
fi

NS=$(cat "$NS_FILE")

sed "s/__ROBOT_NS__/${NS}/g" "$TEMPLATE" > "$OUTPUT"

echo "[gen_zenoh_config] Generated $OUTPUT for namespace: $NS"

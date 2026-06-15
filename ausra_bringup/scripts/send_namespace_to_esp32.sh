#!/bin/bash
# ============================================================
# FILE:    send_namespace_to_esp32.sh
# PURPOSE: Standalone utility to send the robot namespace to the
#          ESP32-S3 via USB serial. Reads the namespace from
#          /etc/ausra/namespace (written by ns_resolver.py).
#
# This script can be called independently or as part of the
# start_micro_ros.sh boot sequence.
# ============================================================
set -e

NS_FILE="/etc/ausra/namespace"
SERIAL_DEV="${MICRO_ROS_DEV:-/dev/ttyACM0}"
BAUD=6000000
WAIT_AFTER_SEND=3  # seconds — ESP32 needs time to process the namespace

if [ ! -f "$NS_FILE" ]; then
    echo "[send_ns] ERROR: $NS_FILE not found. ns_resolver must run first." >&2
    exit 1
fi

NS=$(cat "$NS_FILE")

if [ -z "$NS" ]; then
    echo "[send_ns] ERROR: Namespace file is empty." >&2
    exit 1
fi

if [ ! -e "$SERIAL_DEV" ]; then
    echo "[send_ns] ERROR: Serial device '$SERIAL_DEV' not found." >&2
    echo "[send_ns]        Make sure the ESP32 is plugged in." >&2
    exit 1
fi

echo "[send_ns] Setting baud rate $BAUD on $SERIAL_DEV"
stty -F "$SERIAL_DEV" "$BAUD"

echo "[send_ns] Sending namespace 'ns:${NS}' to ESP32 on $SERIAL_DEV"
echo "ns:${NS}" > "$SERIAL_DEV"

echo "[send_ns] Waiting ${WAIT_AFTER_SEND}s for ESP32 to process..."
sleep "$WAIT_AFTER_SEND"
echo "[send_ns] Done."

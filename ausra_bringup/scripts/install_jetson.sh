#!/bin/bash
# ============================================================
# FILE:    install_jetson.sh
# RUNS ON: Jetson Orin Nano (one-time setup via SSH)
# PURPOSE: Set up the AUSRA autonomous system on a fresh Jetson.
#          After this runs, the Jetson boots autonomously and
#          never needs SSH again for normal operation.
#
# USAGE:
#   sudo bash install_jetson.sh [USER]
#
# EXAMPLE:
#   sudo bash install_jetson.sh ausranano
# ============================================================
set -e

# ── Parameters ───────────────────────────────────────────────
AUSRA_USER="${1:-ausranano}"
AUSRA_WS="/home/${AUSRA_USER}/ausra_NM_ws"
ROS_DISTRO="humble"
ZENOH_VER="1.2.1"
ZENOH_BRIDGE_BIN="/opt/zenoh-bridge/zenoh-bridge-ros2dds"

echo "╔══════════════════════════════════════════════════════╗"
echo "║           AUSRA Jetson Installer                    ║"
echo "║   User:      ${AUSRA_USER}                          ║"
echo "║   Workspace: ${AUSRA_WS}                            ║"
echo "║   Discovery: Zenoh multicast (auto, no router IP)   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── 1. Validate user exists ─────────────────────────────────
if ! id "$AUSRA_USER" &>/dev/null; then
    echo "[install] ERROR: User '$AUSRA_USER' does not exist." >&2
    echo "[install]        Create the user first or pass the correct username." >&2
    exit 1
fi

# Add to required groups (dialout for serial, video for GPU, i2c for IMU)
usermod -aG dialout,video,i2c,sudo "$AUSRA_USER" 2>/dev/null || true
echo "[install] ✓ User '$AUSRA_USER' added to dialout, video, i2c groups"

# ── 2. Create /etc/ausra directory ───────────────────────────
mkdir -p /etc/ausra
chown "${AUSRA_USER}:${AUSRA_USER}" /etc/ausra
chmod 755 /etc/ausra
echo "[install] ✓ Created /etc/ausra/ (owned by $AUSRA_USER)"

# ── 3. Write system configuration ───────────────────────────
# This env file is loaded by all systemd services for common config.
cat > /etc/ausra/ausra.env <<EOF
# AUSRA system configuration — written by install_jetson.sh
# Modify this file to change system-level settings.
AUSRA_WS=${AUSRA_WS}
MICROROS_WS=/home/${AUSRA_USER}/microros_ws
ZENOH_BRIDGE_BIN=${ZENOH_BRIDGE_BIN}
ROS_DISTRO=${ROS_DISTRO}
MICRO_ROS_DEV=/dev/ttyACM0
MICRO_ROS_BAUD=115200
SESSION_READY_TIMEOUT=30
# Optional: comma-separated explicit Zenoh peer endpoints (in addition to multicast)
# ZENOH_CONNECT_PEERS=tcp/192.168.1.6:7447,tcp/192.168.1.8:7447
EOF
chown "${AUSRA_USER}:${AUSRA_USER}" /etc/ausra/ausra.env
echo "[install] ✓ Wrote /etc/ausra/ausra.env"

# ── 4. Install Python dependencies ──────────────────────────
REQUIREMENTS="${AUSRA_WS}/src/AUSRA-Autonomous-System/ausra_bringup/requirements.txt"
if [ -f "$REQUIREMENTS" ]; then
    echo "[install] Installing Python dependencies from requirements.txt..."
    pip3 install --no-cache-dir -r "$REQUIREMENTS"
    echo "[install] ✓ Python dependencies installed"
else
    echo "[install] WARNING: requirements.txt not found at $REQUIREMENTS"
    echo "[install]          Installing dependencies manually..."
    pip3 install --no-cache-dir 'eclipse-zenoh>=1.0.0,<2.0.0' 'sdnotify>=0.3.2'
fi

# ── 5. Install Zenoh bridge (if not present) ────────────────
if [ ! -x "$ZENOH_BRIDGE_BIN" ]; then
    echo "[install] Zenoh bridge not found — installing v${ZENOH_VER}..."
    ARCH="$(uname -m)"
    TARBALL="zenoh-plugin-ros2dds-${ZENOH_VER}-${ARCH}-unknown-linux-gnu-standalone.zip"
    URL="https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VER}/${TARBALL}"

    cd /tmp
    curl -LO "${URL}"
    mkdir -p /opt/zenoh-bridge
    unzip -o "${TARBALL}" -d /opt/zenoh-bridge
    chmod +x "${ZENOH_BRIDGE_BIN}"
    rm -f "/tmp/${TARBALL}"

    echo "[install] ✓ Zenoh bridge installed: $(${ZENOH_BRIDGE_BIN} --version 2>&1 || true)"
else
    echo "[install] ✓ Zenoh bridge already installed: $(${ZENOH_BRIDGE_BIN} --version 2>&1 || true)"
fi

# ── 6. Build the ROS workspace ──────────────────────────────
echo "[install] Building AUSRA workspace..."
cd "$AUSRA_WS"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install \
    --packages-select ausra_msgs ausra_supervisor ausra_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "[install] ✓ Workspace built"

# ── 7. Make scripts executable ───────────────────────────────
BRINGUP_SRC="${AUSRA_WS}/src/AUSRA-Autonomous-System/ausra_bringup"
chmod +x "${BRINGUP_SRC}/scripts/"*.sh
chmod +x "${BRINGUP_SRC}/scripts/"*.py
echo "[install] ✓ Scripts marked executable"

# ── 8. Install systemd units ────────────────────────────────
cp "${BRINGUP_SRC}/systemd/"*.service /etc/systemd/system/
systemctl daemon-reload

for svc in ausra-ns-resolver ausra-micro-ros ausra-ros-stack ausra-zenoh-bridge ausra-supervisor; do
    systemctl enable "$svc"
    echo "[install]   Enabled: $svc"
done
echo "[install] ✓ All systemd units installed and enabled"

# ── 9. Install esptool (if not present) ──────────────────────
if ! command -v esptool.py &>/dev/null; then
    echo "[install] Installing esptool..."
    pip3 install --no-cache-dir esptool
    echo "[install] ✓ esptool installed"
else
    echo "[install] ✓ esptool already installed"
fi

# ── 10. Verify critical binaries ─────────────────────────────
echo ""
echo "[install] Verification:"
FAIL=0
for cmd in ros2 esptool.py python3; do
    if command -v "$cmd" &>/dev/null; then
        echo "  ✓ $cmd found"
    else
        echo "  ✗ $cmd NOT FOUND" >&2
        FAIL=1
    fi
done

if [ -x "$ZENOH_BRIDGE_BIN" ]; then
    echo "  ✓ zenoh-bridge-ros2dds found"
else
    echo "  ✗ zenoh-bridge-ros2dds NOT FOUND at $ZENOH_BRIDGE_BIN" >&2
    FAIL=1
fi

if [ -e "/dev/ttyACM0" ]; then
    echo "  ✓ /dev/ttyACM0 present"
else
    echo "  ⚠ /dev/ttyACM0 not present (ESP32 not plugged in?)"
fi

echo ""
if [ $FAIL -eq 0 ]; then
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║   Installation complete!                            ║"
    echo "║                                                     ║"
    echo "║   Reboot to start the AUSRA stack:                  ║"
    echo "║     sudo reboot                                     ║"
    echo "║                                                     ║"
    echo "║   Monitor boot sequence:                            ║"
    echo "║     journalctl -u ausra-ns-resolver -f              ║"
    echo "║     journalctl -u ausra-micro-ros -f                ║"
    echo "║     journalctl -u ausra-ros-stack -f                ║"
    echo "║                                                     ║"
    echo "║   Configuration: /etc/ausra/ausra.env               ║"
    echo "╚══════════════════════════════════════════════════════╝"
else
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║   Installation finished with WARNINGS.              ║"
    echo "║   Review the issues above before rebooting.         ║"
    echo "╚══════════════════════════════════════════════════════╝"
fi

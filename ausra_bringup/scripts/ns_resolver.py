#!/usr/bin/env python3
# ============================================================
# FILE:    ns_resolver.py
# RUNS ON: Jetson (systemd, before any ROS process)
# PURPOSE: Dynamically negotiate a unique robot namespace
#          (ausra_1 … ausra_N) via Zenoh, then write it to
#          /etc/ausra/namespace for downstream services.
#
# DISCOVERY: Uses Zenoh PEER mode with multicast scouting —
#            no hardcoded router IP required. All Zenoh peers
#            on the same LAN discover each other automatically
#            via multicast (224.0.0.224:7446).
#
# IMPORTANT: This script must NOT import rclpy or any ROS
#            package — ROS is not started yet when this runs.
# ============================================================

import json
import os
import random
import subprocess
import sys
import time
import uuid

import sdnotify
import zenoh

# ── Configuration ────────────────────────────────────────────
NS_FILE      = "/etc/ausra/namespace"
NS_ENV_FILE  = "/etc/ausra/namespace.env"

HEARTBEAT_KEY    = "ausra/heartbeat/{ns}"
CLAIM_KEY        = "ausra/ns-claim/{ns}"
LISTEN_WINDOW    = 2.5    # seconds to collect existing heartbeats
COLLISION_WINDOW = 0.6    # seconds to watch for a collision after publishing claim
MAX_N            = 20     # maximum number of robots in the swarm

# Zenoh scouting — multicast auto-discovery on the LAN
MULTICAST_ADDR   = "224.0.0.224:7446"
ZENOH_LISTEN_PORT = "7448"   # ephemeral port for ns_resolver (7447 is for the bridge)

# Optional: explicit peer endpoints to connect to in addition to multicast.
# Read from env var as comma-separated list, e.g. "tcp/192.168.1.6:7447,tcp/192.168.1.8:7447"
_extra_peers = os.environ.get("ZENOH_CONNECT_PEERS", "")


# ── Helpers ──────────────────────────────────────────────────

def wait_for_network_interface(timeout_sec: int = 60) -> bool:
    """Block until a non-loopback network interface has an IP address."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            # Check if we have any non-loopback interface with an IP
            result = subprocess.run(
                ["ip", "-4", "route", "show", "default"],
                capture_output=True, text=True, timeout=5,
            )
            if result.stdout.strip():
                return True  # We have a default route → network is up
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        time.sleep(1.0)
    return False


def _fallback_and_exit(notifier: sdnotify.SystemdNotifier) -> None:
    """Write fallback namespace and signal ready so the systemd chain continues."""
    _write_namespace("ausra_fallback")
    notifier.notify("STATUS=Fallback: network unavailable")
    notifier.notify("READY=1")
    sys.exit(0)


def _write_namespace(ns: str) -> None:
    """Persist the namespace to disk for downstream services."""
    # Ensure directory exists (should already exist from install)
    os.makedirs(os.path.dirname(NS_FILE), exist_ok=True)

    with open(NS_FILE, "w") as f:
        f.write(ns)

    with open(NS_ENV_FILE, "w") as f:
        f.write(f"AUSRA_ROBOT_NAME={ns}\n")

    print(f"[ns_resolver] Wrote namespace '{ns}' → {NS_FILE}, {NS_ENV_FILE}")


def get_mac_tail() -> str:
    """Return the last 4 hex chars of the machine's MAC address (tiebreaker)."""
    mac = uuid.getnode()
    return format(mac, "012x")[-4:]


# ── Main ─────────────────────────────────────────────────────

def main() -> None:
    notifier = sdnotify.SystemdNotifier()
    notifier.notify("STATUS=Waiting for network interface...")

    # 1. Wait for a network interface to come up (Wi-Fi connects)
    if not wait_for_network_interface(timeout_sec=60):
        print("[ns_resolver] ERROR: No network interface available — using fallback namespace")
        _fallback_and_exit(notifier)

    notifier.notify("STATUS=Network up, opening Zenoh peer session...")
    print("[ns_resolver] Network interface is up.")

    # Brief pause for multicast routing to stabilize after interface comes up
    time.sleep(1.0)

    # 2. Open Zenoh session in PEER mode with multicast scouting
    #    No router IP needed — peers discover each other via multicast.
    config = zenoh.Config()
    config.insert_json5("mode", '"peer"')
    config.insert_json5("listen/endpoints", json.dumps([f"tcp/0.0.0.0:{ZENOH_LISTEN_PORT}"]))
    config.insert_json5("scouting/multicast/enabled", "true")
    config.insert_json5("scouting/multicast/address", f'"{MULTICAST_ADDR}"')
    config.insert_json5("scouting/multicast/interface", '"auto"')
    config.insert_json5("scouting/gossip/enabled", "true")

    # If explicit peer endpoints were provided, connect to them too
    if _extra_peers:
        peers = [p.strip() for p in _extra_peers.split(",") if p.strip()]
        config.insert_json5("connect/endpoints", json.dumps(peers))
        print(f"[ns_resolver] Also connecting to explicit peers: {peers}")

    session = zenoh.open(config)
    print("[ns_resolver] Zenoh peer session opened (multicast scouting active).")

    # 3. Listen for existing heartbeats to find which namespaces are taken
    notifier.notify("STATUS=Scanning heartbeats...")
    seen: set = set()

    def _on_heartbeat(sample):
        # Extract last path segment: "ausra/heartbeat/ausra_2" → "ausra_2"
        key = str(sample.key_expr)
        ns_name = key.rsplit("/", 1)[-1]
        seen.add(ns_name)

    sub_hb = session.declare_subscriber("ausra/heartbeat/*", _on_heartbeat)
    time.sleep(LISTEN_WINDOW)
    sub_hb.undeclare()

    print(f"[ns_resolver] Heartbeats seen: {seen if seen else '(none)'}")

    # 4. Find lowest available candidate
    candidate = None
    for n in range(1, MAX_N + 1):
        name = f"ausra_{n}"
        if name not in seen:
            candidate = name
            break

    if candidate is None:
        print(f"[ns_resolver] ERROR: All namespace slots 1..{MAX_N} are taken!")
        _fallback_and_exit(notifier)

    print(f"[ns_resolver] Claiming candidate: {candidate}")

    # 5. Collision detection — publish claim and watch for competing claims
    collision = False
    my_mac = get_mac_tail()

    def _on_claim(sample):
        nonlocal collision
        try:
            payload = bytes(sample.payload).decode("utf-8")
            data = json.loads(payload)
            if data.get("mac") != my_mac:
                collision = True
                print(f"[ns_resolver] COLLISION detected from MAC {data.get('mac')}")
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass

    claim_key = f"ausra/ns-claim/{candidate}"
    sub_claim = session.declare_subscriber(claim_key, _on_claim)

    # Publish our claim
    claim_payload = json.dumps({"claiming": candidate, "mac": my_mac})
    session.put(claim_key, claim_payload)

    time.sleep(COLLISION_WINDOW)
    sub_claim.undeclare()
    session.close()

    # 6. Handle collision — retry with jitter
    if collision:
        jitter = 0.5 + random.uniform(0, 0.3)
        print(f"[ns_resolver] Collision — retrying in {jitter:.2f}s...")
        time.sleep(jitter)
        main()  # Recursive retry — re-opens fresh session and re-observes heartbeats
        return

    # 7. Success — write namespace and signal systemd
    _write_namespace(candidate)
    notifier.notify(f"STATUS=Namespace: {candidate}")
    notifier.notify("READY=1")
    print(f"[ns_resolver] ✓ Namespace '{candidate}' claimed successfully.")


if __name__ == "__main__":
    main()

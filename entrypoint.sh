#!/bin/bash
# ==============================================================================
# WEDA ROS VLM — Container Entrypoint
# ------------------------------------------------------------------------------
# Startup sequence:
#   1. Source the ROS 2 underlay (fails loudly if the install is broken).
#   2. Auto-build the colcon workspace if packages exist but haven't been built
#      (self-heals a workspace built under an older image layout).
#   3. Source the user workspace overlay (never fatal).
#   4. Run the component license gate (installs WISE_LICENSED_* packages only
#      after EULA acceptance — commercial bits are never baked into the image).
#   5. Ultralytics BYOL gate for the detector tier (EULA-gated, never baked).
#   6. Hydrate bundle runtime pieces (VLM client, camera SDK, display tools).
#   7. Install debug tooling (strace) — only when WEDA_DEBUG_TOOLS=true.
#   8. Print the environment summary, signal readiness (/tmp/.ready), and
#      execute the container command.
#
# To customize startup (e.g., auto-launch the pipeline), edit the
# USER STARTUP section below.
# ==============================================================================
set -eo pipefail

# --- 1. ROS underlay ----------------------------------------------------------
if [ -n "${ROS_DISTRO:-}" ]; then
    underlay="/opt/ros/${ROS_DISTRO}/setup.bash"
    if [ -f "$underlay" ]; then
        # shellcheck disable=SC1090
        source "$underlay"
    else
        echo "[entrypoint] FATAL: ROS_DISTRO='${ROS_DISTRO}' but ${underlay} is missing." >&2
        exit 1
    fi
else
    echo "[entrypoint] note: ROS_DISTRO unset — running without a ROS underlay." >&2
fi

# --- 2. Auto-build colcon workspace (first-run convenience) --------------------
WEDA_WS="/opt/weda_ws"
if [ -d "${WEDA_WS}/src" ] && [ -n "$(ls -A "${WEDA_WS}/src" 2>/dev/null)" ]; then
    if [ ! -d "${WEDA_WS}/install" ]; then
        echo "[entrypoint] Workspace packages found but not built — running colcon build..."
        cd "${WEDA_WS}"
        if ! colcon build --symlink-install 2>&1 | tail -5; then
            # Stale state from a previous image generation (e.g. different
            # python layout) manifests as setup.py/--uninstall errors:
            # wipe and retry once.
            echo "[entrypoint] build failed — cleaning stale build state and retrying..."
            rm -rf "${WEDA_WS}/build" "${WEDA_WS}/install" "${WEDA_WS}/log"
            colcon build --symlink-install 2>&1 | tail -5 \
                || echo "[entrypoint] WARN: colcon build failed; fix manually in the container." >&2
        fi
        echo "[entrypoint] colcon build complete."
    fi
fi
cd "${WEDA_WS}" 2>/dev/null || true

# --- 3. User workspace overlay (optional; never fatal) -------------------------
overlay="${WEDA_WS}/install/setup.bash"
if [ -f "$overlay" ]; then
    # shellcheck disable=SC1090
    if source "$overlay"; then
        echo "[entrypoint] user workspace overlay sourced"
    else
        echo "[entrypoint] WARNING: overlay ${overlay} failed to source; continuing without it." >&2
    fi
fi

# --- 4. Component license gate (accept/decline) --------------------------------
# Installs WISE_LICENSED_APT / WISE_LICENSED_PIP at runtime, only when
# ACCEPT_COMMERCIAL_EULA=true (or an interactive TTY acceptance). Both vars
# flow from .env via docker-compose.
if [ -x /usr/local/bin/hydrate_licensed.sh ]; then
    /usr/local/bin/hydrate_licensed.sh || \
        echo "[entrypoint] WARNING: license gate failed; continuing without gated components." >&2
fi

# --- 5. Ultralytics BYOL gate (detector tier) ----------------------------------
# Commercial package: installed at container start ONLY after EULA acceptance,
# never baked into the image. --no-deps is REQUIRED: ultralytics' dependency
# list would pull a CPU opencv wheel over the CUDA build (cv2 4.13 ships
# without pip metadata, so pip cannot see it is already satisfied).
if [ "${ACCEPT_COMMERCIAL_EULA:-}" = "true" ]; then
    if ! python3 -c "import ultralytics" &>/dev/null; then
        echo "[entrypoint] EULA accepted — installing Ultralytics detector (BYOL)..."
        uv pip install --system --no-cache-dir --no-deps \
            "ultralytics==8.4.24" "ultralytics-thop==2.0.18" \
            && echo "[entrypoint] Ultralytics installed." \
            || echo "[entrypoint] WARN: Ultralytics install failed (offline?) — detector tier disabled."
    fi
    # Keep it from self-updating dependencies at import time
    python3 - <<'PYPATCH' 2>/dev/null || true
try:
    import inspect, ultralytics.utils.checks as c
    path = inspect.getfile(c)
    src = open(path).read()
    if "WEDA Protection" not in src:
        with open(path, "a") as f:
            f.write("\n# WEDA Protection: never auto-install/upgrade deps\n")
            f.write("check_requirements = lambda *a, **k: True\n")
except Exception:
    pass
PYPATCH
else
    echo "[entrypoint] Ultralytics detector tier: skipped (ACCEPT_COMMERCIAL_EULA != true)."
fi

# --- 6. VLM overlay hydration (shared base image -> this bundle) ----------------
# The bundle runs on the shared WEDA ROS-CV base image; these OSS pieces are
# bundle-specific and hydrate once per container creation. Pinned for
# reproducibility; guarded non-fatal so offline starts still come up.
if ! python3 -c "import openai" &>/dev/null; then
    echo "[entrypoint] hydrating VLM client (openai + deps)..."
    uv pip install --system --no-cache-dir "openai==2.38.0" \
        && echo "[entrypoint] openai ready." \
        || echo "[entrypoint] WARN: openai install failed (offline?) — VLM tier unavailable until a start with network." >&2
fi
if ! python3 -c "import pyorbbecsdk" &>/dev/null; then
    echo "[entrypoint] hydrating Orbbec SDK (pyorbbecsdk2)..."
    uv pip install --system --no-cache-dir --no-deps "pyorbbecsdk2==2.1.1" \
        && echo "[entrypoint] pyorbbecsdk ready." \
        || echo "[entrypoint] WARN: pyorbbecsdk install failed (offline?) — depth camera unavailable until a start with network." >&2
fi
if ! command -v rviz2 >/dev/null 2>&1; then
    echo "[entrypoint] hydrating display tools (rviz2 + image-transport-plugins)..."
    (apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        ros-${ROS_DISTRO:-jazzy}-rviz2 ros-${ROS_DISTRO:-jazzy}-image-transport-plugins) \
        >/dev/null 2>&1 \
        && echo "[entrypoint] display tools ready." \
        || echo "[entrypoint] WARN: display tools install failed (offline?) — demo GUIs unavailable until a start with network." >&2
fi

# --- 7. Debug tooling (opt-in) --------------------------------------------------
# Syscall tracer for in-container debugging. Off unless asked for: it is only
# useful with CAP_SYS_PTRACE, and shipping a tracer into every deployed
# container is attack surface nobody asked for. Set WEDA_DEBUG_TOOLS=true to
# install it -- compose forwards it, so setting it in .env is enough.
# Idempotent; never fatal when offline.
if [ "${WEDA_DEBUG_TOOLS:-false}" = "true" ] && ! command -v strace >/dev/null 2>&1; then
    echo "[entrypoint] installing strace (WEDA_DEBUG_TOOLS=true)..."
    (apt-get update -qq && apt-get install -y -qq --no-install-recommends strace) \
        >/dev/null 2>&1 \
        && echo "[entrypoint] strace installed." \
        || echo "[entrypoint] WARN: strace install failed (offline?) — continuing without it."
fi

# --- 8. Environment summary -----------------------------------------------------
echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO:-<unset>} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "[entrypoint] DISPLAY=${DISPLAY:-<unset>} XAUTHORITY=${XAUTHORITY:-<unset>}"
python3 - 2>/dev/null <<'PYEOF' || true
versions = []
for mod, label in (("torch", "torch"), ("cv2", "opencv"), ("openai", "openai"),
                   ("pyorbbecsdk", "pyorbbecsdk"), ("rclpy", "rclpy")):
    try:
        m = __import__(mod)
        versions.append(f"{label}={getattr(m, '__version__', 'ok')}")
    except Exception:
        versions.append(f"{label}=MISSING")
print("[entrypoint] VLM stack:", " | ".join(versions))
PYEOF

# ==============================================================================
# USER STARTUP SECTION
# ------------------------------------------------------------------------------
# Add commands to run every time the container starts. Examples:
#
#   # Auto-launch the full perception pipeline:
#   # ros2 launch vlm_perception perception.launch.py &
# ==============================================================================

# Signal ready (for build.sh hydration wait)
touch /tmp/.ready

echo "[entrypoint] Environment ready."
echo "----------------------------------------------------------------------"

exec "$@"

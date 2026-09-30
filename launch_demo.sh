#!/bin/bash
# ==============================================================================
# WEDA ROS VLM — Perception Demo Launcher (run INSIDE the container)
# ------------------------------------------------------------------------------
# Starts the full pipeline (Orbbec driver + depth context + VLM perception),
# opens rviz2 + rqt_graph when a display is available, and streams the live
# perception payloads to the terminal. Press q or Esc to stop everything
# cleanly and release the camera (Ctrl-C works too).
#
#   /opt/weda-release/launch_demo.sh              # or: ./build.sh --demo (host)
# ==============================================================================
# no -u: ROS setup.bash references unset ament vars (breaks under nounset)
set -o pipefail

CONFIG="${VLM_APP_CONFIG:-/opt/weda-release/config/vlm_app.yaml}"
RVIZ_PRESET="/opt/weda-release/display/vlm_perception.rviz"

cyan()  { echo -e "\e[1;36m$1\e[0m"; }
warn()  { echo -e "\e[1;33m[WARN]\e[0m  $1"; }
ok()    { echo -e "\e[1;32m[ OK ]\e[0m  $1"; }
fail()  { echo -e "\e[1;31m[FAIL]\e[0m  $1" >&2; exit 1; }

cyan "======================================================================"
cyan "       WEDA ROS VLM — Live Perception Demo (press q or Esc to stop)   "
cyan "======================================================================"

# --- Environment ---------------------------------------------------------------
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" || fail "ROS underlay missing"

# Auto-rebuild when source is newer than the last build (the workspace is
# bind-mounted from the host, so edits made there land here instantly — but
# this colcon setup COPIES python modules on install, so a rebuild is needed
# for changes to take effect).
STAMP=/opt/weda_ws/install/setup.bash
if [ ! -f "${STAMP}" ] || \
   [ -n "$(find /opt/weda_ws/src \( -name '*.py' -o -name '*.xml' -o -name '*.cfg' \) \
            -newer "${STAMP}" -print -quit 2>/dev/null)" ]; then
    cyan "Workspace source changed — rebuilding (colcon build)..."
    ( cd /opt/weda_ws && colcon build 2>&1 | tail -2 ) || warn "colcon build failed — running previous build"
fi
[ -f /opt/weda_ws/install/setup.bash ] && source /opt/weda_ws/install/setup.bash \
    || warn "workspace overlay not built — run: cd /opt/weda_ws && colcon build"

# --- Pre-check: pipeline already running? ---------------------------------------
# Process-based (ros2 node list can serve a stale daemon cache after a crash).
if ps -eo args | grep -E "vlm_perception/lib/vlm_perception/vlm-(depth|perception)" | grep -qv grep; then
    fail "the perception pipeline is already running (the camera is exclusive).
       Stop it first (q in its demo terminal), or kill: pkill -f vlm-depth"
fi

# --- Pre-check: vLLM server health ---------------------------------------------
if curl -sf -m 3 http://127.0.0.1:8000/health >/dev/null 2>&1; then
    ok "vLLM server healthy on :8000"
else
    warn "vLLM server NOT reachable on :8000 — perception will publish"
    warn "'server_unreachable' payloads until it is up. Start it on the HOST:"
    warn "    ./build.sh --server     (first boot downloads model weights)"
fi

# --- GUI availability ----------------------------------------------------------
GUI=0
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ] \
   && ls /tmp/.X11-unix/X* >/dev/null 2>&1; then
    GUI=1
else
    warn "No usable DISPLAY — running headless (topics only, no rviz2/rqt)."
fi

# Window toggles from THE config (display: section); headless overrides all.
read -r SHOW_RVIZ SHOW_RQT SHOW_VIEWER <<< "$(python3 - "${CONFIG}" <<'PYCFG'
import sys, yaml
try:
    d = (yaml.safe_load(open(sys.argv[1])) or {}).get("display") or {}
except Exception:
    d = {}
flag = lambda k: "1" if d.get(k, True) else "0"
print(flag("rviz"), flag("rqt_graph"), flag("viewer"))
PYCFG
)"
if [ "${GUI}" = "1" ]; then
    ok "Display available (${DISPLAY}) — windows per config: rviz=${SHOW_RVIZ} rqt_graph=${SHOW_RQT} viewer=${SHOW_VIEWER}"
fi

# --- Launch the pipeline in its own process group ------------------------------
PIDS=()
CLEANED=0
cleanup() {
    [ "${CLEANED}" = "1" ] && return
    CLEANED=1
    echo ""
    cyan "Shutting down demo..."
    for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
    if [ -n "${LAUNCH_PGID:-}" ]; then
        # INT first: ros2 launch forwards it so nodes shut down gracefully.
        kill -INT -- "-${LAUNCH_PGID}" 2>/dev/null
        for _ in $(seq 1 15); do
            kill -0 -- "-${LAUNCH_PGID}" 2>/dev/null || break
            sleep 1
        done
        # A node blocked in a long VLM HTTP read ignores INT — escalate.
        if kill -0 -- "-${LAUNCH_PGID}" 2>/dev/null; then
            kill -TERM -- "-${LAUNCH_PGID}" 2>/dev/null; sleep 3
            kill -KILL -- "-${LAUNCH_PGID}" 2>/dev/null
        fi
    fi
    # Sweep any leaked helper from a previous unclean session (viewer/feed run
    # outside the launch pgid); pattern built at runtime to avoid self-match.
    _p="vlm_perception/vlm-"; pkill -KILL -f "${_p}feed" 2>/dev/null
    pkill -KILL -f "${_p}view" 2>/dev/null
    ok "Demo stopped — camera released."
}
trap cleanup INT TERM EXIT

setsid ros2 launch vlm_perception perception.launch.py "config:=${CONFIG}" \
    > /tmp/vlm_demo_launch.log 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= "${LAUNCH_PID}" | tr -d ' ')
ok "Pipeline launching (pgid ${LAUNCH_PGID}; log: /tmp/vlm_demo_launch.log)"

# Detector tier status — surfaced HERE because node logs go to the launch log,
# not this terminal (first-run engine export would otherwise be invisible).
if [ "${ACCEPT_COMMERCIAL_EULA:-}" != "true" ]; then
    warn "detector tier disabled (EULA not accepted) — VLM boxes only"
else
    ENGINE=$(python3 -c "import yaml; d=yaml.safe_load(open('${CONFIG}')); print((d.get('detector') or {}).get('model',''))" 2>/dev/null || true)
    if [ -n "${ENGINE}" ] && [ ! -f "${ENGINE}" ]; then
        ok "detector: first-run TensorRT engine export starting (~3 min) — live boxes appear when it completes (progress: /tmp/vlm_demo_launch.log)"
    fi
fi

# --- Wait for the camera / topics ----------------------------------------------
echo "  Waiting for camera topics..."
CAMERA_UP=0
for _ in $(seq 1 30); do
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "/vlm/depth_context"; then
        if timeout 4 ros2 topic echo --once /vlm/depth_context >/dev/null 2>&1; then
            CAMERA_UP=1; break
        fi
    fi
    kill -0 "${LAUNCH_PID}" 2>/dev/null || { tail -15 /tmp/vlm_demo_launch.log; fail "pipeline exited early"; }
    sleep 2
done
if [ "${CAMERA_UP}" = "1" ]; then
    ok "Depth camera streaming (aligned depth + context live)."
else
    warn "No depth frames after 60s — check the Orbbec USB connection."
    warn "Pipeline continues; perception runs without depth grounding."
fi

# --- GUI tools -----------------------------------------------------------------
if [ "${GUI}" = "1" ]; then
    if [ "${SHOW_RVIZ}" = "1" ] && [ -f "${RVIZ_PRESET}" ]; then
        rviz2 -d "${RVIZ_PRESET}" > /dev/null 2>&1 & PIDS+=($!)
        ok "rviz2 opened (color | depth colormap | annotated panels)."
    fi
    if [ "${SHOW_VIEWER}" = "1" ]; then
        # THE demo window: large annotated view (camera + boxes + distances + OCR)
        ros2 run vlm_perception vlm-view --config "${CONFIG}" > /tmp/vlm_view.log 2>&1 & PIDS+=($!)
        ok "vlm-view opened — LARGE annotated window (f = fullscreen, q = close)."
    fi
    if [ "${SHOW_RQT}" = "1" ]; then
        rqt_graph > /dev/null 2>&1 & PIDS+=($!)
        ok "rqt_graph opened (live node/topic graph)."
    fi
fi

# --- Live perception feed -------------------------------------------------------
cyan "----------------------------------------------------------------------"
cyan " Live /vlm/perception payloads (description + OCR + measured depths)  "
cyan " Press q or Esc to stop the demo.                                     "
cyan "----------------------------------------------------------------------"
# NOT exec: the shell must survive to run the cleanup trap —
# exec would orphan the setsid'd launch group and leave the camera claimed.
# With the viewer window up, the FULL response renders in the window banner —
# terminal feed silenced to avoid double output; headless keeps it as before.
if [ "${GUI}" = "1" ] && [ "${SHOW_VIEWER}" = "1" ]; then
    ok "Full responses appear IN the annotated window (terminal feed off)."
else
    ros2 run vlm_perception vlm-feed --config "${CONFIG}" &
    ECHO_PID=$!
    PIDS+=("${ECHO_PID}")
fi

if [ -t 0 ]; then
    # Interactive: single-keystroke listener (q/Q or Esc quits; Ctrl-C still works).
    while true; do
        # Stop if the pipeline died on its own
        kill -0 -- "-${LAUNCH_PGID}" 2>/dev/null || break
        IFS= read -rsn1 -t 1 key || continue
        if [ "$key" = $'\e' ]; then
            # Distinguish bare Esc from arrow/function-key escape sequences
            IFS= read -rsn2 -t 0.05 _rest && continue
            break
        fi
        case "$key" in q|Q) break ;; esac
    done
else
    # Non-interactive (automation/headless exec): wait on the feed; signals stop us.
    wait "${ECHO_PID}" 2>/dev/null || true
fi
cleanup

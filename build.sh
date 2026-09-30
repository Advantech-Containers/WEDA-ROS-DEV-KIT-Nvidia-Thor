#!/usr/bin/env bash
# ==============================================================================
# WEDA ROS VLM — Device Launcher
# NVIDIA Jetson AGX Thor (JetPack 7.x)
# ------------------------------------------------------------------------------
# Starts the ROS+VLM dev container and opens a shell inside it.
# Run from this directory on the target device.
#
# Usage:
#   ./build.sh              start the container and open a shell (uses the local
#                           image when present; pulls only if missing)
#   ./build.sh --pull       force-pull/refresh the image from the registry
#   ./build.sh --build      rebuild the local release image (Dockerfile overlay)
#   ./build.sh --shell      open a shell in the running container
#   ./build.sh --server     start the vLLM server sidecar (profile: vlm)
#   ./build.sh --bench      WISE-Bench: verify every package/function this
#                           container needs for ROS 2 + VLM (verdict + report)
#   ./build.sh --demo       run the VLM perception demo (camera + rviz2 + rqt)
#   ./build.sh --status     show container status and health
#   ./build.sh --logs       tail the container logs
#   ./build.sh --down       stop and remove the containers
#   ./build.sh --help       this text
# ==============================================================================
set -euo pipefail
IFS=$'\n\t'

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
cd "${SCRIPT_DIR}"

# ── Load Static Config from .env ──────────────────────────────────────────────
ENV_FILE="${SCRIPT_DIR}/.env"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"

[[ -f "${ENV_FILE}" ]] || { echo -e "\e[1;31m[FAIL]\e[0m  .env not found at ${ENV_FILE}" >&2; exit 1; }

# Caller-provided env wins over .env (fleet/CI path: ACCEPT_COMMERCIAL_EULA=true ./build.sh)
_CLI_EULA="${ACCEPT_COMMERCIAL_EULA:-}"
set -a
# shellcheck disable=SC1090
source "${ENV_FILE}"
set +a
[ -n "${_CLI_EULA}" ] && export ACCEPT_COMMERCIAL_EULA="${_CLI_EULA}"

readonly SERVICE="${SERVICE_NAME:-weda-ros-vlm}"
readonly CONTAINER="${CONTAINER_NAME:-weda-ros-vlm}"
readonly IMAGE="${WEDA_ROS_VLM_IMAGE}"

log_info()    { echo -e "\e[1;36m[INFO]\e[0m  $1"; }
log_success() { echo -e "\e[1;32m[ OK ]\e[0m  $1"; }
log_warn()    { echo -e "\e[1;33m[WARN]\e[0m  $1"; }
log_error()   { echo -e "\e[1;31m[FAIL]\e[0m  $1" >&2; exit 1; }

banner() {
    echo -e "\e[1;36m"
    cat << "EOF"
 __        _______ ____    _      ____   ___  ____   __     ___     __  __
 \ \      / / ____|  _ \  / \    |  _ \ / _ \/ ___|  \ \   / / |   |  \/  |
  \ \ /\ / /|  _| | | | |/ _ \   | |_) | | | \___ \   \ \ / /| |   | |\/| |
   \ V  V / | |___| |_| / ___ \  |  _ <| |_| |___) |   \ V / | |___| |  | |
    \_/\_/  |_____|____/_/   \_\ |_| \_\____/ |____/    \_/  |_____|_|  |_|
EOF
    echo -e "     ROS 2 + Vision-Language Perception Powered by Advantech WEDA-EDGE-AI\e[0m\n"
}

# ------------------------------------------------------------------------------
# Prerequisite checks
# ------------------------------------------------------------------------------
check_prereqs() {
    command -v docker >/dev/null 2>&1 \
        || log_error "docker is not installed. Install Docker Engine first."
    docker compose version >/dev/null 2>&1 \
        || log_error "docker compose v2 is not available. Install the compose plugin."
    docker info 2>/dev/null | grep -qi "nvidia" \
        || log_warn "NVIDIA container runtime not detected in 'docker info'. GPU access will fail — see TROUBLESHOOTING.md."
    [ "$(uname -m)" = "aarch64" ] \
        || log_warn "Host is $(uname -m), not aarch64. This image targets NVIDIA Jetson devices."

    # Workspace structure (world-writable: container writes as its own user)
    mkdir -p "${SCRIPT_DIR}/weda_ws/src"
    chmod a+rwX "${SCRIPT_DIR}/weda_ws" "${SCRIPT_DIR}/weda_ws/src"

    # Depth camera presence (informational — the app degrades gracefully without it)
    if lsusb 2>/dev/null | grep -qiE "orbbec|2bc5"; then
        log_success "Orbbec depth camera detected on USB."
    else
        log_warn "No Orbbec camera on USB — perception will run RGB-only/test-pattern."
    fi
}

# ------------------------------------------------------------------------------
# X11 Display Detection
# ------------------------------------------------------------------------------
detect_x11() {
    export X11_SOCKET=/tmp/.X11-unix

    if [[ -n "${DISPLAY:-}" ]]; then
        log_info "DISPLAY already set: ${DISPLAY}"
    else
        if [[ ! -d "${X11_SOCKET}" ]]; then
            log_warn "X11 socket directory not found: ${X11_SOCKET}. GUI output will be unavailable."
            export DISPLAY=":0"
            return
        fi
        mapfile -t X11_SOCKETS < <(find "${X11_SOCKET}" -name "X[0-9]*" -type s 2>/dev/null | sort)
        if [[ ${#X11_SOCKETS[@]} -eq 0 ]]; then
            log_warn "No X11 sockets found. GUI output will be unavailable."
            export DISPLAY=":0"
            return
        fi
        local DETECTED_SOCKET="${X11_SOCKETS[0]}"
        export DISPLAY=":${DETECTED_SOCKET##*X}"
        log_info "Detected DISPLAY=${DISPLAY}"
        [[ ${#X11_SOCKETS[@]} -gt 1 ]] \
            && log_warn "Multiple X11 sockets found — using first. Override: export DISPLAY=:N"
    fi

    if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
        log_info "XAUTHORITY already set: ${XAUTHORITY}"
    else
        local DETECTED_XAUTH=""
        for candidate in \
            "/run/user/$(id -u)/gdm/Xauthority" \
            "/run/user/$(id -u)/Xauthority" \
            "${HOME}/.Xauthority" \
            "/tmp/.Xauthority"; do
            if [[ -f "${candidate}" && -s "${candidate}" ]]; then
                DETECTED_XAUTH="${candidate}"
                break
            fi
        done
        if [[ -n "${DETECTED_XAUTH}" ]]; then
            export XAUTHORITY="${DETECTED_XAUTH}"
            log_info "Detected XAUTHORITY=${XAUTHORITY}"
        else
            log_warn "Could not detect XAUTHORITY. GUI output may be unavailable."
            export XAUTHORITY="/tmp/.Xauthority"
        fi
    fi

    command -v xhost &>/dev/null && xhost +local:docker > /dev/null 2>&1 || true
}

# ------------------------------------------------------------------------------
# EULA / BYOL License Gate — acceptance is SESSION-SCOPED and never written
# to .env; permanent acceptance is the operator's explicit choice (set
# ACCEPT_COMMERCIAL_EULA=true in .env). Declining leaves a clean OSS-only
# deployment.
# ------------------------------------------------------------------------------
eula_gate() {
    if [[ "${ACCEPT_COMMERCIAL_EULA:-}" == "true" ]]; then
        log_success "ACCEPT_COMMERCIAL_EULA=true (set in .env) — skipping prompt."
        return
    fi
    if [[ ! -t 0 ]]; then
        # Never REVOKE as a side effect: if the existing container already
        # carries acceptance (granted interactively), preserve it so a script/
        # CI invocation doesn't silently recreate the container without the
        # licensed tier. Decline happens only explicitly, at the prompt.
        existing=$(docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' \
            "${CONTAINER_NAME:-weda-ros-vlm}" 2>/dev/null \
            | sed -n 's/^ACCEPT_COMMERCIAL_EULA=//p' | head -1)
        if [[ "${existing}" == "true" ]]; then
            log_info "non-interactive: existing container already carries EULA acceptance — preserving it."
            export ACCEPT_COMMERCIAL_EULA=true
        else
            log_warn "non-interactive session: skipping EULA prompt — gated components stay disabled."
            log_warn "For unattended deployments set ACCEPT_COMMERCIAL_EULA=true in .env yourself."
            export ACCEPT_COMMERCIAL_EULA=false
        fi
        return
    fi

    echo -e "\e[1;36m╔══════════════════════════════════════════════════════════════════════╗\e[0m"
    echo -e "\e[1;36m║        WEDA ROS VLM — Component License Agreement                    ║\e[0m"
    echo -e "\e[1;36m╚══════════════════════════════════════════════════════════════════════╝\e[0m"
    echo ""
    echo "  This container includes components under the NVIDIA EULA and can"
    echo "  install optional license-gated packages at startup"
    echo "  (WISE_LICENSED_PIP in .env). Acceptance enables those components."
    echo ""
    echo "  [1] Display the full EULA / License text"
    echo "  [2] Skip display and proceed to acceptance"
    echo "  [3] Decline — continue without gated components"
    echo ""
    printf "  Enter choice [1/2/3]: "
    read -r DISPLAY_CHOICE

    case "$DISPLAY_CHOICE" in
        1)
            echo "══════════════════════════════════════════════════════════════"
            docker run --rm --entrypoint cat "${IMAGE}" \
                /usr/local/share/doc/nvidia/EULA.txt 2>/dev/null \
                || echo "  (License text not available for preview.)"
            echo "══════════════════════════════════════════════════════════════"
            ;;
        3)
            log_warn "EULA declined — gated components will not be installed."
            export ACCEPT_COMMERCIAL_EULA=false
            return
            ;;
    esac

    printf "\n  Accept EULA and BYOL terms? [yes/no]: "
    read -r USER_ACCEPT
    if [[ "$(echo "$USER_ACCEPT" | tr '[:upper:]' '[:lower:]')" =~ ^(yes|y)$ ]]; then
        export ACCEPT_COMMERCIAL_EULA=true
        log_success "EULA accepted for THIS session only (nothing written to .env)."
        log_info "To make acceptance permanent, set ACCEPT_COMMERCIAL_EULA=true in .env yourself."
    else
        export ACCEPT_COMMERCIAL_EULA=false
        log_warn "EULA not accepted."
    fi
}

# ------------------------------------------------------------------------------
# Image management
# ------------------------------------------------------------------------------
do_pull() {
    log_info "Pulling ${IMAGE} ..."
    docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" pull "${SERVICE}"
    log_success "Image up to date."
}

do_build() {
    log_info "Building local release image from Dockerfile ..."
    docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" build "${SERVICE}"
    log_success "Image built."
}

ensure_image() {
    if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
        log_info "Using local image ${IMAGE} (run './build.sh --pull' to refresh)."
    else
        log_info "Image not present on this device — pulling once."
        do_pull
    fi
}

do_up() {
    ensure_image
    docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" up -d "${SERVICE}"
    log_success "Container '${CONTAINER}' is running."
}

# Wait for entrypoint hydration (.ready signal)
wait_hydration() {
    echo "  Waiting for environment hydration..."
    local TIMEOUT=300 elapsed=0
    while [[ $elapsed -lt $TIMEOUT ]]; do
        if docker exec "${CONTAINER}" ls /tmp/.ready &>/dev/null; then
            printf "\r  [\033[1;32m%s\033[0m] 100%%\n" "$(printf "%*s" 50 | tr ' ' '#')"
            return 0
        fi
        local pct=$(( elapsed * 100 / TIMEOUT ))
        local hashes=$(( (pct * 50) / 100 ))
        printf "\r  [\033[1;36m%s\033[0m%s] %3d%% " \
            "$(printf "%*s" $hashes "" | tr ' ' '#')" \
            "$(printf "%*s" $(( 50 - hashes )) "")" "$pct"
        sleep 2
        elapsed=$(( elapsed + 2 ))
    done
    echo -e "\n  [\033[1;31mTIMEOUT\033[0m]"
    log_warn "Hydration timeout (${TIMEOUT}s). Check: docker logs ${CONTAINER}"
}

do_shell() {
    do_up
    wait_hydration
    log_info "Opening shell (exit with Ctrl-D)..."
    exec docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" exec "${SERVICE}" /bin/bash -c '
        source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
        if [ -f /opt/weda_ws/install/setup.bash ]; then source /opt/weda_ws/install/setup.bash; fi
        exec /bin/bash -i'
}

do_demo() {
    do_up
    wait_hydration
    log_info "Launching the VLM perception demo (press q or Esc to stop)..."
    exec docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" exec "${SERVICE}" \
        /opt/weda-release/launch_demo.sh
}

do_server() {
    log_info "Starting vLLM server sidecar (first boot downloads model weights)..."
    docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" --profile vlm up -d vlm-server
    log_success "vlm-server starting — watch: curl -sf http://127.0.0.1:8000/health"
}

do_bench() {
    do_up
    wait_hydration
    local rc=0
    log_info "[1/2] WISE-Bench: ROS container (weda-ros-vlm)..."
    docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" exec "${SERVICE}" /bin/bash -c '
        source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
        if [ -f /opt/weda_ws/install/setup.bash ]; then source /opt/weda_ws/install/setup.bash; fi
        /opt/weda-release/tests/wise-bench-ros-vlm' || rc=1

    if docker ps --format '{{.Names}}' | grep -q '^weda-vlm-server$'; then
        log_info "[2/2] WISE-Bench: server container (weda-vlm-server, vLLM stack)..."
        docker exec weda-vlm-server \
            /opt/weda-release/tests/wise-bench-vlm-server || rc=1
        # server writes to its /tmp — collect onto the host next to the ROS
        # reports. The directory is created by the container, so it is owned by
        # root; docker cp then writes into it as the host user and is refused.
        # Hand it to the invoking user instead of widening the mode: the ids
        # resolve on the host, and the bind mount makes the change visible on
        # both sides.
        docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" exec "${SERVICE}" \
            chown -R "$(id -u):$(id -g)" /opt/weda_ws/reports 2>/dev/null || true
        docker cp weda-vlm-server:/tmp/wise-bench-reports/. \
            "${SCRIPT_DIR}/weda_ws/reports/" 2>/dev/null \
            || log_warn "server reports could not be collected to weda_ws/reports/"
    else
        log_warn "[2/2] vlm-server not running — server bench skipped (start: ./build.sh --server)"
    fi

    [ $rc -eq 0 ] \
        && log_success "WISE-Bench verdict: PASS — reports in weda_ws/reports/" \
        || log_error "WISE-Bench verdict: FAIL — verbose diagnostics in weda_ws/reports/"
}

do_status() { docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" ps; }
do_logs()   { docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" logs -f --tail 100 "${SERVICE}"; }

do_down() {
    export X11_SOCKET=${X11_SOCKET:-/tmp/.X11-unix}
    export XAUTHORITY=${XAUTHORITY:-/tmp/.Xauthority}
    export DISPLAY=${DISPLAY:-:0}
    docker compose -f "${COMPOSE_FILE}" --env-file "${ENV_FILE}" --profile vlm down
    log_success "Containers stopped and removed."
}

usage() { sed -n '3,23p' "$0" | sed 's/^# \{0,1\}//'; }

# ------------------------------------------------------------------------------
banner
case "${1:-}" in
    --pull)      check_prereqs; detect_x11; do_pull ;;
    --build)     check_prereqs; detect_x11; do_build ;;
    --shell)     check_prereqs; detect_x11; eula_gate; do_shell ;;
    --demo)      check_prereqs; detect_x11; eula_gate; do_demo ;;
    --server)    check_prereqs; do_server ;;
    --bench)     check_prereqs; detect_x11; eula_gate; do_bench ;;
    --status)    do_status ;;
    --logs)      do_logs ;;
    --down)      do_down ;;
    --help|-h)   usage ;;
    "")          check_prereqs; detect_x11; eula_gate; do_shell ;;
    *)           log_error "Unknown option: $1 (see ./build.sh --help)" ;;
esac

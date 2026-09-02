#!/bin/bash
# bridge - Docker Compose + ROS 2 node management helper
# Usage: bridge [-f compose.yml] <command> [args...]

set -euo pipefail

# ──────────────────────────────────────────────────────────────
# Configuration (edit these to match your project)
# ──────────────────────────────────────────────────────────────
DEFAULT_COMPOSE_FILE=/home/sreeram/major_project/compose.yml
SERVICES=("mediamtx" "vision" "ros2")          # must match service names in compose
ROS_SERVICE="ros2"                             # service that runs ROS 2 nodes

# ──────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────
COMPOSE_FILE="${COMPOSE_FILE:-$DEFAULT_COMPOSE_FILE}"
COMPOSE_CMD=(docker compose -f "$COMPOSE_FILE")

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()  { echo -e "${BLUE}ℹ${NC}  $*"; }
ok()    { echo -e "${GREEN}✅${NC} $*"; }
warn()  { echo -e "${YELLOW}⚠️${NC}  $*"; }
err()   { echo -e "${RED}❌${NC} $*"; }

usage() {
    cat <<EOF
Usage: bridge [options] <command> [args...]

Options:
  -f, --file FILE     Path to compose.yml (default: $DEFAULT_COMPOSE_FILE)
                      Can also be set via COMPOSE_FILE environment variable

Commands:
  status              Show status of all containers
  start   [SERVICE]   Start all services or one service (mediamtx|vision|ros2)
  stop    [SERVICE]   Stop all services or one service
  restart [SERVICE]   Restart all services or one service
  down                Stop and remove all containers/networks (keeps volumes)
  shell   <SERVICE>   Open interactive shell in a container
  exec    <SERVICE> <command...>   Run a command inside a running container
  logs    [SERVICE]   Follow logs (all or one service)

  node status <name>              Check if a ROS 2 node is running
  node stop   <name>              Stop a running ROS 2 node (pkill)
  node start <node_name>          Start a node in background inside ros2 container

Examples:
  bridge status
  bridge start vision
  bridge stop
  bridge shell ros2
  bridge exec ros2 esptool.py --port /dev/esp32 write_flash 0x10000 /workspace/firmware.bin
  bridge node status manipulator_bridge
  bridge node stop manipulator_bridge
  bridge node start manipulator_bridge
EOF
    exit 1
}

# Parse -f / --file
while [[ $# -gt 0 ]]; do
    case "$1" in
        -f|--file)
            COMPOSE_FILE="$2"
            COMPOSE_CMD=(docker compose -f "$COMPOSE_FILE")
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            break
            ;;
    esac
done

if [[ $# -lt 1 ]]; then
    usage
fi

CMD="$1"
shift

# Validate compose file exists
if [[ ! -f "$COMPOSE_FILE" ]]; then
    err "Compose file not found: $COMPOSE_FILE"
    exit 1
fi

# ──────────────────────────────────────────────────────────────
# Core functions
# ──────────────────────────────────────────────────────────────
is_service_valid() {
    local svc="$1"
    for s in "${SERVICES[@]}"; do
        [[ "$s" == "$svc" ]] && return 0
    done
    return 1
}

container_running() {
    local svc="$1"
    "${COMPOSE_CMD[@]}" ps --services --filter "status=running" 2>/dev/null | grep -qx "$svc"
}

require_running() {
    local svc="$1"
    if ! container_running "$svc"; then
        err "Container '$svc' is not running. Start it first with: bridge start $svc"
        exit 1
    fi
}

do_status() {
    info "Compose file: $COMPOSE_FILE"
    echo
    "${COMPOSE_CMD[@]}" ps
    echo
    local all_up=true
    for svc in "${SERVICES[@]}"; do
        if container_running "$svc"; then
            ok "$svc is running"
        else
            warn "$svc is NOT running"
            all_up=false
        fi
    done
    if $all_up; then
        ok "All containers are up"
    else
        warn "Some containers are down"
        exit 1
    fi
}

do_start() {
    local target="${1:-all}"
    if [[ "$target" == "all" ]]; then
        info "Starting all services..."
        "${COMPOSE_CMD[@]}" up -d
        ok "All services started"
    else
        if ! is_service_valid "$target"; then
            err "Unknown service: $target (valid: ${SERVICES[*]})"
            exit 1
        fi
        info "Starting $target..."
        "${COMPOSE_CMD[@]}" up -d "$target"
        ok "$target started"
    fi
}

do_stop() {
    local target="${1:-all}"
    if [[ "$target" == "all" ]]; then
        info "Stopping all services..."
        "${COMPOSE_CMD[@]}" stop
        ok "All services stopped"
    else
        if ! is_service_valid "$target"; then
            err "Unknown service: $target (valid: ${SERVICES[*]})"
            exit 1
        fi
        info "Stopping $target..."
        "${COMPOSE_CMD[@]}" stop "$target"
        ok "$target stopped"
    fi
}

do_restart() {
    local target="${1:-all}"
    if [[ "$target" == "all" ]]; then
        info "Restarting all services..."
        "${COMPOSE_CMD[@]}" restart
        ok "All services restarted"
    else
        if ! is_service_valid "$target"; then
            err "Unknown service: $target (valid: ${SERVICES[*]})"
            exit 1
        fi
        info "Restarting $target..."
        "${COMPOSE_CMD[@]}" restart "$target"
        ok "$target restarted"
    fi
}

do_down() {
    info "Bringing everything down (containers + networks)..."
    "${COMPOSE_CMD[@]}" down
    ok "Done"
}

do_shell() {
    local svc="${1:-}"
    if [[ -z "$svc" ]]; then
        err "Usage: bridge shell <service>"
        err "Available: ${SERVICES[*]}"
        exit 1
    fi
    if ! is_service_valid "$svc"; then
        err "Unknown service: $svc"
        exit 1
    fi
    require_running "$svc"
    info "Opening shell in $svc  (type 'exit' to leave)"
    "${COMPOSE_CMD[@]}" exec "$svc" bash || "${COMPOSE_CMD[@]}" exec "$svc" sh
}

do_exec() {
    local svc="${1:-}"
    shift || true

    if [[ -z "$svc" || $# -eq 0 ]]; then
        err "Usage: bridge exec <service> <command...>"
        err "Example: bridge exec ros2 esptool.py --port /dev/esp32 write_flash 0x10000 /firmware.bin"
        exit 1
    fi

    if ! is_service_valid "$svc"; then
        err "Unknown service: $svc (valid: ${SERVICES[*]})"
        exit 1
    fi

    require_running "$svc"
    "${COMPOSE_CMD[@]}" exec -T "$svc" "$@" </dev/null
}

do_logs() {
    local svc="${1:-}"
    if [[ -z "$svc" ]]; then
        "${COMPOSE_CMD[@]}" logs -f --tail=100
    else
        if ! is_service_valid "$svc"; then
            err "Unknown service: $svc"
            exit 1
        fi
        "${COMPOSE_CMD[@]}" logs -f --tail=100 "$svc"
    fi
}

# ──────────────────────────────────────────────────────────────
# ROS 2 node helpers
# ──────────────────────────────────────────────────────────────
ros_exec() {
    # Run a command inside the ros2 container with ROS sourced
    "${COMPOSE_CMD[@]}" exec -T "$ROS_SERVICE" bash -c "$1" </dev/null
}

node_status() {
    local node_name="${1:-}"
    if [[ -z "$node_name" ]]; then
        err "Usage: bridge node status <node_name>"
        exit 1
    fi

    info "Checking for ROS 2 node: $node_name"
    if ros_exec "ros2 node list 2>/dev/null" | grep -qx "/$node_name"; then
        ok "Node '$node_name' is running"
    #     # echo
    #     # ros_exec "ros2 node info /$node_name 2>/dev/null || ros2 node info $node_name 2>/dev/null || true"
        return 0
    else
        warn "Node '$node_name' is NOT running"
        return 1
    fi
}

node_stop() {
    local node_name="${1:-}"

    if [[ -z "$node_name" ]]; then
        err "Usage: bridge node stop <node_name>"
        return 1
    fi

    if ! node_status "$node_name" >/dev/null 2>&1; then
        warn "Node '$node_name' is not running"
        return 0
    fi

    info "Stopping node '$node_name'..."

    local pid
    pid=$(ros_exec "pgrep -f '/ros2_ws/install/delta_nodes/lib/delta_nodes/$node_name' | head -n1" 2>/dev/null || true)

    if [[ -n "$pid" ]]; then
        ros_exec "kill '$pid' 2>/dev/null || true"
    fi

    sleep 1

    if node_status "$node_name" >/dev/null 2>&1; then
        warn "Still running, sending SIGKILL..."

        pid=$(ros_exec "pgrep -f '/ros2_ws/install/delta_nodes/lib/delta_nodes/$node_name' | head -n1" 2>/dev/null || true)

        if [[ -n "$pid" ]]; then
            ros_exec "kill -9 '$pid' 2>/dev/null || true"
        fi

        sleep 1
    fi

    if node_status "$node_name" >/dev/null 2>&1; then
        err "Node '$node_name' is still running"
        return 1
    fi

    ok "Node '$node_name' stopped"
    return 0
}

node_start() {
    local node_name="${1:-}"

    if [[ -z "$node_name" ]]; then
        err "Usage: bridge node start <node_name>"
        err "Example: bridge node start manipulator_bridge"
        exit 1
    fi

    if node_status "$node_name" >/dev/null 2>&1; then
        warn "Node '$node_name' is already running"
        return 0
    fi

    local log_file="/tmp/${node_name//\//_}.log"
    local cmd="ros2 run delta_nodes $node_name"

    info "Starting node '$node_name' in background..."
    info "Command: $cmd"
    info "Log file inside container: $log_file"

    ros_exec "nohup $cmd > '$log_file' 2>&1 &"

    # Give the node up to 5 seconds to appear
    local i
    for i in {1..10}; do
        if node_status "$node_name" >/dev/null 2>&1; then
            ok "Node '$node_name' started successfully"
            return 0
        fi
        sleep 0.5
    done

    warn "Node '$node_name' did not appear in 'ros2 node list'."
    warn "Check the log: bridge shell $ROS_SERVICE → cat $log_file"
    return 1
}
# ──────────────────────────────────────────────────────────────
# Main dispatcher
# ──────────────────────────────────────────────────────────────
case "$CMD" in
    status)
        do_status
        ;;
    start)
        do_start "${1:-all}"
        ;;
    stop)
        do_stop "${1:-all}"
        ;;
    restart)
        do_restart "${1:-all}"
        ;;
    down)
        do_down
        ;;
    shell|term)
        do_shell "${1:-}"
        ;;
    exec)
        do_exec "$@"
        ;;
    logs)
        do_logs "${1:-}"
        ;;
    node)
        subcmd="${1:-}"
        shift || true
        case "$subcmd" in
            status) node_status "${1:-}" ;;
            stop)   node_stop   "${1:-}" ;;
            start)  node_start  "$@" ;;
            *)
                err "Unknown node subcommand: $subcmd"
                echo "Available: status | stop | start"
                exit 1
                ;;
        esac
        ;;
    *)
        err "Unknown command: $CMD"
        usage
        ;;
esac
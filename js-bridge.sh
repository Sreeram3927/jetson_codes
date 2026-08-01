#!/bin/bash

# Run the below command to add this to user binaries
# chmod +x js-bridge.sh
# sudo cp js-bridge.sh /usr/local/bin/bridge

# Check if an argument was provided
if [ -z "$1" ]; then
    echo "Usage: bridge [mode]"
    echo "mode: start / stop / restart / status / shell"
    exit 1
fi

CONTAINER_NAME="jetson_core"
TELEMETRY_PROCESS_MATCH="[m]ain.py"
MEDIAMTX_MATCH="mediamtx"
TELEMETRY_START_CMD="python3 /workspace/main.py"

start_telemetry() {
    docker exec -d $CONTAINER_NAME sh -c "nohup $TELEMETRY_START_CMD > /workspace/logs/telemetry.log 2>&1"
}
check_telemetry_process() {
    # docker exec returns exit status 0 if pgrep finds a match, non-zero if not
    docker exec "$CONTAINER_NAME" pgrep -f "$TELEMETRY_PROCESS_MATCH" > /dev/null 2>&1
}
stop_telemetry() {
    docker exec $CONTAINER_NAME pkill -f "$TELEMETRY_PROCESS_MATCH"
}

start_mediamtx() {
    # Run mediamtx in a new shell, cd into the directory, then start it in the background
    nohup bash -c 'cd ~/major_project/mediamtx_server && ./mediamtx' \
        > ~/major_project/logs/mediamtx.log 2>&1 &
}
check_mediamtx_process() {
    pgrep -f "$MEDIAMTX_MATCH" > /dev/null 2>&1
}
stop_mediamtx() {
    pkill -f "$MEDIAMTX_MATCH"
}

# Ensure the Docker container is actually running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ Error: Docker container '${CONTAINER_NAME}' is not running!"
    exit 1
fi

if [ "$1" == "status" ]; then
    if check_telemetry_process; then
        echo "Telemetry is running"
    fi
    if check_mediamtx_process; then
        echo "MediaMTX is running"
    fi

    if ! check_telemetry_process && ! check_mediamtx_process; then
        echo "No process is running"
        exit 1
    fi

elif [ "$1" == "start" ]; then
    echo "🚀 Starting bridge inside Docker..."

    if check_telemetry_process; then
        echo "⚠️ Telemetry is already running!"
        exit 0
    fi
    start_telemetry
    sleep 1
    echo "✅ Telemetry started in background."

    if check_mediamtx_process; then
        echo "⚠️ MediaMTX is already running!"
        exit 0
    fi
    start_mediamtx
    sleep 1
    echo "✅ MediaMTX started in background."

    exit 0

elif [ "$1" == "stop" ]; then
    if check_telemetry_process; then
        echo "🛑 Telemetry is running inside Docker. Stopping it..."
        stop_telemetry
        sleep 2 
        echo "✅ Stopped."
    else
        echo "⏸️ Telemetry is not currently running inside Docker."
    fi

    if check_mediamtx_process; then
        echo "🛑 MediaMTX is running. Stopping it..."
        stop_mediamtx
        sleep 2 
        echo "✅ Stopped."
    else
        echo "⏸️ MediaMTX is not currently running inside Docker."
    fi

elif [ "$1" == "restart" ]; then
    if ! check_mediamtx_process; then
        echo "MediaMTX is not running, Starting it..."
        start_mediamtx
        sleep 1
        echo "Started MediaMTX"
    fi

    echo "Restarting telemetry"
    stop_telemetry
    sleep 2
    start_telemetry
    echo "Telemetry has started"

elif [ "$1" == "shell" ] || [ "$1" == "term" ]; then
    echo "🔌 Connecting to Docker container terminal..."
    echo "Type 'exit' to leave the container and return to Jetson host."
    # -it creates an interactive terminal session
    docker exec -it $CONTAINER_NAME /bin/bash
    exit 0

else
    echo "❌ Invalid Usage"
    echo "Usage: bridge [mode]"
    echo "mode: start / stop / restart / status / shell"
    exit 1
fi
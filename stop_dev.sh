#!/bin/bash

PROJECT_ROOT=$(pwd)
PID_DIR="$PROJECT_ROOT/.pids"

stop_process() {
  NAME=$1
  PID_FILE="$PID_DIR/$NAME.pid"

  if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")

    if kill -0 "$PID" 2>/dev/null; then
      echo "Stopping $NAME with PID $PID..."
      kill "$PID"

      sleep 2

      if kill -0 "$PID" 2>/dev/null; then
        echo "$NAME did not stop cleanly. Forcing stop..."
        kill -9 "$PID"
      fi
    else
      echo "$NAME is not running."
    fi

    rm -f "$PID_FILE"
  else
    echo "No PID file found for $NAME."
  fi
}

echo "Stopping IoT Robot development stack..."
echo ""

stop_process "tracking_controller"
stop_process "camera_stream"
stop_process "frontend"
stop_process "backend"

echo ""
echo "Cleaning possible remaining frontend/backend ports..."

fuser -k 5173/tcp 2>/dev/null
fuser -k 3001/tcp 2>/dev/null

echo ""
echo "Stopped."
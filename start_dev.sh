#!/bin/bash

PROJECT_ROOT=$(pwd)

echo "Starting IoT Robot development stack..."
echo "Project root: $PROJECT_ROOT"
echo ""

mkdir -p "$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/.pids"

echo "Starting backend..."
cd "$PROJECT_ROOT/backend" || exit 1
nohup npm run dev > "$PROJECT_ROOT/logs/backend.log" 2>&1 &
echo $! > "$PROJECT_ROOT/.pids/backend.pid"

sleep 3

echo "Starting frontend..."
cd "$PROJECT_ROOT/frontend" || exit 1
nohup npm run dev > "$PROJECT_ROOT/logs/frontend.log" 2>&1 &
echo $! > "$PROJECT_ROOT/.pids/frontend.pid"

sleep 3

echo "Starting camera stream with TARGET=bird..."
cd "$PROJECT_ROOT/rpi" || exit 1
nohup bash -c "source venv/bin/activate && TARGET=bird python -m camera.camera_stream" > "$PROJECT_ROOT/logs/camera_stream.log" 2>&1 &
echo $! > "$PROJECT_ROOT/.pids/camera_stream.pid"

sleep 8

echo "Starting tracking controller..."
cd "$PROJECT_ROOT/rpi" || exit 1
nohup bash -c "source venv/bin/activate && python -m tracking.tracking_controller" > "$PROJECT_ROOT/logs/tracking_controller.log" 2>&1 &
echo $! > "$PROJECT_ROOT/.pids/tracking_controller.pid"

echo ""
echo "Development stack started."
echo ""
echo "Frontend: http://iotrobot.local:5173"
echo "Backend:  http://iotrobot.local:3001"
echo "Stream:   http://iotrobot.local:5000/video_feed"
echo ""
echo "Logs:"
echo "  logs/backend.log"
echo "  logs/frontend.log"
echo "  logs/camera_stream.log"
echo "  logs/tracking_controller.log"
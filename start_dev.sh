#!/bin/bash

PROJECT_ROOT=$(pwd)

# Detect Raspberry Pi local IP
PI_IP=$(hostname -I | awk '{print $1}')

echo "Starting IoT Robot development stack..."
echo "Project root: $PROJECT_ROOT"
echo "Detected Raspberry Pi IP: $PI_IP"
echo ""

mkdir -p "$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/.pids"

# --------------------------------------------------
# Auto-update frontend .env
# --------------------------------------------------

cat > "$PROJECT_ROOT/frontend/.env" <<EOF
VITE_BACKEND_URL=http://$PI_IP:3001
VITE_STREAM_URL=http://$PI_IP:8080/stream.mjpg
EOF

echo "Updated frontend/.env"
echo ""

# --------------------------------------------------
# Backend
# --------------------------------------------------

echo "Starting backend..."
cd "$PROJECT_ROOT/backend" || exit 1
nohup npm run dev > "$PROJECT_ROOT/logs/backend.log" 2>&1 &
echo $! > "$PROJECT_ROOT/.pids/backend.pid"

sleep 3

# --------------------------------------------------
# Frontend
# --------------------------------------------------

echo "Starting frontend..."
cd "$PROJECT_ROOT/frontend" || exit 1
nohup npm run dev > "$PROJECT_ROOT/logs/frontend.log" 2>&1 &
echo $! > "$PROJECT_ROOT/.pids/frontend.pid"

sleep 3

# --------------------------------------------------
# Camera stream
# --------------------------------------------------

echo "Starting camera stream with TARGET=bird..."
cd "$PROJECT_ROOT/rpi" || exit 1

nohup bash -c "
source venv/bin/activate &&
TARGET=bird python -m camera.camera_stream
" > "$PROJECT_ROOT/logs/camera_stream.log" 2>&1 &

echo $! > "$PROJECT_ROOT/.pids/camera_stream.pid"

sleep 8

# --------------------------------------------------
# Tracking controller
# --------------------------------------------------

echo "Starting tracking controller..."
cd "$PROJECT_ROOT/rpi" || exit 1

nohup bash -c "
source venv/bin/activate &&
python -m tracking.tracking_controller
" > "$PROJECT_ROOT/logs/tracking_controller.log" 2>&1 &

echo $! > "$PROJECT_ROOT/.pids/tracking_controller.pid"

echo ""
echo "Development stack started."
echo ""
echo "Frontend: http://$PI_IP:5173"
echo "Backend:  http://$PI_IP:3001"
echo "Stream:   http://$PI_IP:8080/stream.mjpg"
echo ""
echo "Logs:"
echo "  logs/backend.log"
echo "  logs/frontend.log"
echo "  logs/camera_stream.log"
echo "  logs/tracking_controller.log"
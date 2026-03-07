#!/bin/bash

PROJECT_ROOT=$(pwd)

echo "Starting backend..."

cd "$PROJECT_ROOT/backend" || exit
nohup npm run dev > "$PROJECT_ROOT/backend/dev.log" 2>&1 &
echo $! > "$PROJECT_ROOT/backend/dev.pid"

sleep 2

echo "Starting frontend..."

cd "$PROJECT_ROOT/frontend" || exit
nohup npm run dev > "$PROJECT_ROOT/frontend/dev.log" 2>&1 &
echo $! > "$PROJECT_ROOT/frontend/dev.pid"

echo ""
echo "Development servers started"
echo "Frontend: http://iotrobot.local:5173"
echo "Backend:  http://iotrobot.local:3001"
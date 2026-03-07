#!/bin/bash

echo "Stopping backend on port 3001..."
fuser -k 3001/tcp 2>/dev/null

echo "Stopping frontend on port 5173..."
fuser -k 5173/tcp 2>/dev/null

echo "Stopped."
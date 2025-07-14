#!/bin/bash

# Script to copy the latest raw map from robot to laptop
# Usage: ./copy_latest_map.sh

ROBOT_IP="172.16.14.113"  # Change this to your robot's IP
ROBOT_USER="robot"
ROBOT_MAPS_DIR="/tmp/robot_maps"
LAPTOP_MAPS_DIR="/home/avenblake/robot_maps"

# Create laptop maps directory if it doesn't exist
mkdir -p "$LAPTOP_MAPS_DIR"

echo "Copying latest raw map from robot..."

# Find the latest raw map on robot and copy it
ssh "$ROBOT_USER@$ROBOT_IP" "ls -t $ROBOT_MAPS_DIR/raw_map_*.pcd 2>/dev/null | head -1" | while read -r latest_file; do
    if [ -n "$latest_file" ]; then
        echo "Found latest map: $latest_file"
        scp "$ROBOT_USER@$ROBOT_IP:$latest_file" "$LAPTOP_MAPS_DIR/"
        echo "✓ Copied to: $LAPTOP_MAPS_DIR/$(basename "$latest_file")"
    else
        echo "No raw map files found on robot"
    fi
done

echo "Map copy complete!" 
#!/bin/bash

# Check Transforms for Tilted LiDAR Setup
# This script verifies that the static transforms are properly published

echo "=== Checking Transforms for Tilted LiDAR Setup ==="
echo ""

# Check if tf2_echo is available
if ! command -v tf2_echo &> /dev/null; then
    echo "❌ tf2_echo not found. Please install tf2_tools:"
    echo "   sudo apt install ros-humble-tf2-tools"
    exit 1
fi

echo "🔍 Checking transform from 'body' to 'foot'..."
echo "   This should show a 30-degree rotation around Y-axis (pitch)"
echo ""

# Check body to foot transform
echo "Transform: body → foot"
tf2_echo body foot 2>/dev/null | head -10

echo ""
echo "🔍 Checking transform from 'camera_init' to 'foot_init'..."
echo "   This should show the same 30-degree rotation for visualization correction"
echo ""

# Check camera_init to foot_init transform
echo "Transform: camera_init → foot_init"
tf2_echo camera_init foot_init 2>/dev/null | head -10

echo ""
echo "🔍 Checking available frames..."
echo ""

# List all available frames
echo "Available frames:"
ros2 run tf2_tools view_frames --ros-args -p output_file=/tmp/frames.pdf 2>/dev/null
if [ -f /tmp/frames.pdf ]; then
    echo "✅ Frame tree saved to /tmp/frames.pdf"
    echo "   You can view it with: evince /tmp/frames.pdf"
else
    echo "❌ Could not generate frame tree"
fi

echo ""
echo "=== Transform Check Complete ==="
echo ""
echo "📋 Expected Results:"
echo "   • body → foot: Should show rotation around Y-axis of ~-0.5230 radians (-30°)"
echo "   • camera_init → foot_init: Should show same rotation"
echo "   • In Foxglove, select 'foot_init' frame for corrected visualization"
echo ""
echo "🎯 Usage:"
echo "   1. Start robot: ros2 launch pilot_control robot_complete.launch.py"
echo "   2. Start laptop control: ros2 launch pilot_control laptop_teleop.launch.py"
echo "   3. In Foxglove, set frame to 'foot_init' for corrected map view"
echo "" 
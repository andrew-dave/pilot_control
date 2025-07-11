#!/bin/bash

# Check Maps Script
# Lists saved robot maps on the laptop

MAP_DIR="$HOME/robot_maps"

echo "🤖 Robot Maps Checker"
echo "====================="

if [ ! -d "$MAP_DIR" ]; then
    echo "❌ No maps directory found at: $MAP_DIR"
    echo "   Maps will be saved here when you press 'M' in teleop"
    exit 1
fi

echo "📁 Maps directory: $MAP_DIR"
echo ""

# Count total maps
TOTAL_MAPS=$(find "$MAP_DIR" -name "*.pcd" | wc -l)
echo "📊 Total maps found: $TOTAL_MAPS"

if [ "$TOTAL_MAPS" -eq 0 ]; then
    echo "   No maps found yet. Press 'M' in teleop to save your first map!"
    exit 0
fi

echo ""
echo "🗺️  Available maps:"
echo "=================="

# List maps with details
find "$MAP_DIR" -name "*.pcd" -type f | sort | while read -r map_file; do
    filename=$(basename "$map_file")
    size=$(du -h "$map_file" | cut -f1)
    date=$(stat -c "%y" "$map_file" | cut -d' ' -f1)
    time=$(stat -c "%y" "$map_file" | cut -d' ' -f2 | cut -d'.' -f1)
    
    echo "📄 $filename"
    echo "   📅 Date: $date $time"
    echo "   📏 Size: $size"
    echo ""
done

echo "💡 Tips:"
echo "   - Use 'pcl_viewer filename.pcd' to view maps"
echo "   - Maps are filtered (0.1m-2.0m height, outliers removed)"
echo "   - Each map contains the complete laser scan data" 
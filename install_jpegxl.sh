#!/bin/bash

# JPEG XL Installation Script for Unified Data Collector
# This script installs the JPEG XL library for state-of-the-art image compression

echo "Installing JPEG XL library for unified_data_collector..."

# Check if we're on Ubuntu/Debian
if command -v apt &> /dev/null; then
    echo "Detected Ubuntu/Debian system"
    
    # Update package list
    sudo apt update
    
    # Install JPEG XL development package
    sudo apt install -y libjxl-dev
    
    if [ $? -eq 0 ]; then
        echo "✅ JPEG XL library installed successfully!"
        echo "The unified_data_collector will now use JPEG XL compression."
        echo ""
        echo "Expected benefits:"
        echo "  - 50-60% better compression than JPEG"
        echo "  - Superior quality retention for computer vision"
        echo "  - Visually lossless compression at distance=1.0"
        echo ""
        echo "Configuration parameters:"
        echo "  - jxl_effort: 1-9 (7 recommended for good balance)"
        echo "  - jxl_distance: 0.0=lossless, 1.0=visually lossless"
    else
        echo "❌ Failed to install JPEG XL library"
        echo "The system will fall back to PNG compression"
        exit 1
    fi

# Check if we're on macOS
elif command -v brew &> /dev/null; then
    echo "Detected macOS system"
    
    # Install JPEG XL via Homebrew
    brew install jpeg-xl
    
    if [ $? -eq 0 ]; then
        echo "✅ JPEG XL library installed successfully!"
        echo "The unified_data_collector will now use JPEG XL compression."
    else
        echo "❌ Failed to install JPEG XL library"
        echo "The system will fall back to PNG compression"
        exit 1
    fi

else
    echo "❌ Unsupported system. Please install JPEG XL manually:"
    echo ""
    echo "Ubuntu/Debian: sudo apt install libjxl-dev"
    echo "macOS: brew install jpeg-xl"
    echo "Windows: Download from https://github.com/libjxl/libjxl/releases"
    echo ""
    echo "The system will fall back to PNG compression without JPEG XL"
    exit 1
fi

echo ""
echo "Next steps:"
echo "1. Rebuild the project: cd /home/avenblake/pilot_ws && colcon build"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Run unified_data_collector with JPEG XL compression enabled"

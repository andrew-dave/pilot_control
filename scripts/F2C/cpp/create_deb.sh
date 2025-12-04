#!/bin/bash
# Create Debian package for F2C Coverage Planner

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
PACKAGE_NAME="f2c-coverage-planner"
VERSION="1.0.0"
ARCH="amd64"
DEB_DIR="${BUILD_DIR}/${PACKAGE_NAME}_${VERSION}_${ARCH}"
DEB_FILE="${BUILD_DIR}/${PACKAGE_NAME}_${VERSION}_${ARCH}.deb"

echo "=== Creating Debian Package for F2C Coverage Planner ==="

# Check if executable exists
if [ ! -f "${BUILD_DIR}/f2c_coverage_planner" ]; then
    echo "Error: Executable not found at ${BUILD_DIR}/f2c_coverage_planner"
    echo "Please build the application first: ./build.sh"
    exit 1
fi

# Clean previous package
rm -rf "$DEB_DIR"
rm -f "$DEB_FILE"

# Create Debian package directory structure
echo "Creating package structure..."
mkdir -p "$DEB_DIR/DEBIAN"
mkdir -p "$DEB_DIR/usr/bin"
mkdir -p "$DEB_DIR/usr/lib/f2c-coverage-planner"
mkdir -p "$DEB_DIR/usr/share/applications"
mkdir -p "$DEB_DIR/usr/share/icons/hicolor/256x256/apps"
mkdir -p "$DEB_DIR/usr/share/doc/f2c-coverage-planner"
mkdir -p "$DEB_DIR/usr/share/f2c-coverage-planner"

# Create control file
echo "Creating control file..."
cat > "$DEB_DIR/DEBIAN/control" << EOF
Package: f2c-coverage-planner
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Depends: libqt5core5a (>= 5.9.5), libqt5widgets5 (>= 5.9.5), libqt5gui5 (>= 5.9.5), libc6 (>= 2.27), libstdc++6 (>= 6.0), libgcc-s1 (>= 3.0), ros-humble-rclcpp, ros-humble-std-msgs
Maintainer: Your Name <your.email@example.com>
Description: F2C Coverage Path Planning GUI with ROS2 Integration
 Fields2Cover Coverage Path Planning GUI application with ROS2 waypoint publishing.
 This application provides a graphical interface for planning coverage paths
 using the Fields2Cover library and can directly publish waypoints to ROS2
 pose controllers for autonomous robot navigation.
EOF

# Create postinst script
echo "Creating postinst script..."
cat > "$DEB_DIR/DEBIAN/postinst" << 'EOF'
#!/bin/bash
set -e

# Update desktop database
if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database /usr/share/applications >/dev/null 2>&1 || true
fi

# Update icon cache
if command -v gtk-update-icon-cache >/dev/null 2>&1; then
    gtk-update-icon-cache -f -t /usr/share/icons/hicolor >/dev/null 2>&1 || true
fi

# Set executable permissions
chmod +x /usr/bin/f2c_coverage_planner
chmod +x /usr/bin/f2c_coverage_planner_launcher

echo "F2C Coverage Planner has been installed successfully!"
echo "You can find it in your applications menu or run 'f2c_coverage_planner_launcher' from the terminal."
EOF
chmod +x "$DEB_DIR/DEBIAN/postinst"

# Create prerm script
echo "Creating prerm script..."
cat > "$DEB_DIR/DEBIAN/prerm" << 'EOF'
#!/bin/bash
set -e

# Remove desktop database entry
if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database /usr/share/applications >/dev/null 2>&1 || true
fi
EOF
chmod +x "$DEB_DIR/DEBIAN/prerm"

# Copy executable
echo "Copying executable..."
cp "${BUILD_DIR}/f2c_coverage_planner" "$DEB_DIR/usr/bin/"

# Create launcher script
echo "Creating launcher script..."
cat > "$DEB_DIR/usr/bin/f2c_coverage_planner_launcher" << 'EOF'
#!/bin/bash
# F2C Coverage Planner Launcher with ROS2 Support

# Source ROS2 environment (try different distributions)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    ROS_DISTRO="humble"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    ROS_DISTRO="foxy"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    ROS_DISTRO="galactic"
elif [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    ROS_DISTRO="noetic"
else
    echo "Warning: No ROS2 installation found. F2C waypoint publishing will not work."
    echo "Please install ROS2 (Humble, Foxy, or Galactic recommended) to enable waypoint publishing features."
fi

# Set basic environment
export DISPLAY=${DISPLAY:-:0}

# Determine CycloneDDS config (user-agnostic)
# Read saved profile from QSettings config file if it exists
SETTINGS_FILE="$HOME/.config/PilotControl/F2CCoveragePlanner.conf"
DDS_PROFILE="rf"  # Default to RF

if [ -f "$SETTINGS_FILE" ]; then
    # Extract dds_profile value from INI-style config
    SAVED_PROFILE=$(grep -E "^dds_profile=" "$SETTINGS_FILE" 2>/dev/null | cut -d'=' -f2)
    if [ -n "$SAVED_PROFILE" ]; then
        DDS_PROFILE="$SAVED_PROFILE"
    fi
fi

# Select appropriate config file based on profile
if [ "$DDS_PROFILE" = "wifi" ]; then
    DDS_CONFIG="$HOME/wifi_cyclonedds.xml"
else
    DDS_CONFIG="$HOME/rf_cyclonedds.xml"
fi

# Set CYCLONEDDS_URI if config file exists, otherwise let CycloneDDS auto-discover
if [ -f "$DDS_CONFIG" ]; then
    export CYCLONEDDS_URI="$DDS_CONFIG"
    echo "Using CycloneDDS config: $DDS_CONFIG"
else
    echo "Warning: DDS config not found at $DDS_CONFIG"
    echo "CycloneDDS will use auto-discovery (may not work on all networks)"
fi

# Add bundled library path first (highest priority)
export LD_LIBRARY_PATH="/usr/lib/f2c-coverage-planner:$LD_LIBRARY_PATH"

# Add system ROS paths if available (fallback)
if [ -n "$ROS_DISTRO" ] && [ -d "/opt/ros/$ROS_DISTRO/lib" ]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/ros/$ROS_DISTRO/lib:/opt/ros/$ROS_DISTRO/lib/x86_64-linux-gnu:/opt/ros/$ROS_DISTRO/opt/ortools_vendor/lib"
fi

# Add other common library paths
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"

# Set Qt platform plugin path - try system Qt first, fallback to bundled
if [ -d "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms" ]; then
    export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms:$QT_QPA_PLATFORM_PLUGIN_PATH"
elif [ -d "/usr/lib/qt5/plugins/platforms" ]; then
    export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/qt5/plugins/platforms:$QT_QPA_PLATFORM_PLUGIN_PATH"
else
    # Fallback to bundled plugins
    export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/f2c-coverage-planner/qt5/plugins/platforms:$QT_QPA_PLATFORM_PLUGIN_PATH"
fi

# Set Qt to use xcb platform (standard Linux desktop)
export QT_QPA_PLATFORM=xcb

# Launch the application
exec "/usr/bin/f2c_coverage_planner" "$@"
EOF
chmod +x "$DEB_DIR/usr/bin/f2c_coverage_planner_launcher"

# Copy required Qt libraries (only the ones not commonly available)
echo "Copying required libraries..."
QT_LIBS=$(ldd "${BUILD_DIR}/f2c_coverage_planner" | grep -E "(Qt5Core|Qt5Widgets|Qt5Gui)" | awk '{print $3}')
for lib in $QT_LIBS; do
    if [ -f "$lib" ]; then
        # Only copy if it's not in standard system locations
        if [[ "$lib" == /usr/local/* ]] || [[ "$lib" == /opt/* ]]; then
            cp "$lib" "$DEB_DIR/usr/lib/f2c-coverage-planner/"
        fi
    fi
done

# Note: Using system Qt platform plugins instead of bundling them
# This avoids conflicts and reduces package size
# The launcher script will find and use system Qt plugins

# Copy Fields2Cover and related libraries (required for functionality)
echo "Copying Fields2Cover libraries..."
if [ -d "/home/avenblake/pilot_ws/install/fields2cover/lib" ]; then
    cp -r /home/avenblake/pilot_ws/install/fields2cover/lib/* "$DEB_DIR/usr/lib/f2c-coverage-planner/" 2>/dev/null || true
fi

# Copy OR-Tools library
echo "Copying OR-Tools library..."
if [ -f "/opt/ros/humble/opt/ortools_vendor/lib/libortools.so.9" ]; then
    cp "/opt/ros/humble/opt/ortools_vendor/lib/libortools.so.9" "$DEB_DIR/usr/lib/f2c-coverage-planner/"
fi

# Copy other required libraries (only non-standard ones)
REQUIRED_LIBS=$(ldd "${BUILD_DIR}/f2c_coverage_planner" | grep -v "=>" | grep -v "linux-vdso" | grep -v "ld-linux" | awk '{print $1}')
for lib in $REQUIRED_LIBS; do
    LIB_PATH=$(ldd "${BUILD_DIR}/f2c_coverage_planner" | grep "$lib" | awk '{print $3}')
    if [ -f "$LIB_PATH" ] && [ "$LIB_PATH" != "" ]; then
        # Only copy non-standard libraries
        if [[ "$LIB_PATH" == /usr/local/* ]] || [[ "$LIB_PATH" == /opt/* ]] || [[ "$LIB_PATH" == /home/avenblake/pilot_ws/* ]]; then
            cp "$LIB_PATH" "$DEB_DIR/usr/lib/f2c-coverage-planner/"
        fi
    fi
done

# Create desktop file
echo "Creating desktop entry..."
cat > "$DEB_DIR/usr/share/applications/f2c_coverage_planner.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=F2C Coverage Planner
Comment=Fields2Cover Coverage Path Planning GUI
Exec=/usr/bin/f2c_coverage_planner_launcher
Icon=f2c_coverage_planner
Terminal=false
Categories=Development;Engineering;
StartupWMClass=f2c_coverage_planner
EOF
chmod +x "$DEB_DIR/usr/share/applications/f2c_coverage_planner.desktop"

# Create icon
echo "Creating application icon..."
convert -size 256x256 xc:"#4A90E2" -fill white -pointsize 72 -gravity center -annotate +0+0 "F2C" "$DEB_DIR/usr/share/icons/hicolor/256x256/apps/f2c_coverage_planner.png" 2>/dev/null || {
    echo "ImageMagick not found, creating simple icon manually..."
    cat > "$DEB_DIR/usr/share/icons/hicolor/256x256/apps/f2c_coverage_planner.svg" << 'EOF'
<svg width="256" height="256" xmlns="http://www.w3.org/2000/svg">
  <rect width="256" height="256" fill="#4A90E2"/>
  <text x="128" y="140" font-family="Arial" font-size="72" fill="white" text-anchor="middle">F2C</text>
</svg>
EOF
}

# Create copyright file
echo "Creating copyright file..."
cat > "$DEB_DIR/usr/share/doc/f2c-coverage-planner/copyright" << EOF
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: f2c-coverage-planner
Upstream-Contact: Your Name <your.email@example.com>
Source: https://github.com/your-repo/f2c-coverage-planner

Files: *
Copyright: $(date +%Y) Your Name <your.email@example.com>
License: GPL-3+
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 .
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 .
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
EOF

# Create changelog
echo "Creating changelog..."
cat > "$DEB_DIR/usr/share/doc/f2c-coverage-planner/changelog.Debian" << EOF
f2c-coverage-planner ($VERSION) unstable; urgency=medium

  * Initial release

 -- Your Name <your.email@example.com>  $(date -R)
EOF
gzip "$DEB_DIR/usr/share/doc/f2c-coverage-planner/changelog.Debian"

# Set permissions
echo "Setting permissions..."
find "$DEB_DIR" -type d -exec chmod 755 {} \;
find "$DEB_DIR" -type f -exec chmod 644 {} \;
chmod +x "$DEB_DIR/usr/bin/"*
chmod 755 "$DEB_DIR/DEBIAN/postinst"
chmod 755 "$DEB_DIR/DEBIAN/prerm"

# Build the package
echo "Building Debian package..."
dpkg-deb --build "$DEB_DIR" "$DEB_FILE"

echo "=== Debian Package Created Successfully ==="
echo "Package: $DEB_FILE"
echo "Size: $(du -h "$DEB_FILE" | cut -f1)"
echo ""
echo "To install:"
echo "  sudo dpkg -i $DEB_FILE"
echo "  sudo apt-get install -f  # Install any missing dependencies"
echo ""
echo "To test installation:"
echo "  f2c_coverage_planner_launcher"
echo ""
echo "Package contents:"
dpkg-deb -c "$DEB_FILE" | head -10

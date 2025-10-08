#!/bin/bash
# GPR Scan Controller - System Verification Script
# Run this after launching robot_complete.launch.py

echo "=========================================="
echo "GPR Scan Controller - System Check"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS not sourced${NC}"
    echo "   Run: source /home/raj/BDR/pilot_ws/install/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓ ROS sourced ($ROS_DISTRO)${NC}"
fi

echo ""
echo "Checking nodes..."
echo "===================="

# Check GPR scan controller node
if ros2 node list 2>/dev/null | grep -q "gpr_scan_controller"; then
    echo -e "${GREEN}✓ gpr_scan_controller node is running${NC}"
else
    echo -e "${RED}❌ gpr_scan_controller node NOT running${NC}"
    echo "   Expected: /gpr_scan_controller"
fi

# Check GPR serial bridge
if ros2 node list 2>/dev/null | grep -q "gpr_serial_bridge"; then
    echo -e "${GREEN}✓ gpr_serial_bridge node is running${NC}"
else
    echo -e "${YELLOW}⚠️  gpr_serial_bridge node NOT running${NC}"
    echo "   L/K keys and Arduino control won't work"
fi

# Check differential drive controller
if ros2 node list 2>/dev/null | grep -q "diff_drive_controller"; then
    echo -e "${GREEN}✓ diff_drive_controller node is running${NC}"
else
    echo -e "${RED}❌ diff_drive_controller node NOT running${NC}"
fi

echo ""
echo "Checking services..."
echo "===================="

# Check GPR scan toggle service
if ros2 service list 2>/dev/null | grep -q "/gpr_scan/toggle"; then
    echo -e "${GREEN}✓ /gpr_scan/toggle service available${NC}"
else
    echo -e "${RED}❌ /gpr_scan/toggle service NOT available${NC}"
fi

# Check Arduino services
if ros2 service list 2>/dev/null | grep -q "/gpr_line_start"; then
    echo -e "${GREEN}✓ /gpr_line_start service available${NC}"
else
    echo -e "${YELLOW}⚠️  /gpr_line_start service NOT available${NC}"
fi

if ros2 service list 2>/dev/null | grep -q "/gpr_line_stop"; then
    echo -e "${GREEN}✓ /gpr_line_stop service available${NC}"
else
    echo -e "${YELLOW}⚠️  /gpr_line_stop service NOT available${NC}"
fi

echo ""
echo "Checking topics..."
echo "===================="

# Check Fast-LIO odometry
if ros2 topic list 2>/dev/null | grep -q "/Odometry"; then
    echo -e "${GREEN}✓ /Odometry topic exists${NC}"
    
    # Check if publishing
    ODOM_HZ=$(timeout 3 ros2 topic hz /Odometry 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ -n "$ODOM_HZ" ]; then
        echo -e "${GREEN}  └─ Publishing at ${ODOM_HZ} Hz${NC}"
    else
        echo -e "${YELLOW}  └─ Not publishing (Fast-LIO may not be active)${NC}"
    fi
else
    echo -e "${RED}❌ /Odometry topic NOT found${NC}"
    echo "   Fast-LIO is not running"
fi

# Check GPR motor control
if ros2 topic list 2>/dev/null | grep -q "/gpr/control_message"; then
    echo -e "${GREEN}✓ /gpr/control_message topic exists${NC}"
else
    echo -e "${RED}❌ /gpr/control_message topic NOT found${NC}"
fi

# Check GPR motor status
if ros2 topic list 2>/dev/null | grep -q "/gpr/controller_status"; then
    echo -e "${GREEN}✓ /gpr/controller_status topic exists${NC}"
else
    echo -e "${RED}❌ /gpr/controller_status topic NOT found${NC}"
fi

echo ""
echo "Checking filesystem..."
echo "===================="

# Check log directory
LOG_DIR="$HOME/gpr_scans"
if [ -d "$LOG_DIR" ]; then
    echo -e "${GREEN}✓ Log directory exists: $LOG_DIR${NC}"
    
    # Check if writable
    if [ -w "$LOG_DIR" ]; then
        echo -e "${GREEN}  └─ Directory is writable${NC}"
    else
        echo -e "${RED}  └─ Directory is NOT writable${NC}"
    fi
    
    # Count existing logs
    LOG_COUNT=$(ls -1 "$LOG_DIR"/gpr_scan_*.csv 2>/dev/null | wc -l)
    if [ "$LOG_COUNT" -gt 0 ]; then
        echo -e "${GREEN}  └─ Found $LOG_COUNT existing scan log(s)${NC}"
        echo "     Latest: $(ls -t "$LOG_DIR"/gpr_scan_*.csv 2>/dev/null | head -1)"
    else
        echo "  └─ No existing scan logs"
    fi
else
    echo -e "${YELLOW}⚠️  Log directory does not exist: $LOG_DIR${NC}"
    echo "   Will be created automatically on first scan"
fi

echo ""
echo "System Summary"
echo "===================="

# Count issues
CRITICAL=0
WARNINGS=0

# Check critical components
ros2 node list 2>/dev/null | grep -q "gpr_scan_controller" || ((CRITICAL++))
ros2 service list 2>/dev/null | grep -q "/gpr_scan/toggle" || ((CRITICAL++))
ros2 topic list 2>/dev/null | grep -q "/gpr/control_message" || ((CRITICAL++))

# Check warnings
ros2 node list 2>/dev/null | grep -q "gpr_serial_bridge" || ((WARNINGS++))
ros2 topic list 2>/dev/null | grep -q "/Odometry" || ((WARNINGS++))

if [ "$CRITICAL" -eq 0 ] && [ "$WARNINGS" -eq 0 ]; then
    echo -e "${GREEN}✅ All systems operational!${NC}"
    echo ""
    echo "Ready to scan:"
    echo "  1. Launch teleop on laptop"
    echo "  2. Press E to arm motors"
    echo "  3. Press G to start GPR scan"
    echo "  4. Press G again to stop"
elif [ "$CRITICAL" -eq 0 ]; then
    echo -e "${YELLOW}⚠️  System functional with $WARNINGS warning(s)${NC}"
    echo ""
    echo "You can proceed, but some features may not work."
else
    echo -e "${RED}❌ System has $CRITICAL critical issue(s)${NC}"
    echo ""
    echo "Fix critical issues before scanning."
fi

echo ""
echo "=========================================="
echo "Test Commands"
echo "=========================================="
echo ""
echo "# Test service manually:"
echo "ros2 service call /gpr_scan/toggle std_srvs/srv/Trigger"
echo ""
echo "# Monitor Fast-LIO:"
echo "ros2 topic hz /Odometry"
echo ""
echo "# Check GPR motor status:"
echo "ros2 topic echo /gpr/controller_status"
echo ""
echo "# View logs:"
echo "ls -lht ~/gpr_scans/"
echo ""

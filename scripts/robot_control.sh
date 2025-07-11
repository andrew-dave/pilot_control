#!/bin/bash

# Robot Control Script
# This script handles the complete robot workflow including map saving and shutdown

set -e  # Exit on any error

# Configuration
ROBOT_IP="172.16.14.113"  # Change this to your robot's IP
MAP_SAVE_DIR="/home/roofus/maps"
CAN_INTERFACE="can0"
CAN_BITRATE="250000"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to setup CAN interface
setup_can() {
    print_status "Setting up CAN interface..."
    sudo ip link set $CAN_INTERFACE up type can bitrate $CAN_BITRATE
    sudo ip link set $CAN_INTERFACE txqueuelen 1000
    print_success "CAN interface $CAN_INTERFACE is up"
}

# Function to shutdown CAN interface
shutdown_can() {
    print_status "Shutting down CAN interface..."
    sudo ip link set $CAN_INTERFACE down
    print_success "CAN interface $CAN_INTERFACE is down"
}

# Function to save map and shutdown
save_map_and_shutdown() {
    print_status "Saving map and shutting down..."
    
    # Create timestamp for map filename
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    MAP_FILENAME="robot_map_${TIMESTAMP}"
    
    # Save current map using PCD saver
    print_status "Triggering map save..."
    ros2 service call /pcd_saver/save_map std_srvs/srv/Trigger
    
    # Wait a moment for the save to complete
    sleep 2
    
    # Shutdown Fast-LIO and Livox nodes
    print_status "Shutting down mapping nodes..."
    ros2 node kill /laserMapping
    ros2 node kill /livox_lidar_publisher
    
    # Wait for nodes to shutdown
    sleep 3
    
    # Shutdown CAN interface
    shutdown_can
    
    print_success "Map saved and system shutdown complete"
    print_status "Map files saved to: $MAP_SAVE_DIR"
}

# Function to start the complete system
start_system() {
    print_status "Starting complete robot system..."
    
    # Setup CAN interface
    setup_can
    
    # Start the complete launch file
    ros2 launch pilot_control robot_complete.launch.py &
    LAUNCH_PID=$!
    
    print_success "Robot system started with PID: $LAUNCH_PID"
    print_status "Use 'M' key in laptop teleop to save map and shutdown"
    
    # Wait for the launch process
    wait $LAUNCH_PID
}

# Function to start laptop teleop
start_laptop_teleop() {
    print_status "Starting laptop teleop..."
    ros2 launch pilot_control laptop_teleop.launch.py
}

# Function to show usage
show_usage() {
    echo "Robot Control Script"
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  start-robot     - Start the complete robot system on robot"
    echo "  start-laptop    - Start laptop teleop (run on laptop)"
    echo "  save-map        - Save map and shutdown mapping nodes"
    echo "  setup-can       - Setup CAN interface only"
    echo "  shutdown-can    - Shutdown CAN interface only"
    echo "  help            - Show this help message"
    echo ""
    echo "Examples:"
    echo "  # On robot:"
    echo "  $0 start-robot"
    echo ""
    echo "  # On laptop:"
    echo "  $0 start-laptop"
    echo ""
    echo "  # Save map and shutdown (from laptop):"
    echo "  ssh robot@$ROBOT_IP '$0 save-map'"
}

# Main script logic
case "${1:-}" in
    "start-robot")
        start_system
        ;;
    "start-laptop")
        start_laptop_teleop
        ;;
    "save-map")
        save_map_and_shutdown
        ;;
    "setup-can")
        setup_can
        ;;
    "shutdown-can")
        shutdown_can
        ;;
    "help"|"-h"|"--help")
        show_usage
        ;;
    *)
        print_error "Unknown command: $1"
        show_usage
        exit 1
        ;;
esac 
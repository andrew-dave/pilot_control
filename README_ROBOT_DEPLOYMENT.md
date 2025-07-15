# Robot Deployment Guide

This guide provides a complete workflow for deploying and controlling the robot with mapping capabilities.

## 🚀 Quick Start

### **Single Command Robot Launch:**
```bash
# On robot (SSH into robot)
ros2 launch pilot_control robot_complete.launch.py
```

### **Laptop Teleop:**
```bash
# On laptop
ros2 launch pilot_control laptop_teleop.launch.py
```

## 📋 Complete Workflow

### **Step 1: Robot Setup**
```bash
# SSH into robot
ssh roofus@172.16.14.113  # Replace with your robot's IP

# Navigate to workspace
cd ~/pilot_ws

# Source the workspace
source install/setup.bash

# Start complete system (includes CAN setup, Fast-LIO2, Livox, Foxglove)
ros2 launch pilot_control robot_complete.launch.py
```

### **Step 2: Laptop Control**
```bash
# On your laptop
cd ~/pilot_ws
source install/setup.bash

# Start laptop teleop
ros2 launch pilot_control laptop_teleop.launch.py
```

### **Step 3: Control the Robot**
- **WASD** - Move robot
- **E** - Arm motors
- **Q** - Disarm motors  
- **M** - Save map, shutdown mapping, copy to laptop, and process

## 🔧 What's Included

### **Robot Complete Launch (`robot_complete.launch.py`):**
- ✅ **CAN Interface Setup** (250kbps bitrate)
- ✅ **Robot Control** (ODrive motors, differential drive)
- ✅ **Fast-LIO2 Mapping** (MID360 configuration)
- ✅ **Livox ROS Driver** (MID360)
- ✅ **Foxglove Bridge** (port 8765)
- ✅ **Raw Map Saver** (saves unprocessed maps to /tmp/robot_maps)
- ✅ **Shutdown Service** (for remote shutdown)

### **Laptop Teleop (`laptop_teleop.launch.py`):**
- ✅ **Robot Movement** (WASD controls)
- ✅ **Motor Control** (E=Arm, Q=Disarm)
- ✅ **Map Processing** (PCD processor for cleaning raw maps)
- ✅ **M Key Workflow** (save → shutdown → copy → process)

## 🗺️ Map Saving & Processing Workflow

### **When 'M' is Pressed:**
1. **Save Raw Map** - Saves unprocessed Fast-LIO2 map to `/tmp/robot_maps/` on robot
2. **Shutdown Mapping** - Kills Fast-LIO2, Livox, and Foxglove nodes
3. **Copy to Laptop** - Copies raw map from robot to `/home/avenblake/robot_maps/`
4. **Process Map** - Applies filtering, cleaning, and rotation correction on laptop

### **Map File Locations:**
- **Robot Raw Maps**: `/tmp/robot_maps/raw_map_YYYYMMDD_HHMMSS_mmm.pcd`
- **Laptop Processed Maps**: `/home/avenblake/robot_maps/processed_map_YYYYMMDD_HHMMSS_mmm.pcd`

## ⚙️ Configuration

### **Robot Parameters:**
```bash
# Custom robot configuration
ros2 launch pilot_control robot_complete.launch.py \
  --ros-args \
  -p wheel_radius:=0.063 \
  -p wheel_base:=0.32 \
  -p can_interface:=can0 \
  -p can_bitrate:=250000 \
  -p velocity_multiplier:=0.8 \
  -p turn_speed_multiplier:=0.4
```

### **PCD Processor Parameters:**
```bash
# Custom map processing
ros2 launch pilot_control laptop_teleop.launch.py \
  --ros-args \
  -p processing_mode:=high_quality \
  -p voxel_size:=0.05 \
  -p remove_outliers:=true \
  -p outlier_std_dev:=2.0 \
  -p apply_rotation_correction:=true \
  -p rotation_angle:=-0.5230
```

## 🔍 Monitoring

### **Check System Status:**
```bash
# List all nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor robot position
ros2 topic echo /odom

# Check map generation
ros2 topic echo /Laser_map

# Monitor services
ros2 service list
```

### **Foxglove Studio:**
- Open browser: `http://robot_ip:8765`
- Or use Foxglove Studio desktop app
- Connect to: `ws://robot_ip:8765`

## 🛠️ Troubleshooting

### **CAN Interface Issues:**
```bash
# Check CAN interface
ip link show can0

# Restart CAN interface
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 250000

# Check CAN messages
candump can0
```

### **Motor Issues:**
```bash
# Check ODrive nodes
ros2 node list | grep odrive

# Check motor states
ros2 topic echo /left/axis_state
ros2 topic echo /right/axis_state
```

### **Mapping Issues:**
```bash
# Check Fast-LIO2
ros2 node list | grep laserMapping

# Check Livox driver
ros2 node list | grep livox

# Check point cloud topics
ros2 topic list | grep cloud
```

### **Map Processing Issues:**
```bash
# Check raw map saver
ros2 node list | grep raw_map_saver

# Check PCD processor
ros2 node list | grep pcd_processor

# Check services
ros2 service list | grep -E "(save|process|shutdown)"
```

## 🚨 Emergency Procedures

### **Emergency Stop:**
```bash
# Stop all ROS2 nodes
pkill -f ros2

# Or stop specific components
ros2 node kill /laserMapping
ros2 node kill /diff_drive_controller
```

### **Manual CAN Shutdown:**
```bash
sudo ip link set can0 down
```

### **Manual Map Operations:**
```bash
# Save raw map manually
ros2 service call /save_raw_map std_srvs/srv/Trigger

# Process map manually
ros2 service call /process_and_save_map std_srvs/srv/Trigger

# Shutdown mapping manually
ros2 service call /shutdown_mapping std_srvs/srv/Trigger
```

## 📁 File Structure

```
pilot_ws/
├── src/roofus_pilot1/
│   ├── launch/
│   │   ├── robot_complete.launch.py    # Complete robot system
│   │   ├── laptop_teleop.launch.py     # Laptop control & processing
│   │   └── diff_drive_only.launch.py   # Robot control only
│   ├── src/
│   │   ├── diff_drive_controller.cpp   # Robot motor control
│   │   ├── host_teleop.cpp             # Laptop teleop with M key
│   │   ├── raw_map_saver.cpp           # Robot-side raw map saving
│   │   ├── pcd_processor.cpp           # Laptop-side map processing
│   │   └── shutdown_service.cpp        # Remote shutdown service
│   └── scripts/
│       └── copy_latest_map.sh          # Copy raw maps from robot to laptop
```

## 🎯 Key Features

- **Single Command Launch** - Everything starts with one command
- **Automatic CAN Setup** - No manual CAN configuration needed
- **Integrated Mapping** - Fast-LIO2 + Livox + raw map saving
- **Laptop Control** - Full teleop with map processing
- **Safety Features** - Reduced speed multipliers, emergency procedures
- **Two-Phase Workflow** - Mapping at full performance, processing on laptop

## 🔄 Workflow Summary

1. **SSH into robot** → `ros2 launch pilot_control robot_complete.launch.py`
2. **Start laptop teleop** → `ros2 launch pilot_control laptop_teleop.launch.py`
3. **Control robot** → WASD for movement, E/Q for motors
4. **Save & Process** → Press 'M' key for complete workflow
5. **Access maps** → Raw maps on robot, processed maps on laptop

This system provides a complete, integrated solution for robot deployment with optimized mapping and processing capabilities! 
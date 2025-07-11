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
ssh robot@192.168.1.100  # Replace with your robot's IP

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
- **M** - Save map and shutdown mapping system

## 🔧 What's Included

### **Robot Complete Launch (`robot_complete.launch.py`):**
- ✅ **CAN Interface Setup** (250kbps bitrate)
- ✅ **Robot Control** (ODrive motors, differential drive)
- ✅ **Fast-LIO2 Mapping** (MID360 configuration)
- ✅ **Livox ROS Driver** (MID360)
- ✅ **Foxglove Bridge** (port 8765)
- ✅ **Map Relay** (QoS bridge)
- ✅ **PCD Saver** (filtered point clouds)

### **Laptop Teleop:**
- ✅ **Robot Movement** (WASD controls)
- ✅ **Motor Control** (E=Arm, Q=Disarm)
- ✅ **Map Saving** (M key triggers save + shutdown)

## 🗺️ Map Saving Features

### **Automatic Saving:**
- Saves filtered point clouds every 60 seconds
- Height filtering: 0.1m to 2.0m
- Outlier removal for clean maps
- Voxel grid filtering (5cm resolution)

### **Manual Saving (M Key):**
- Triggers immediate map save
- Shuts down Fast-LIO2 and Livox nodes
- Saves to `/home/robot/maps/` with timestamp

### **Map File Format:**
```
/home/robot/maps/laser_map_YYYYMMDD_HHMMSS_mmm.pcd
```

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
  -p turn_speed_multiplier:=0.4 \
  -p save_directory:=/home/robot/maps
```

### **PCD Saver Parameters:**
```bash
# Custom PCD saving
ros2 launch pilot_control robot_complete.launch.py \
  --ros-args \
  -p enable_pcd_saver:=true \
  -p save_interval:=60 \
  -p max_height:=2.0 \
  -p min_height:=0.1 \
  -p remove_outliers:=true \
  -p voxel_size:=0.05
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
ros2 topic echo /map

# Monitor point clouds
ros2 topic echo /Laser_map
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

### **Performance Issues:**
```bash
# Monitor CPU usage
htop

# Check disk space
df -h /home/robot/maps

# Monitor memory
free -h
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

### **Save Map Manually:**
```bash
# Trigger map save via service
ros2 service call /pcd_saver/save_map std_srvs/srv/Trigger
```

## 📁 File Structure

```
pilot_ws/
├── src/roofus_pilot1/
│   ├── launch/
│   │   ├── robot_complete.launch.py    # Complete robot system
│   │   ├── laptop_teleop.launch.py     # Laptop control
│   │   └── robot.launch.py             # Basic robot control
│   ├── src/
│   │   ├── pcd_saver.cpp               # Map saving with filtering
│   │   ├── host_teleop.cpp             # Laptop teleop with M key
│   │   └── map_relay.cpp               # QoS bridge
│   └── scripts/
│       └── robot_control.sh            # Management script
```

## 🎯 Key Features

- **Single Command Launch** - Everything starts with one command
- **Automatic CAN Setup** - No manual CAN configuration needed
- **Integrated Mapping** - Fast-LIO2 + Livox + PCD saving
- **Laptop Control** - Full teleop with map saving
- **Safety Features** - Reduced speed multipliers, emergency procedures
- **Map Management** - Automatic and manual saving with filtering

## 🔄 Workflow Summary

1. **SSH into robot** → `ros2 launch pilot_control robot_complete.launch.py`
2. **Start laptop teleop** → `ros2 launch pilot_control laptop_teleop.launch.py`
3. **Control robot** → WASD for movement, E/Q for motors
4. **Save map** → Press 'M' key to save and shutdown mapping
5. **Access maps** → Files saved to `/home/robot/maps/`

This system provides a complete, integrated solution for robot deployment with mapping capabilities! 
# Manual Map Saving System

## Overview

The system now supports manual map saving with custom filenames that include the date and mapping duration. When you press 'M', the map is saved once with a descriptive filename and then the mapping system is completely shut down, leaving only robot control and teleop active.

## New Features

### **Manual Save Only**
- Auto-save is disabled by default
- Maps are only saved when 'M' key is pressed
- Custom filename format: `map_July_25_duration_2h_30m.pcd`

### **Complete Shutdown**
- When 'M' is pressed, the map is saved once
- All mapping nodes are shut down (Fast-LIO2, Livox, Foxglove)
- Only robot control and teleop remain active
- Auto-save timer is disabled after manual save

## Usage

### **1. Start the System**
```bash
# On robot
ros2 launch pilot_control robot_complete.launch.py

# On laptop
ros2 launch pilot_control laptop_teleop.launch.py
```

### **2. Map and Control**
- Use WASD to move the robot
- Use E/Q to arm/disarm motors
- Map is built in real-time but not saved automatically

### **3. Save and Shutdown**
- Press 'M' key to save map once and shutdown mapping
- Map is saved with filename like: `map_July_25_duration_2h_30m.pcd`
- All mapping nodes are shut down
- Only robot control and teleop remain active

## Filename Format

The manual save generates filenames with the following format:
```
map_[Month]_[Day]_duration_[Hours]h_[Minutes]m.pcd
```

**Examples:**
- `map_July_25_duration_1h_45m.pcd`
- `map_December_15_duration_0h_30m.pcd`
- `map_January_3_duration_3h_20m.pcd`

## System Behavior

### **Before 'M' Key:**
- Robot control: ✅ Active
- Teleop: ✅ Active
- Fast-LIO2 mapping: ✅ Active
- Livox driver: ✅ Active
- Foxglove visualization: ✅ Active
- Auto-save: ❌ Disabled
- Manual save: ⏳ Waiting for 'M' key

### **After 'M' Key:**
- Robot control: ✅ Active
- Teleop: ✅ Active
- Fast-LIO2 mapping: ❌ Shutdown
- Livox driver: ❌ Shutdown
- Foxglove visualization: ❌ Shutdown
- Auto-save: ❌ Disabled
- Manual save: ✅ Completed once

## Services

### **Manual Save Service**
- **Service**: `/manual_save_map`
- **Type**: `std_srvs/srv/Trigger`
- **Function**: Saves map once with custom filename and disables auto-save

### **Shutdown Service**
- **Service**: `/shutdown_mapping`
- **Type**: `std_srvs/srv/Trigger`
- **Function**: Shuts down all mapping nodes

## Configuration

### **PCD Saver Parameters**
```yaml
auto_save: false                    # Disable automatic saving
save_interval: 60                   # Not used when auto_save is false
save_directory: '/home/avenblake/robot_maps'
apply_rotation_correction: true     # Apply 30° rotation correction
rotation_angle: -0.5230            # -30 degrees in radians
```

## File Locations

### **Saved Maps**
- **Location**: `/home/avenblake/robot_maps/`
- **Format**: PCD files with custom filenames
- **Example**: `map_July_25_duration_2h_30m.pcd`

## Troubleshooting

### **Map Not Saved**
- Check if PCD saver node is running: `ros2 node list | grep pcd_saver`
- Check service availability: `ros2 service list | grep manual_save_map`
- Check logs for error messages

### **Shutdown Not Working**
- Check if shutdown service is available: `ros2 service list | grep shutdown_mapping`
- Verify robot launch is running with shutdown service

### **Custom Filename Issues**
- Filename includes current date and time
- Duration is calculated from system uptime
- Check file permissions in save directory

## Commands

### **Manual Save via Command Line**
```bash
# Trigger manual save
ros2 service call /manual_save_map std_srvs/srv/Trigger

# Check saved files
ls -la ~/robot_maps/
```

### **Check Service Status**
```bash
# List available services
ros2 service list | grep -E "(save|shutdown)"

# Check node status
ros2 node list
```

## Integration with Existing System

The manual save system integrates seamlessly with the existing tilted LiDAR setup:

- **Rotation Correction**: Applied before filtering
- **Custom Filenames**: Include date and duration
- **Complete Shutdown**: Only robot control remains active
- **Visualization**: Foxglove uses `foot_init` frame for corrected view

## Next Steps

1. Test the manual save functionality
2. Verify custom filenames are generated correctly
3. Confirm complete shutdown of mapping nodes
4. Check that robot control remains active after shutdown 
# Tilted LiDAR Setup Documentation

## Overview

This system uses a LiDAR sensor mounted at a 30-degree angle facing forward. To correct the visualization and mapping, static transforms are used to adjust the coordinate frames.

## Hardware Configuration

- **LiDAR**: MID360 mounted at 30° pitch (facing forward)
- **Mounting**: Tilted upward by 30 degrees from horizontal
- **Purpose**: Provides better ground coverage and obstacle detection

## Transform Setup

### Static Transform Configuration

The system uses two static transforms to correct the tilted LiDAR setup:

```bash
# Transform from body to foot (corrects the 30-degree tilt)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.5230 0 body foot

# Transform from camera_init to foot_init (corrects the global frame for visualization)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.5230 0 camera_init foot_init
```

**Parameters for both transforms:**
- `x, y, z`: 0, 0, 0 (no translation)
- `roll, pitch, yaw`: 0, -0.5230, 0 (30° pitch correction)
- **body → foot**: Corrects the robot body frame
- **camera_init → foot_init**: Corrects the global mapping frame

### Frame Hierarchy

```
camera_init (Fast-LIO2 global frame)
    ↓ [static transform: 30° pitch correction]
foot_init (corrected visualization frame)
    ↓ [Fast-LIO2 mapping]
body (robot body frame)
    ↓ [static transform: 30° pitch correction]
foot (corrected body frame)
    ↓ [odometry]
base_link (robot base)
    ↓ [wheel odometry]
odom (odometry frame)
```

## Usage Instructions

### 1. Start the Robot System

```bash
# On robot
ros2 launch pilot_control robot_complete.launch.py
```

This automatically includes the static transform publisher.

### 2. Start Laptop Control

```bash
# On laptop
ros2 launch pilot_control laptop_teleop.launch.py
```

### 3. Visualize in Foxglove

1. Open Foxglove Studio
2. Connect to robot (port 8765)
3. **Important**: Set the frame to `foot_init` for corrected visualization
4. Add the `/Laser_map` topic for point cloud visualization

### 4. Verify Transforms

```bash
# Check if transforms are working
./src/roofus_pilot1/scripts/check_transforms.sh

# Or manually check
ros2 run tf2_echo camera_init foot_init
```

## Expected Results

### Transform Verification

Both transforms should show the same correction:
- **Translation**: (0, 0, 0)
- **Rotation**: (0, -0.5230, 0) radians = (0°, -30°, 0°)

**body → foot**: Corrects the robot body frame
**camera_init → foot_init**: Corrects the global mapping frame

### Visualization and Map Saving

- **Without correction**: Map appears tilted due to LiDAR mounting
- **With correction**: Map appears level and properly oriented
- **Frame selection**: Use `foot_init` frame in Foxglove for correct view
- **Saved maps**: PCD files are automatically rotated and saved in corrected orientation

## Technical Details

### Why This Works

1. **Fast-LIO2**: Publishes maps in `camera_init` frame (its global frame)
2. **Static Transform**: Corrects the global frame by applying inverse of LiDAR tilt
3. **Visualization**: Foxglove uses `foot_init` frame, which shows corrected map
4. **Map Saving**: PCD saver applies the same rotation correction to saved files

### Coordinate System

- **LiDAR Mounting**: 30° upward pitch from horizontal
- **Transform Correction**: -30° pitch (inverse of mounting angle)
- **Result**: Maps appear level and properly oriented

## Troubleshooting

### Common Issues

1. **Map still appears tilted**
   - Check if `foot_init` frame is selected in Foxglove
   - Verify transform is published: `ros2 run tf2_echo camera_init foot_init`

2. **Transform not found**
   - Ensure robot launch is running
   - Check launch file includes static transform publisher

3. **Poor mapping quality**
   - Verify LiDAR is properly mounted at 30°
   - Check LiDAR configuration in `MID360_config.json`

### Debug Commands

```bash
# List all available frames
ros2 run tf2_tools view_frames

# Check specific transforms
ros2 run tf2_echo body foot
ros2 run tf2_echo camera_init foot_init

# List all transforms
ros2 topic echo /tf

# Check if static transforms are published
ros2 node list | grep static_transform_publisher
```

## Integration with Existing System

The tilted LiDAR setup integrates seamlessly with the existing system:

- **Robot Control**: Unaffected (uses `base_link` and `odom` frames)
- **Mapping**: Fast-LIO2 works with tilted LiDAR
- **Visualization**: Corrected via `foot_init` frame
- **Map Saving**: PCD files are automatically rotated and saved in corrected coordinate system

## Files Modified

- `src/roofus_pilot1/launch/robot_complete.launch.py`: Added static transform publisher
- `src/roofus_pilot1/scripts/check_transforms.sh`: Transform verification script
- `src/roofus_pilot1/docs/TILTED_LIDAR_SETUP.md`: This documentation

## Next Steps

1. Test the system with the new transform setup
2. Verify visualization in Foxglove using `foot_init` frame
3. Check map quality and orientation
4. Adjust LiDAR mounting angle if needed (update transform accordingly) 
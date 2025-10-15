# Launcher Configuration Verification

## ROS2 Environment Setup

The launcher is configured to source the following files before running any ROS2 commands:

### 1. ROS2 Installation Setup
**Path**: `/opt/ros/jazzy/setup.bash`
- This sources the base ROS2 Jazzy installation

### 2. Workspace Setup
**Path**: `~/ros2_ws/install/setup.bash`
- This sources your workspace overlay with all custom packages
- Includes both `roboto_viz` and `roboto_diffbot` packages

## Command Sequence

### For Diffbot Launch:
```bash
source /opt/ros/jazzy/setup.bash && \
source ~/ros2_ws/install/setup.bash && \
ros2 launch roboto_diffbot launch_roboto.launch.py
```

### For GUI Launch:
```bash
source /opt/ros/jazzy/setup.bash && \
source ~/ros2_ws/install/setup.bash && \
ros2 launch roboto_viz gui_launch.py
```

## How It Works

1. **ROS2 Base Setup**: First sources the ROS2 installation to get core ROS2 commands and libraries
2. **Workspace Overlay**: Then sources your workspace setup which overlays your custom packages
3. **Launch Command**: Finally executes the ros2 launch command with the sourced environment

This ensures that:
- All ROS2 commands are available
- Your custom packages are discoverable
- The correct package versions from your workspace are used
- All dependencies are properly loaded

## Verification

You can manually verify the commands work by running them in a terminal:

```bash
# Test diffbot launch
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch roboto_diffbot launch_roboto.launch.py

# Test GUI launch
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch roboto_viz gui_launch.py
```

If these commands work in a terminal, they will work in the launcher.

## Troubleshooting

### Issue: "Package not found" error

**Cause**: Workspace setup file doesn't exist or packages not built

**Solution**:
```bash
cd ~/ros2_ws
colcon build --packages-select roboto_viz roboto_diffbot
source install/setup.bash
```

### Issue: "ROS_DISTRO not set" error

**Cause**: ROS2 base setup not sourced

**Solution**: Verify the ROS2 installation path is correct:
```bash
ls /opt/ros/jazzy/setup.bash
```

If the file doesn't exist, adjust the `ros2_setup` path in `launcher.py` to match your ROS2 installation.

### Issue: Tilde (~) not expanding

**Note**: The bash shell will automatically expand `~` to the home directory. If you encounter issues, you can use the full path instead:

Change in launcher.py:
```python
workspace_setup = "~/ros2_ws/install/setup.bash"
```

To:
```python
workspace_setup = "/home/rooteq/ros2_ws/install/setup.bash"
```

## Current Configuration Summary

✅ Sources ROS2 Jazzy installation
✅ Sources workspace overlay (`~/ros2_ws/install/setup.bash`)
✅ Launches diffbot package first
✅ Waits 2 seconds
✅ Launches GUI package
✅ Both processes have proper environment

The configuration is **correct and ready to use**.

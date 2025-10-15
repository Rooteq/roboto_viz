# Quick Start Guide

## What You Have Now

You have two applications:

1. **Launcher App** - A simple app with a big button to start the robot system
2. **Main GUI** - Your existing robot control interface with a new close button

## Step 1: Build the Modified ROS2 Package

```bash
cd /home/rooteq/ros2_ws
colcon build --packages-select roboto_viz
source install/setup.bash
```

This builds the modifications to the main GUI (close button functionality).

## Step 2: Test the Launcher Manually

```bash
cd /home/rooteq/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

You should see a window with a button labeled "Uruchom tryb jazdy".

## Step 3: Test the Full Workflow

1. **Click "Uruchom tryb jazdy"** in the launcher
   - Wait for both systems to start (may take 10-20 seconds)
   - The main GUI should appear

2. **Use the robot control GUI normally**
   - Connect to robot
   - Control the robot
   - Run plans, etc.

3. **Click the "✕ Zamknij" button** (top-right red button in the GUI)
   - GUI should close
   - Launcher should remain visible
   - Button should become enabled again

4. **Click "Uruchom tryb jazdy" again** to verify restart works

## Step 4: Set Up Autostart (Optional)

If everything works correctly, follow the instructions in [AUTOSTART_SETUP.md](AUTOSTART_SETUP.md) to configure the launcher to start automatically at boot.

**Recommended Method**: Use the systemd service method described in the autostart guide.

## Common Issues

### "The launcher button doesn't do anything"

**Check**:
- Are the ROS2 paths correct in `launcher.py`?
- Run the launcher from terminal to see error messages

**Fix**:
```bash
cd /home/rooteq/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
# Click the button and watch for errors in the terminal
```

### "The close button doesn't appear"

**Fix**: Rebuild and source the package:
```bash
cd /home/rooteq/ros2_ws
colcon build --packages-select roboto_viz --cmake-clean-cache
source install/setup.bash
```

### "Processes don't terminate when I close the GUI"

**Check**: Look for leftover processes
```bash
ps aux | grep ros2
```

**Fix**: Kill them manually if needed
```bash
pkill -f "ros2 launch"
```

## Testing Checklist

Before setting up autostart, verify:

- ✓ Launcher starts without errors
- ✓ Button launches both ROS2 packages
- ✓ GUI appears and works normally
- ✓ Close button is visible in GUI
- ✓ Close button terminates both processes
- ✓ Launcher remains visible after close
- ✓ Can restart system by clicking button again

## Next Steps

Once manual testing is successful:

1. Read [AUTOSTART_SETUP.md](AUTOSTART_SETUP.md) for autostart configuration
2. Choose systemd service method (recommended)
3. Create and enable the service
4. Reboot to test autostart
5. Check service status: `sudo systemctl status robot-launcher.service`

## Need More Info?

- **Detailed implementation**: See [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)
- **Autostart setup**: See [AUTOSTART_SETUP.md](AUTOSTART_SETUP.md)
- **Basic usage**: See [README.md](README.md)

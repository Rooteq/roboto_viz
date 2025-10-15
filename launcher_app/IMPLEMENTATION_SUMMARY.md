# Robot Control Launcher - Implementation Summary

## Overview

This implementation creates a two-tier application system:

1. **Launcher Application** (`launcher_app/`) - A standalone PyQt5 app that starts at boot
2. **Main GUI Application** (existing `roboto_viz` package) - The robot control interface with a new close button

## What Was Created

### 1. Launcher Application (`launcher_app/`)

**Location**: `/home/rooteq/ros2_ws/src/roboto_viz/launcher_app/`

**Files**:
- `launcher.py` - Main launcher application
- `README.md` - Basic usage instructions
- `AUTOSTART_SETUP.md` - Detailed autostart configuration guide
- `IMPLEMENTATION_SUMMARY.md` - This file

**Features**:
- Simple button interface: "Uruchom tryb jazdy" (Start driving mode)
- Launches two ROS2 packages simultaneously:
  - `ros2 launch roboto_diffbot launch_roboto.launch.py`
  - `ros2 launch roboto_viz gui_launch.py`
- Automatically cleans up both processes when the GUI is closed
- Remains visible after GUI closes, allowing restart without rebooting
- Large, visible UI optimized for 1920x1080 displays

### 2. Main GUI Modifications

**Modified Files**:
- `roboto_viz/main_view.py` - Added close button and shutdown signal
- `roboto_viz/gui_state_machine.py` - Added shutdown handler

**Changes**:
- Added a red "✕ Zamknij" (Close) button at the top-right of the main view
- Clicking the button triggers a clean shutdown of:
  - The GUI application
  - The ROS2 nodes
  - Returns control to the launcher application

## Architecture

```
System Boot
    ↓
Launcher App (autostart)
    ↓
[User clicks "Uruchom tryb jazdy"]
    ↓
Launches:
  - roboto_diffbot (background process)
  - roboto_viz GUI (foreground process)
    ↓
[User works with robot]
    ↓
[User clicks "✕ Zamknij" button]
    ↓
GUI closes + diffbot process terminated
    ↓
Back to Launcher App (ready to restart)
```

## Process Management

### Launcher Application
- Uses `QProcess` to spawn ROS2 launch commands
- Monitors GUI process state
- Automatically terminates diffbot when GUI closes
- Handles cleanup on application exit

### GUI Application
- Emits `request_shutdown` signal when close button is clicked
- Cleanly shuts down ROS2 nodes through `gui_manager.stop()`
- Calls `rclpy.shutdown()` to cleanup ROS2 context
- Exits PyQt5 application loop

## Directory Structure

```
roboto_viz/
├── launcher_app/                    # NEW - Standalone launcher (not built by colcon)
│   ├── launcher.py                  # Main launcher application
│   ├── README.md                    # Basic usage
│   ├── AUTOSTART_SETUP.md          # Autostart configuration guide
│   └── IMPLEMENTATION_SUMMARY.md   # This file
├── roboto_viz/                      # Existing ROS2 package
│   ├── main_view.py                # MODIFIED - Added close button
│   ├── gui_state_machine.py        # MODIFIED - Added shutdown handler
│   └── ...                         # Other existing files
├── gui_app/                        # Existing entry point
├── launch/                         # Existing launch files
└── ...
```

## Important Notes

### Launcher App is NOT Built by Colcon

The `launcher_app/` folder is intentionally **separate** from the ROS2 package. It:
- Does **NOT** get installed by `colcon build`
- Is a standalone Python application
- Can be run directly with `python3 launcher.py`
- Uses absolute paths to launch ROS2 packages

### ROS2 Package Paths

The launcher uses these paths (verified for your system):
- ROS2 setup: `/opt/ros/jazzy/setup.bash`
- Workspace setup: `/home/rooteq/ros2_ws/install/setup.bash`

If you move the workspace or use a different ROS2 distribution, update these paths in `launcher.py`.

## Testing

### Before Setting Up Autostart

1. **Test the launcher manually**:
   ```bash
   cd /home/rooteq/ros2_ws/src/roboto_viz/launcher_app
   python3 launcher.py
   ```

2. **Click "Uruchom tryb jazdy"** - Both packages should launch

3. **Test the close button** - Click "✕ Zamknij" in the GUI
   - GUI should close
   - Launcher should remain visible
   - Button should become enabled again

4. **Test restart** - Click "Uruchom tryb jazdy" again to ensure it works

### After Everything Works

Follow the instructions in `AUTOSTART_SETUP.md` to configure autostart at boot.

## Usage Workflow

### Normal Operation

1. System boots → Launcher appears automatically
2. User clicks "Uruchom tryb jazdy"
3. Robot control system starts
4. User operates robot
5. User clicks "✕ Zamknij" to close
6. Back to step 2 (can restart without reboot)

### Debugging

- Launcher logs: `journalctl -u robot-launcher.service -f`
- GUI logs: Check terminal output from launcher
- Manual testing: Run `python3 launcher.py` directly

## Dependencies

### System Requirements
- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12+
- PyQt5

### Python Packages
- PyQt5 (already required by roboto_viz)
- No additional dependencies needed

## Configuration

### Systemd Service

The recommended autostart method uses a systemd service that:
- Starts after graphical target
- Runs as user `rooteq`
- Sets proper DISPLAY and X11 environment
- Restarts on failure
- Logs to system journal

See `AUTOSTART_SETUP.md` for complete setup instructions.

## Troubleshooting

### Launcher doesn't start at boot
- Check service status: `sudo systemctl status robot-launcher.service`
- View logs: `journalctl -u robot-launcher.service -n 50`
- Verify DISPLAY variable is correct (usually `:0`)

### "Uruchom tryb jazdy" button does nothing
- Check if ROS2 is sourced correctly
- Verify paths in launcher.py match your system
- Run launcher manually to see error messages

### Close button doesn't work
- Rebuild the package: `colcon build --packages-select roboto_viz`
- Source the workspace: `source install/setup.bash`
- Restart the GUI

### Processes don't terminate
- Check process list: `ps aux | grep ros2`
- Manually kill if needed: `pkill -f "ros2 launch"`
- Check QProcess cleanup in launcher.py

## Future Enhancements

Possible improvements:
- Add status indicators for individual processes
- Add restart button without full shutdown
- Add system status monitoring
- Add error notification dialogs
- Add process output logging to file

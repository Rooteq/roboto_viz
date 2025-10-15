# Terminal Output Mode

The launcher now opens ROS2 processes in **visible terminal windows** so you can see all output, logs, and debug information.

## How It Works

When you click "Uruchom tryb jazdy", the launcher will:

1. **Open first terminal** (top-left): `RobotoDiffbot`
   - Runs: `ros2 launch roboto_diffbot launch_roboto.launch.py`
   - Shows all diffbot output and logs

2. **Wait 2 seconds**

3. **Open second terminal** (top-right): `RobotoViz GUI`
   - Runs: `ros2 launch roboto_viz gui_launch.py`
   - Shows all GUI output and logs
   - Opens the robot control GUI window

## Terminal Positioning

- **Diffbot terminal**: Top-left corner (geometry: 100x30+0+0)
- **GUI terminal**: Top-right corner (geometry: 100x30+800+0)

This way both terminals are visible and don't overlap.

## Benefits

✅ **See all ROS2 output** - No hidden logs or errors
✅ **Debug easier** - See what's happening in real-time
✅ **Monitor status** - Watch ROS2 nodes starting up
✅ **Error visibility** - Errors are immediately visible in terminals

## Terminal Behavior

### During Operation
- Both terminals stay open while the system is running
- You can scroll through the output
- Output continues in real-time

### When GUI Closes
When you click "✕ Zamknij" in the main robot GUI:
- All ROS2 processes are terminated (`pkill -f ros2 launch`)
- Both terminals will close automatically
- Launcher button becomes enabled again

### When Launcher Closes
When you click "✕" in the launcher:
- All ROS2 processes are killed
- All terminals are closed
- Launcher exits completely

## Terminal Emulator

The launcher uses `gnome-terminal` by default (standard on Ubuntu).

If you're using a different desktop environment, you might need to change the terminal command in `launcher.py`:

### For other terminals:

**Konsole (KDE)**:
```python
diffbot_cmd = f"konsole --title 'RobotoDiffbot' -e bash -c '...'"
gui_cmd = f"konsole --title 'RobotoViz GUI' -e bash -c '...'"
```

**xterm**:
```python
diffbot_cmd = f"xterm -title 'RobotoDiffbot' -geometry 100x30+0+0 -hold -e bash -c '...'"
gui_cmd = f"xterm -title 'RobotoViz GUI' -geometry 100x30+800+0 -hold -e bash -c '...'"
```

**Terminator**:
```python
diffbot_cmd = f"terminator --title='RobotoDiffbot' -e 'bash -c \"...\"'"
gui_cmd = f"terminator --title='RobotoViz GUI' -e 'bash -c \"...\"'"
```

## Troubleshooting

### Issue: Terminals don't appear

**Check if gnome-terminal is installed**:
```bash
which gnome-terminal
```

If not installed:
```bash
sudo apt install gnome-terminal
```

### Issue: Terminals close immediately

**Cause**: The ROS2 launch command might be failing.

**Solution**: Check the terminal output before it closes. The GUI terminal is configured to wait for Enter after the launch command fails, so you can read the error.

### Issue: Can't see terminal output (behind other windows)

**Solution**: The terminals should appear on top. If not, you can:
1. Alt+Tab to switch to the terminal windows
2. Look for "RobotoDiffbot" and "RobotoViz GUI" in the taskbar
3. Adjust the geometry positions in launcher.py if needed

### Issue: Want output in launcher window instead

If you prefer output in the launcher window instead of separate terminals, you can modify the launcher to capture output. See the git history for the previous version that captured output internally.

## Customizing Terminal Positions

Edit `launcher.py` and change the `--geometry` parameter:

**Format**: `--geometry=WIDTHxHEIGHT+X+Y`

Examples:
```python
# Bottom-left corner
diffbot_cmd = f"gnome-terminal --title='RobotoDiffbot' --geometry=100x30+0+500 -- ..."

# Right side, full height
gui_cmd = f"gnome-terminal --title='RobotoViz GUI' --geometry=80x60+1200+0 -- ..."

# Larger terminals
diffbot_cmd = f"gnome-terminal --title='RobotoDiffbot' --geometry=120x40+0+0 -- ..."
```

## Advantages of Terminal Mode

1. **Full ROS2 output** - See everything ROS2 prints
2. **Colored output** - Terminal preserves ROS2's colored logging
3. **Copy-paste** - Can copy error messages from terminal
4. **Scrollback** - Can scroll through history
5. **Separate windows** - Can position terminals where you want
6. **Debugging** - Much easier to debug issues

## Alternative: Hidden Mode

If you don't want to see terminals, you can modify the launcher to run processes in the background (without terminals). This would be similar to how it worked before this change.

To revert to hidden mode, check the git history for the previous version of `launch_robot_system()` and `launch_gui_process()`.

## Summary

**Now**: ROS2 output is visible in terminal windows ✅
**Before**: ROS2 output was hidden, only accessible via logs ❌

This makes debugging and monitoring much easier!

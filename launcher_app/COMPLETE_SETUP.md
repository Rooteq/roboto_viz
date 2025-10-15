# Complete Setup Guide - Robot Control Launcher

## Quick Reference: What You Have

✅ **Launcher App** - Fullscreen app that starts robot system
✅ **Close Button** - Red ✕ button in top-right corner
✅ **Main GUI Close Button** - Closes GUI and returns to launcher
✅ **Auto-cleanup** - All processes terminate when closing

---

## Complete Setup in 3 Steps

### Step 1: Build the Modified GUI Package

```bash
cd ~/ros2_ws
colcon build --packages-select roboto_viz
source install/setup.bash
```

### Step 2: Test the Launcher

```bash
cd ~/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

**Expected behavior**:
- Fullscreen launcher appears
- Red ✕ button in top-right corner
- Click "Uruchom tryb jazdy" to start robot system
- Click ✕ to close launcher

### Step 3: Choose Your Setup

Pick one or more options:

#### Option A: Desktop Icon (Quick Restart)
```bash
cat > ~/Desktop/robot-launcher.desktop << 'EOF'
[Desktop Entry]
Version=1.0
Type=Application
Name=Robot Launcher
Comment=Launch Robot Control System
Exec=/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Icon=utilities-terminal
Terminal=false
Categories=System;Utility;
StartupNotify=true
EOF

chmod +x ~/Desktop/robot-launcher.desktop
gio set ~/Desktop/robot-launcher.desktop metadata::trusted true
```

#### Option B: Autostart at Boot
```bash
sudo nano /etc/systemd/system/robot-launcher.service
```

Add:
```ini
[Unit]
Description=Robot Control Launcher
After=graphical.target
Wants=graphical.target

[Service]
Type=simple
User=rooteq
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/rooteq/.Xauthority"
Environment="XDG_RUNTIME_DIR=/run/user/1000"
WorkingDirectory=/home/rooteq/ros2_ws/src/roboto_viz/launcher_app
ExecStart=/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=graphical.target
```

Enable:
```bash
sudo systemctl daemon-reload
sudo systemctl enable robot-launcher.service
sudo systemctl start robot-launcher.service
```

---

## Usage Workflow

### Normal Operation Flow

1. **System boots** → Launcher appears (if autostart enabled)
2. **Click "Uruchom tryb jazdy"** → Robot system starts
3. **Work with robot** → Use main GUI for control
4. **Click "✕ Zamknij"** in main GUI → GUI closes, launcher reappears
5. **Repeat from step 2** or click ✕ in launcher to fully exit

### Manual Start Flow

1. **Double-click desktop icon** → Launcher appears
2. **Follow normal operation flow** from step 2 above

---

## All Features

### Launcher App Features:
- ✅ Fullscreen display (1920x1080 optimized)
- ✅ Large fonts and buttons for visibility
- ✅ Close button (✕) in top-right corner
- ✅ "Uruchom tryb jazdy" button to start robot system
- ✅ Status display (yellow → green)
- ✅ Automatic process management
- ✅ Clean shutdown of all processes

### Main GUI Features:
- ✅ Robot control interface
- ✅ Close button (✕ Zamknij) at top
- ✅ Returns to launcher when closed
- ✅ Clean ROS2 node shutdown

---

## File Locations

```
~/ros2_ws/src/roboto_viz/launcher_app/
├── launcher.py                  # Main launcher application
├── QUICK_START.md               # Quick testing guide
├── AUTOSTART_SETUP.md          # Autostart configuration
├── DESKTOP_ICON_SETUP.md       # Desktop icon guide
├── COMPLETE_SETUP.md           # This file
├── IMPLEMENTATION_SUMMARY.md   # Technical details
├── VERIFICATION.md             # Environment verification
├── CHANGES.md                  # Change log
└── README.md                   # Basic info
```

---

## Troubleshooting

### Launcher won't start
```bash
# Test manually
cd ~/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
# Check for errors in terminal
```

### Robot system won't launch
```bash
# Verify packages built
cd ~/ros2_ws
colcon build --packages-select roboto_viz roboto_diffbot
source install/setup.bash

# Test commands manually
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roboto_diffbot launch_roboto.launch.py
# In another terminal:
ros2 launch roboto_viz gui_launch.py
```

### Close button doesn't work
- Make sure you rebuilt the GUI package: `colcon build --packages-select roboto_viz`
- Source the workspace: `source ~/ros2_ws/install/setup.bash`

### Desktop icon doesn't appear
```bash
# Check file exists and is executable
ls -l ~/Desktop/robot-launcher.desktop

# Make executable if needed
chmod +x ~/Desktop/robot-launcher.desktop

# Trust the launcher
gio set ~/Desktop/robot-launcher.desktop metadata::trusted true
```

---

## Key Commands Reference

### Build and Test
```bash
# Build GUI with close button
cd ~/ros2_ws
colcon build --packages-select roboto_viz
source install/setup.bash

# Test launcher
cd ~/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

### Autostart Management
```bash
# Check status
sudo systemctl status robot-launcher.service

# View logs
journalctl -u robot-launcher.service -f

# Stop/start
sudo systemctl stop robot-launcher.service
sudo systemctl start robot-launcher.service

# Enable/disable autostart
sudo systemctl enable robot-launcher.service
sudo systemctl disable robot-launcher.service
```

### Desktop Icon
```bash
# Create desktop icon (one command)
cat > ~/Desktop/robot-launcher.desktop << 'EOF'
[Desktop Entry]
Version=1.0
Type=Application
Name=Robot Launcher
Comment=Launch Robot Control System
Exec=/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Icon=utilities-terminal
Terminal=false
Categories=System;Utility;
StartupNotify=true
EOF
chmod +x ~/Desktop/robot-launcher.desktop
gio set ~/Desktop/robot-launcher.desktop metadata::trusted true
```

---

## Recommended Setup for Production

For a production robot control system:

1. ✅ **Enable autostart** - Launcher starts at boot
2. ✅ **Create desktop icon** - Easy manual restart
3. ✅ **Test thoroughly** - Verify all workflows
4. ✅ **Document for users** - Show how to use buttons

---

## Support

For detailed information, see:
- [QUICK_START.md](QUICK_START.md) - Quick testing
- [AUTOSTART_SETUP.md](AUTOSTART_SETUP.md) - Boot autostart
- [DESKTOP_ICON_SETUP.md](DESKTOP_ICON_SETUP.md) - Desktop icons
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Technical details

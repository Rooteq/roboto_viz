# Fix: "Qt platform plugin could not be initialized"

## Error Message
```
This application failed to start because no Qt platform plugin could be initialized.
Reinstalling the application may fix this problem.

Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, wayland-egl, wayland, xcb.
```

## Cause
The systemd user service doesn't have proper access to the X11 display server or is missing Qt environment variables.

## Quick Fix

### Step 1: Update Your Service File

Edit the service file:
```bash
nano ~/.config/systemd/user/robot-launcher.service
```

Make sure it has **ALL** these Environment variables:
```ini
[Unit]
Description=Robot Control Launcher
After=graphical-session.target

[Service]
Type=simple
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/amr/.Xauthority"
Environment="XDG_RUNTIME_DIR=/run/user/%U"
Environment="QT_QPA_PLATFORM=xcb"
WorkingDirectory=/home/amr/ros2_ws/src/roboto_viz/launcher_app
ExecStart=/usr/bin/python3 /home/amr/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
```

**Important**: Replace `/home/amr/` with your actual username path!

### Step 2: Reload and Restart

```bash
systemctl --user daemon-reload
systemctl --user restart robot-launcher.service
systemctl --user status robot-launcher.service
```

### Step 3: Check Logs

```bash
journalctl --user -u robot-launcher.service -f
```

Should now start without errors!

---

## Alternative: Use Desktop Autostart (Simpler)

If the user service keeps having issues, use this method instead:

```bash
mkdir -p ~/.config/autostart

cat > ~/.config/autostart/robot-launcher.desktop << 'EOF'
[Desktop Entry]
Type=Application
Name=Robot Launcher
Comment=Launch Robot Control System
Exec=/usr/bin/python3 /home/amr/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Terminal=false
X-GNOME-Autostart-enabled=true
EOF

chmod +x ~/.config/autostart/robot-launcher.desktop
```

**Advantage**: Automatically has access to your display session (no environment variable issues).

**Test**: Log out and log back in. The launcher should appear automatically.

---

## Detailed Troubleshooting

### Check DISPLAY Variable

Find your actual DISPLAY value:
```bash
echo $DISPLAY
```

Common values:
- `:0` - Most common
- `:1` - Second display
- `:10` or `:1001` - Remote or VNC sessions

If it's **not** `:0`, update the service file:
```ini
Environment="DISPLAY=:1"
```

### Check .Xauthority File

Verify it exists and has correct permissions:
```bash
ls -la ~/.Xauthority
```

Should show:
```
-rw------- 1 amr amr 123 ... .Xauthority
```

If missing or wrong permissions:
```bash
touch ~/.Xauthority
chmod 600 ~/.Xauthority
```

### Check Qt Plugins

Verify Qt plugins are installed:
```bash
python3 -c "from PyQt5.QtWidgets import QApplication; import sys; app = QApplication(sys.argv); print('Qt works!')"
```

If this fails, install Qt dependencies:
```bash
sudo apt install python3-pyqt5 python3-pyqt5.qtsvg libqt5gui5 libqt5x11extras5
```

### Test Manually First

**Always test manually before using service:**
```bash
cd /home/amr/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

- If this **works**: Problem is service environment variables
- If this **fails**: Problem is Qt installation or dependencies

### Check Service User

Make sure service runs as your user:
```bash
systemctl --user status robot-launcher.service
```

Should show your username, not root.

---

## Summary: Three Methods (Pick One)

### Method 1: User Service (Most Control)
- ✅ Runs at boot (with lingering enabled)
- ✅ Restarts on failure
- ⚠️ Requires correct environment variables

**Status**: Use the service file above with all Environment variables.

### Method 2: Desktop Autostart (Simplest)
- ✅ No environment variable issues
- ✅ Always has display access
- ⚠️ Only starts after user login

**Status**: Use the desktop autostart method above.

### Method 3: Desktop Icon (Manual Start)
- ✅ No autostart configuration needed
- ✅ User controls when to start
- ⚠️ Must click to start

**Status**: Double-click desktop icon to launch.

---

## Recommended Solution

For your robot control system, I recommend:

1. **Use Desktop Autostart** (simplest, most reliable):
   ```bash
   mkdir -p ~/.config/autostart
   cat > ~/.config/autostart/robot-launcher.desktop << 'EOF'
   [Desktop Entry]
   Type=Application
   Name=Robot Launcher
   Comment=Launch Robot Control System
   Exec=/usr/bin/python3 /home/amr/ros2_ws/src/roboto_viz/launcher_app/launcher.py
   Terminal=false
   X-GNOME-Autostart-enabled=true
   EOF
   chmod +x ~/.config/autostart/robot-launcher.desktop
   ```

2. **Create Desktop Icon** (for manual restart):
   ```bash
   cat > ~/Desktop/robot-launcher.desktop << 'EOF'
   [Desktop Entry]
   Version=1.0
   Type=Application
   Name=Robot Launcher
   Comment=Launch Robot Control System
   Exec=/usr/bin/python3 /home/amr/ros2_ws/src/roboto_viz/launcher_app/launcher.py
   Icon=utilities-terminal
   Terminal=false
   Categories=System;Utility;
   StartupNotify=true
   EOF
   chmod +x ~/Desktop/robot-launcher.desktop
   gio set ~/Desktop/robot-launcher.desktop metadata::trusted true
   ```

This gives you:
- ✅ Automatic start after login
- ✅ Desktop icon for manual restart
- ✅ No Qt plugin errors
- ✅ Simple and reliable

**Test by logging out and back in!**

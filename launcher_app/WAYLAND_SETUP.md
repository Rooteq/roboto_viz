# Wayland Setup for Robot Control Launcher

## For Ubuntu with Wayland Session

If you're using Ubuntu with Wayland (default on Ubuntu 22.04+), use this configuration instead of the X11 configuration.

## Quick Setup for Wayland

### Method 1: Desktop Autostart (Recommended - Always Works)

This method automatically inherits the Wayland session, so it just works:

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

**That's it!** Log out and log back in. The launcher will start automatically.

**Why this is best for Wayland:**
- ✅ Automatically has Wayland session access
- ✅ No environment variable configuration needed
- ✅ No Qt platform plugin errors
- ✅ Works with both Wayland and X11

---

### Method 2: User Systemd Service for Wayland

If you prefer systemd service, use this Wayland-specific configuration:

```bash
mkdir -p ~/.config/systemd/user
nano ~/.config/systemd/user/robot-launcher.service
```

**For Wayland, use this configuration:**

```ini
[Unit]
Description=Robot Control Launcher
After=graphical-session.target

[Service]
Type=simple
Environment="WAYLAND_DISPLAY=wayland-0"
Environment="XDG_RUNTIME_DIR=/run/user/%U"
Environment="QT_QPA_PLATFORM=wayland"
Environment="GDK_BACKEND=wayland"
Environment="CLUTTER_BACKEND=wayland"
WorkingDirectory=/home/amr/ros2_ws/src/roboto_viz/launcher_app
ExecStart=/usr/bin/python3 /home/amr/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
```

Then enable and start:

```bash
systemctl --user daemon-reload
systemctl --user enable robot-launcher.service
systemctl --user start robot-launcher.service
systemctl --user status robot-launcher.service
```

---

## How to Check if You're Using Wayland

Run this command:
```bash
echo $XDG_SESSION_TYPE
```

Output:
- `wayland` - You're using Wayland ✅
- `x11` - You're using X11
- Empty - Check with: `loginctl show-session $(loginctl | grep $(whoami) | awk '{print $1}') -p Type`

Or check in Settings:
1. Open Settings
2. Go to About
3. Look for "Windowing System" - should say "Wayland"

---

## Universal Configuration (Works for Both X11 and Wayland)

If you want a configuration that works regardless of whether you're using X11 or Wayland:

```ini
[Unit]
Description=Robot Control Launcher
After=graphical-session.target

[Service]
Type=simple
Environment="XDG_RUNTIME_DIR=/run/user/%U"
Environment="QT_QPA_PLATFORM=wayland;xcb"
Environment="WAYLAND_DISPLAY=wayland-0"
Environment="DISPLAY=:0"
WorkingDirectory=/home/amr/ros2_ws/src/roboto_viz/launcher_app
ExecStart=/usr/bin/python3 /home/amr/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
```

The `QT_QPA_PLATFORM=wayland;xcb` tells Qt to try Wayland first, fall back to X11.

---

## Troubleshooting Wayland

### Error: "Could not connect to display"

**Solution**: Make sure `XDG_RUNTIME_DIR` is set correctly:
```bash
echo $XDG_RUNTIME_DIR
# Should show: /run/user/1000 (or your user ID)

id -u
# This is your user ID, use it in the service file
```

### Error: Qt still can't initialize

**Solution 1**: Install Qt Wayland support:
```bash
sudo apt install qt5-wayland python3-pyqt5
```

**Solution 2**: Force X11 compatibility mode:
```ini
Environment="QT_QPA_PLATFORM=xcb"
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/amr/.Xauthority"
```

This makes Qt apps run in X11 compatibility mode on Wayland (XWayland).

### Service starts but no window appears

**Cause**: Service starting before Wayland compositor is ready.

**Solution**: Add delay to service:
```ini
[Service]
Type=simple
ExecStartPre=/bin/sleep 5
Environment="WAYLAND_DISPLAY=wayland-0"
...
```

Or use desktop autostart instead (recommended).

---

## Recommended Setup for Ubuntu Wayland

For maximum compatibility and reliability on Ubuntu with Wayland:

### 1. Use Desktop Autostart
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

### 2. Create Desktop Icon
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

**Why this is best:**
- ✅ Works on both Wayland and X11
- ✅ No configuration needed
- ✅ Inherits correct session automatically
- ✅ Starts when desktop is ready
- ✅ Can manually restart from desktop icon

---

## Test Manual Launch First

Before setting up autostart, test that it works:

```bash
cd /home/amr/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

If this works in your terminal, then desktop autostart will work too!

---

## Summary

**For Wayland (Ubuntu 22.04+):**

| Method | Difficulty | Reliability | Recommendation |
|--------|-----------|-------------|----------------|
| Desktop Autostart | ⭐ Easy | ⭐⭐⭐ High | ✅ **Recommended** |
| Systemd User Service | ⭐⭐ Medium | ⭐⭐ Medium | Use if you need more control |
| Desktop Icon Only | ⭐ Easy | ⭐⭐⭐ High | Good for manual start |

**Best choice**: Desktop Autostart + Desktop Icon

This gives you automatic start after login and manual restart capability, with zero configuration hassle!

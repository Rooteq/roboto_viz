# Desktop Icon Setup for Robot Launcher

This guide explains how to create a desktop icon for the Robot Control Launcher so users can easily run it again after closing.

## Method 1: Create Desktop Icon (Recommended for Easy Access)

### Step 1: Create the Desktop Entry File

Create a desktop entry file that can be placed on the desktop:

```bash
nano ~/Desktop/robot-launcher.desktop
```

Add the following content:

```ini
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
```

### Step 2: Make it Executable

```bash
chmod +x ~/Desktop/robot-launcher.desktop
```

### Step 3: Trust the Launcher (Ubuntu 22.04+)

On Ubuntu 22.04 and later, you need to explicitly trust the desktop file:

1. Right-click the icon on the desktop
2. Select "Allow Launching" or "Trust and Launch"

Alternatively, via command line:
```bash
gio set ~/Desktop/robot-launcher.desktop metadata::trusted true
```

### Step 4: Test the Icon

Double-click the icon on the desktop. The Robot Launcher should appear in fullscreen.

---

## Method 2: Add to Applications Menu (System-Wide Access)

This makes the launcher available from the applications menu (Activities/Show Applications).

### Step 1: Create Desktop Entry in Applications

```bash
sudo nano /usr/share/applications/robot-launcher.desktop
```

Add the following content:

```ini
[Desktop Entry]
Version=1.0
Type=Application
Name=Robot Launcher
Comment=Launch Robot Control System
Exec=/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Icon=robot
Terminal=false
Categories=System;Utility;
StartupNotify=true
Keywords=robot;control;launcher;
```

### Step 2: Update Desktop Database

```bash
sudo update-desktop-database
```

### Step 3: Search and Launch

1. Press the Super key (Windows key) or click "Activities"
2. Type "Robot Launcher"
3. Click the icon to launch

---

## Method 3: Create a Custom Icon (Optional)

If you want a custom icon instead of the default terminal icon:

### Step 1: Create or Download an Icon

Create a simple robot icon or download one. Save it as a PNG file (recommended size: 256x256 or 512x512).

For example, save it as:
```
/home/rooteq/ros2_ws/src/roboto_viz/launcher_app/robot-icon.png
```

### Step 2: Update the Desktop Entry

Edit your desktop file and change the Icon line:

```ini
Icon=/home/rooteq/ros2_ws/src/roboto_viz/launcher_app/robot-icon.png
```

Or use a system icon name like:
- `robot` - Generic robot icon
- `applications-system` - System applications icon
- `utilities-terminal` - Terminal icon
- `computer` - Computer icon

---

## Method 4: Quick Launch Script (Alternative)

Create a simple bash script for easy launching:

### Step 1: Create Launch Script

```bash
nano ~/launch-robot.sh
```

Add this content:

```bash
#!/bin/bash
cd /home/rooteq/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

### Step 2: Make it Executable

```bash
chmod +x ~/launch-robot.sh
```

### Step 3: Use the Script

You can now launch the application by running:
```bash
~/launch-robot.sh
```

Or create a desktop entry for this script using Method 1 above.

---

## Troubleshooting

### Issue: Icon doesn't appear on desktop

**Solution**: Make sure the desktop entry file is in the correct location and has the correct permissions:
```bash
ls -l ~/Desktop/robot-launcher.desktop
# Should show: -rwxr-xr-x
```

### Issue: Double-clicking shows "Untrusted Application Launcher"

**Solution**: Trust the application:
```bash
gio set ~/Desktop/robot-launcher.desktop metadata::trusted true
```

Or right-click and select "Allow Launching".

### Issue: Launcher doesn't start from icon

**Solution**: Test the command manually first:
```bash
/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
```

If this works but the icon doesn't, try adding `Terminal=true` to the desktop entry for debugging.

### Issue: Icon shows but has wrong appearance

**Solution**: Update the icon cache:
```bash
gtk-update-icon-cache -f -t ~/.icons/hicolor
# Or system-wide:
sudo gtk-update-icon-cache -f -t /usr/share/icons/hicolor
```

---

## Summary of Files

After setup, you should have:

- **Desktop Icon**: `~/Desktop/robot-launcher.desktop` (for desktop access)
- **System Menu**: `/usr/share/applications/robot-launcher.desktop` (for menu access)
- **Optional Script**: `~/launch-robot.sh` (alternative launcher)
- **Optional Icon**: Custom icon image file

---

## Best Practice Setup for Your Use Case

Since this is for a robot control system that starts at boot and users may close/restart:

1. **For Autostart**: Use systemd service (already documented in AUTOSTART_SETUP.md)
2. **For Manual Restart**: Create desktop icon using Method 1
3. **For Additional Users**: Add to applications menu using Method 2

This way:
- System boots → Launcher starts automatically
- User closes launcher → Desktop icon available for restart
- Any user can find it in applications menu

---

## Complete Quick Setup

Here's a one-command setup for desktop icon:

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

Done! The icon should now be on your desktop and ready to use.

# Setup for User 'amr'

This guide is for setting up the Robot Control Launcher on a PC with username **amr**.

## Important: Update Paths

The launcher.py file currently has hardcoded paths for user `rooteq`. You need to update them for user `amr`.

### Step 1: Update launcher.py Paths

On the PC with username `amr`, edit the launcher file:

```bash
nano /home/amr/ros2_ws/src/roboto_viz/launcher_app/launcher.py
```

Find these lines (around line 105):
```python
workspace_setup = "~/ros2_ws/install/setup.bash"
```

The `~` will automatically expand to the correct home directory, so this should work as-is.

**However**, if you need to use absolute paths, change it to:
```python
workspace_setup = "/home/amr/ros2_ws/install/setup.bash"
```

### Step 2: Fix User Service (for the error you're seeing)

#### Remove the System Service

```bash
sudo systemctl stop robot-launcher.service
sudo systemctl disable robot-launcher.service
sudo rm /etc/systemd/system/robot-launcher.service
sudo systemctl daemon-reload
```

#### Create User Service Directory

```bash
mkdir -p ~/.config/systemd/user
```

#### Create User Service File

```bash
nano ~/.config/systemd/user/robot-launcher.service
```

**Add this content (adjusted for user amr):**

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

#### Enable and Start

```bash
systemctl --user daemon-reload
systemctl --user enable robot-launcher.service
systemctl --user start robot-launcher.service
```

#### Check Status

```bash
systemctl --user status robot-launcher.service
```

### Step 3: Get User ID for amr

Run this command to find the UID:
```bash
id -u amr
```

If the UID is different from 1000, you may need to update `XDG_RUNTIME_DIR` in the system service (but you shouldn't need this for user service).

### Step 4: Enable Lingering (Optional)

To start at boot before login:
```bash
sudo loginctl enable-linger amr
```

## Desktop Icon for User amr

Create desktop icon:

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

## Quick Commands for User amr

```bash
# Start service
systemctl --user start robot-launcher.service

# Stop service
systemctl --user stop robot-launcher.service

# Check status
systemctl --user status robot-launcher.service

# View logs
journalctl --user -u robot-launcher.service -f

# Test manually
cd /home/amr/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

## If Workspace is in Different Location

If your ROS2 workspace is in a different location than `/home/amr/ros2_ws`, update:

1. **In launcher.py** (line ~105):
   ```python
   workspace_setup = "/path/to/your/ros2_ws/install/setup.bash"
   ```

2. **In service file**:
   ```ini
   WorkingDirectory=/path/to/your/ros2_ws/src/roboto_viz/launcher_app
   ExecStart=/usr/bin/python3 /path/to/your/ros2_ws/src/roboto_viz/launcher_app/launcher.py
   ```

## Troubleshooting

### Error: "Qt platform plugin could not be initialized"

**Full error**: `This application failed to start because no Qt platform plugin could be initialized.`

**Cause**: The service doesn't have proper access to the X11 display server.

**Solution 1: Update service file with X11 environment variables**

Make sure your service file includes these environment variables:
```ini
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/amr/.Xauthority"
Environment="XDG_RUNTIME_DIR=/run/user/%U"
Environment="QT_QPA_PLATFORM=xcb"
```

The service file in Step 2 already includes these. If you created it without them, update it:

```bash
nano ~/.config/systemd/user/robot-launcher.service
# Add the Environment lines above
systemctl --user daemon-reload
systemctl --user restart robot-launcher.service
```

**Solution 2: Check X11 permissions**

Make sure the .Xauthority file is readable:
```bash
ls -la ~/.Xauthority
```

If it doesn't exist or has wrong permissions:
```bash
touch ~/.Xauthority
chmod 600 ~/.Xauthority
```

**Solution 3: Find correct DISPLAY value**

Check what DISPLAY value you're using:
```bash
echo $DISPLAY
```

If it's not `:0`, update the service file with the correct value (e.g., `:1` or `:10`).

**Solution 4: Test manually first**

Before using the service, test that the launcher works manually:
```bash
cd /home/amr/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

If this works but the service doesn't, it's definitely an environment variable issue.

**Solution 5: Use Desktop Autostart Instead**

If the user service keeps having issues, use the desktop autostart method instead (simpler, works after login):

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

This will start the launcher after you log in to the desktop.

### Error: "No such file or directory"

Check if the workspace exists:
```bash
ls -la /home/amr/ros2_ws/src/roboto_viz/launcher_app/
```

If not, adjust the paths in the service file.

### Error: "Failed to determine user credentials"

You're using a system service instead of user service. Follow Step 2 above to switch to user service.

### Launcher doesn't appear

Check the display:
```bash
echo $DISPLAY
```

If it's not `:0`, update the service file to use the correct DISPLAY value.

## Summary for User amr

1. ✅ Copy workspace to `/home/amr/ros2_ws/` (if not already there)
2. ✅ Remove system service
3. ✅ Create user service with paths for `/home/amr/`
4. ✅ Enable and start user service
5. ✅ Create desktop icon
6. ✅ Test by rebooting

The `~` in the launcher.py will automatically expand to `/home/amr/`, so you likely don't need to change the launcher.py file itself - just the service file!

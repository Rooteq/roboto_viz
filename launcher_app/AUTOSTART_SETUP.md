# Auto-start Setup for Robot Control Launcher

This document explains how to configure the Robot Control Launcher to start automatically at system boot on Ubuntu 24.04.

## Method 1: User Systemd Service (Recommended for GUI Apps)

This is the best method for GUI applications that need access to the user's display.

### Step 1: Create user systemd directory

```bash
mkdir -p ~/.config/systemd/user
```

### Step 2: Create user service file

```bash
nano ~/.config/systemd/user/robot-launcher.service
```

Add the following content:

```ini
[Unit]
Description=Robot Control Launcher
After=graphical-session.target

[Service]
Type=simple
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/rooteq/.Xauthority"
Environment="XDG_RUNTIME_DIR=/run/user/%U"
Environment="QT_QPA_PLATFORM=xcb"
WorkingDirectory=/home/rooteq/ros2_ws/src/roboto_viz/launcher_app
ExecStart=/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
```

### Step 3: Enable and start the user service

```bash
# Reload user systemd daemon
systemctl --user daemon-reload

# Enable the service to start at login
systemctl --user enable robot-launcher.service

# Start the service now (optional, for testing)
systemctl --user start robot-launcher.service
```

### Step 4: Enable lingering (optional, to start before login)

If you want the service to start even before user login:

```bash
sudo loginctl enable-linger rooteq
```

### Step 5: Check service status

```bash
# Check if the service is running
systemctl --user status robot-launcher.service

# View service logs
journalctl --user -u robot-launcher.service -f
```

### Step 6: Manage the service

```bash
# Stop the service
systemctl --user stop robot-launcher.service

# Disable autostart
systemctl --user disable robot-launcher.service

# Restart the service
systemctl --user restart robot-launcher.service
```

## Method 2: System Systemd Service (Alternative)

### Step 1: Create a systemd service file

Create a service file for the launcher application:

```bash
sudo nano /etc/systemd/system/robot-launcher.service
```

Add the following content (adjust paths if necessary):

```ini
[Unit]
Description=Robot Control Launcher
After=graphical.target
Wants=graphical.target

[Service]
Type=simple
User=rooteq
Group=rooteq
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/rooteq/.Xauthority"
Environment="XDG_RUNTIME_DIR=/run/user/1000"
PAMName=login
WorkingDirectory=/home/rooteq/ros2_ws/src/roboto_viz/launcher_app
ExecStart=/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=graphical.target
```

### Step 2: Enable and start the service

```bash
# Reload systemd to recognize the new service
sudo systemctl daemon-reload

# Enable the service to start at boot
sudo systemctl enable robot-launcher.service

# Start the service now (optional, for testing)
sudo systemctl start robot-launcher.service
```

### Step 3: Check service status

```bash
# Check if the service is running
sudo systemctl status robot-launcher.service

# View service logs
journalctl -u robot-launcher.service -f
```

### Step 4: Manage the service

```bash
# Stop the service
sudo systemctl stop robot-launcher.service

# Disable autostart
sudo systemctl disable robot-launcher.service

# Restart the service
sudo systemctl restart robot-launcher.service
```

## Method 2: Desktop Autostart Entry (Alternative)

If you prefer a user-level autostart (runs only when the specific user logs in):

### Step 1: Create autostart directory

```bash
mkdir -p ~/.config/autostart
```

### Step 2: Create desktop entry file

```bash
nano ~/.config/autostart/robot-launcher.desktop
```

Add the following content:

```ini
[Desktop Entry]
Type=Application
Name=Robot Control Launcher
Comment=Launcher for robot control system
Exec=/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Terminal=false
X-GNOME-Autostart-enabled=true
```

### Step 3: Make it executable

```bash
chmod +x ~/.config/autostart/robot-launcher.desktop
```

### Step 4: Test

Log out and log back in to test the autostart.

### Step 5: Disable (if needed)

```bash
rm ~/.config/autostart/robot-launcher.desktop
```

## Troubleshooting

### Issue: Black screen or application doesn't start

**Solution**: Make sure the DISPLAY environment variable is set correctly. For most systems, it's `:0`, but you can check by running `echo $DISPLAY` in a terminal.

### Issue: Permission denied errors

**Solution**: Ensure the launcher.py file is executable:
```bash
chmod +x /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
```

### Issue: ROS2 commands not found

**Solution**: The launcher script sources the ROS2 setup files automatically. Make sure the paths in launcher.py are correct:
- `/opt/ros/jazzy/setup.bash` (ROS2 installation)
- `/home/rooteq/ros2_ws/install/setup.bash` (Your workspace)

If your ROS2 installation is in a different location, update the paths in `launcher.py`.

### Issue: "Failed to determine user credentials: No such process"

**Problem**: This error occurs when using a system service (Method 2) trying to start a GUI application.

**Solution**: Use **Method 1 (User Systemd Service)** instead. If you already created a system service, remove it first:

```bash
# Stop and disable the system service
sudo systemctl stop robot-launcher.service
sudo systemctl disable robot-launcher.service

# Remove the system service file
sudo rm /etc/systemd/system/robot-launcher.service

# Reload systemd
sudo systemctl daemon-reload
```

Then follow **Method 1** to create a user service instead:

```bash
# Create user service directory
mkdir -p ~/.config/systemd/user

# Create user service file
nano ~/.config/systemd/user/robot-launcher.service
# (Add the configuration from Method 1)

# Enable and start user service
systemctl --user daemon-reload
systemctl --user enable robot-launcher.service
systemctl --user start robot-launcher.service
```

### Issue: Service fails to start

**Solution**: Check the service logs:

For user service:
```bash
journalctl --user -u robot-launcher.service -n 50
```

For system service:
```bash
sudo journalctl -u robot-launcher.service -n 50
```

Look for error messages and fix the paths or permissions accordingly.

## Testing Before Enabling Autostart

Before setting up autostart, test the launcher manually:

```bash
cd /home/rooteq/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

If this works correctly, you can proceed with the autostart setup.

## Notes

- The systemd method is recommended for production use as it provides better process management and logging
- The desktop autostart method is simpler but only works when a user logs in to the GUI
- Make sure your system has automatic login enabled if you want the launcher to start without user interaction
- The launcher will remain visible on screen even after the main GUI is closed, allowing you to restart the robot system

# Quick Fix: "Failed to determine user credentials" Error

## Problem
You're getting this error:
```
robot-launcher.service: Failed to determine user credentials: No such process
```

## Cause
This happens because you're using a **system service** to run a GUI application. System services run before user login and don't have access to the user's display session.

## Solution: Switch to User Service

### Step 1: Remove the System Service

```bash
# Stop and disable the system service
sudo systemctl stop robot-launcher.service
sudo systemctl disable robot-launcher.service

# Remove the system service file
sudo rm /etc/systemd/system/robot-launcher.service

# Reload systemd
sudo systemctl daemon-reload
```

### Step 2: Create User Service Directory

```bash
mkdir -p ~/.config/systemd/user
```

### Step 3: Create User Service File

```bash
nano ~/.config/systemd/user/robot-launcher.service
```

**Copy and paste this content:**

```ini
[Unit]
Description=Robot Control Launcher
After=graphical-session.target

[Service]
Type=simple
Environment="DISPLAY=:0"
WorkingDirectory=/home/rooteq/ros2_ws/src/roboto_viz/launcher_app
ExecStart=/usr/bin/python3 /home/rooteq/ros2_ws/src/roboto_viz/launcher_app/launcher.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
```

Save and exit (Ctrl+X, then Y, then Enter).

### Step 4: Enable and Start User Service

```bash
# Reload user systemd daemon
systemctl --user daemon-reload

# Enable the service to start at login
systemctl --user enable robot-launcher.service

# Start the service now
systemctl --user start robot-launcher.service
```

### Step 5: Check Status

```bash
# Check if it's running
systemctl --user status robot-launcher.service

# View logs
journalctl --user -u robot-launcher.service -f
```

You should see the service running without errors!

### Step 6: Enable Lingering (Optional)

If you want the service to start even before you log in (starts at system boot):

```bash
sudo loginctl enable-linger rooteq
```

## Verify It Works

1. Reboot your system:
   ```bash
   sudo reboot
   ```

2. After login, the launcher should appear automatically

3. Check status if needed:
   ```bash
   systemctl --user status robot-launcher.service
   ```

## Commands Reference

```bash
# Start
systemctl --user start robot-launcher.service

# Stop
systemctl --user stop robot-launcher.service

# Restart
systemctl --user restart robot-launcher.service

# Enable autostart
systemctl --user enable robot-launcher.service

# Disable autostart
systemctl --user disable robot-launcher.service

# View logs
journalctl --user -u robot-launcher.service -f

# Check status
systemctl --user status robot-launcher.service
```

## Why User Service?

**User Service** = Runs in your user session with access to display
**System Service** = Runs at system level, can't access user's X display

For GUI applications, always use user services!

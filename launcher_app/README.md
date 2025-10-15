# Robot Control Launcher

This is a standalone PyQt5 application that launches the robot control system.

## Features

- Simple button interface to launch robot system
- Launches both `roboto_diffbot` and `roboto_viz` packages
- Automatically cleans up processes when GUI is closed
- Returns to launcher after GUI closes

## Running Manually

```bash
cd /home/rooteq/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

## Auto-start at Boot

See the AUTOSTART_SETUP.md file for instructions on how to configure this application to start automatically at system boot.

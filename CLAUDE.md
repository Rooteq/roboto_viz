# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 package for a robot visualization GUI application (`roboto_viz`) that provides:
- PyQt5-based GUI for robot control and navigation
- Real-time map visualization with interactive route planning
- State machine-based architecture for different operation modes
- Integration with ROS2 navigation stack (nav2) and AMCL localization

## Architecture

### Core Components

**State Machine Architecture** (`gui_state_machine.py`):
- `DisconnectedState`: Initial state, waits for robot connection
- `ConfiguringState`: Robot setup and manual control mode
- `ActiveState`: Normal operation with navigation capabilities  
- `PlannerState`: Route planning and editing mode

**Main Application Flow**:
1. `gui_app/app.py` - Entry point, creates main application
2. `gui_state_machine.py` - Manages GUI states and transitions
3. `gui_manager.py` - ROS2 node handling robot communication
4. `main_view.py` - Main PyQt5 window with stacked views

**Key Modules**:
- `map_view.py` - Interactive map display with route visualization
- `views.py` - ActiveView and DisconnectedView UI components
- `route_manager.py` - Route storage and bezier curve management
- `robot_item.py` - Robot position visualization
- `bezier_graphics.py` - Bezier curve rendering for routes

### ROS2 Integration

**Node**: `ManagerNode` in `gui_manager.py`
- Lifecycle node for robot connection management
- Subscribers: `/diffbot_pose` (robot position updates)
- Publishers: `/cmd_vel_key` (manual control), `/initialpose` (AMCL initialization)
- Service clients: `/trigger_service` (robot availability check)

**Navigation**: Uses `nav2_simple_commander.BasicNavigator` for waypoint following

## Development Commands

### Building
```bash
# From ROS2 workspace root
colcon build --packages-select roboto_viz
```

### Running
```bash
# Launch GUI application
ros2 launch roboto_viz gui_launch.py

# Or run directly
ros2 run roboto_viz gui
```

### Testing
```bash
# Run all tests
colcon test --packages-select roboto_viz

# Run specific test types
pytest test/test_flake8.py    # Code style
pytest test/test_pep257.py    # Docstring style
pytest test/test_copyright.py # Copyright headers
```

### Linting
```bash
# Flake8 code style checking
ament_flake8 roboto_viz/
```

## File Structure

- `roboto_viz/` - Main package source code
- `gui_app/` - Application entry point
- `launch/` - ROS2 launch files
- `test/` - Unit tests and linting tests
- `resource/` - Package resources

## Dependencies

**ROS2 Packages**:
- `rclpy` - ROS2 Python client library
- `nav2_simple_commander` - Navigation interface
- `geometry_msgs`, `std_msgs` - Message types

**Python Packages**:
- `PyQt5` - GUI framework
- `yaml` - Configuration file parsing

## Map and Route Storage

Maps are stored in `~/.robotroutes/maps/` as `.pgm` files with corresponding `.yaml` metadata files. Routes are stored per map using the RouteManager system.
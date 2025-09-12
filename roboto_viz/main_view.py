
from __future__ import annotations
import sys
import os
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QPushButton, QStackedWidget, QLabel, QHBoxLayout, QListWidget, QListWidgetItem, QComboBox, QGridLayout
from abc import ABC, abstractmethod
import yaml
from PyQt5.QtCore import Qt, pyqtSignal, QObject, pyqtSlot, QTimer
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

import copy
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QTabWidget, QLineEdit
# internal imports:
from roboto_viz.map_view import MapView
from roboto_viz.goal_arrow import GoalArrow
from roboto_viz.robot_item import RobotItem
from pathlib import Path

from roboto_viz.views import ActiveView, DisconnectedView
from roboto_viz.plan_views import PlanActiveView
from roboto_viz.plan_manager import PlanManager
from roboto_viz.plan_executor import PlanExecutor
from roboto_viz.dock_manager import DockManager

class MainView(QMainWindow):
    map_loaded_signal = pyqtSignal(bool, str)  # success, error_message
    connection_signal = pyqtSignal(str)
    disconnection_signal = pyqtSignal(str)
    set_position_signal = pyqtSignal(float, float, float)
    map_changed_signal = pyqtSignal(str)  # map_name - for navigation system

    start_planning = pyqtSignal()
    finish_planning = pyqtSignal()
    
    # Plan execution signals
    start_plan_execution = pyqtSignal(str)  # plan_name
    stop_plan_execution = pyqtSignal()
    execute_plan_action = pyqtSignal(str, int)  # plan_name, action_index
    
    # Robot control signals
    dock_robot = pyqtSignal()
    dock_robot_at = pyqtSignal(str)  # dock_name
    undock_robot = pyqtSignal()
    start_nav = pyqtSignal(str, bool, float, float)
    
    # Manual control signals
    start_keys_vel = pyqtSignal(str, float)
    stop_keys_vel = pyqtSignal()
    
    # Navigation control
    stop_navigation = pyqtSignal()
    
    # Signal handling
    signal_button_pressed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sterowanie Robotem")
        # Optimized for 1920x1080 displays
        self.setFixedSize(1800, 1000)  # Large window for 1920x1080 screens
        self.showFullScreen()
        
        # Apply global font scaling for large screens (1920x1080)
        font = self.font()
        font.setPointSize(14)  # Much larger base font size for visibility
        self.setFont(font)
        
        # Apply global scaling optimized for large screens
        self.setStyleSheet("""
            QWidget {
                font-size: 14px;
            }
            QLabel {
                font-size: 14px;
            }
            QPushButton {
                font-size: 16px;
                min-height: 40px;
                padding: 8px 16px;
                font-weight: bold;
            }
            QComboBox {
                font-size: 14px;
                min-height: 35px;
                padding: 5px;
            }
            QListWidget {
                font-size: 14px;
            }
            QLineEdit {
                font-size: 14px;
                min-height: 35px;
                padding: 5px;
            }
            QTextEdit {
                font-size: 14px;
            }
            QGroupBox {
                font-size: 16px;
                font-weight: bold;
                padding-top: 25px;
                margin-top: 10px;
            }
            QTabWidget {
                font-size: 14px;
            }
            QTabBar::tab {
                font-size: 14px;
                min-height: 35px;
                padding: 8px 16px;
                font-weight: bold;
            }
        """)

        # self.setGeometry(100, 100, 300, 200)
        
        # self.map_view = MapView()
        self.map_view: MapView = MapView()
        
        # Initialize managers
        from roboto_viz.route_manager import RouteManager
        self.route_manager = RouteManager()
        self.dock_manager = DockManager()
        self.plan_manager = PlanManager()
        self.plan_executor = PlanExecutor(self.plan_manager)

        self.disconnected_view = DisconnectedView()
        # self.planner_view = PlannerView()
        self.active_view = ActiveView(self.map_view)  # Keep old view for compatibility
        self.plan_active_view = PlanActiveView(self.map_view, self.plan_manager, self.route_manager, self.dock_manager)
        self.maps_dir = Path.home() / ".robotroutes" / "maps"
        self.current_map_path = None
        self.current_yaml_path = None
        self.use_plan_system = True  # Flag to use new plan system
        
        # Robot position tracking for saving/loading (per-map)
        self.robot_x = 0.0
        self.robot_y = 0.0 
        self.robot_theta = 0.0
        self.current_map_name = None
        self.positions_dir = Path.home() / ".robotroutes" / "positions"
        
        # Timer for periodic position saving (every 10 seconds)
        self.position_save_timer = QTimer()
        self.position_save_timer.timeout.connect(self.save_robot_position)
        self.position_save_timer.start(10000)  # 10 seconds in milliseconds
        
        self.setup_ui()


    def setup_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        self.layout.setContentsMargins(0, 0, 0, 0)  # Remove margins
        self.layout.setSpacing(0)  # Remove spacing
        
        self.stacked_widget = QStackedWidget()
        self.layout.addWidget(self.stacked_widget)

        self.stacked_widget.addWidget(self.disconnected_view)
        self.stacked_widget.addWidget(self.active_view)
        self.stacked_widget.addWidget(self.plan_active_view)

        # Note: Route drawing is already handled in views.py
        # self.active_view.active_tools.draw_route -> map_view.display_bezier_route
        # self.active_view.active_tools.stop_drawing_points -> map_view.clear_route
        
        self.setup_plan_connections()

    def load_map(self, map_name: str) -> tuple[bool, str]:
        try:
            map_path = self.maps_dir / f"{map_name}.pgm"
            yaml_path = self.maps_dir / f"{map_name}.yaml"
            
            if not map_path.exists():
                error_msg = f"Map file not found: {map_path}"
                self.map_loaded_signal.emit(False, error_msg)
                return False, error_msg
                
            if not yaml_path.exists():
                error_msg = f"YAML file not found: {yaml_path}"
                self.map_loaded_signal.emit(False, error_msg)
                return False, error_msg
            
            with open(yaml_path, 'r') as file:
                yaml_data = yaml.safe_load(file)
                
            if 'origin' not in yaml_data:
                error_msg = f"Invalid YAML file: 'origin' key not found in {yaml_path}"
                self.map_loaded_signal.emit(False, error_msg)
                return False, error_msg
            
            # Store current paths
            self.current_map_path = map_path
            self.current_yaml_path = yaml_path
            
            # Load the map into the view
            self.map_view.load_image(str(map_path), yaml_data['origin'], yaml_data.get('resolution'))
            
            # Notify navigation system about map change
            self.map_changed_signal.emit(map_name)
            
            self.map_loaded_signal.emit(True, "")
        
            print(f"map name is {map_name}")

            # Track current map for position saving
            self.current_map_name = map_name

            # Try to load saved robot position for this map first, fallback to hardcoded positions
            if not self.load_robot_position(map_name):
                # Fallback to hardcoded positions if no saved position exists
                if map_name == 'pokoj1':
                    self.set_position_signal.emit(3.0,3.0,0.0)
                elif map_name == 'sypialnia':
                    self.set_position_signal.emit(1.0,1.0,0.0)
            return True, ""
        
            
        except yaml.YAMLError as e:
            error_msg = f"Error parsing YAML file: {str(e)}"
            self.map_loaded_signal.emit(False, error_msg)
            return False, error_msg
        except Exception as e:
            error_msg = f"Error loading map: {str(e)}"
            self.map_loaded_signal.emit(False, error_msg)
            return (False, error_msg)
    
    def setup_plan_connections(self):
        """Setup connections for the new plan system"""
        # Plan active view connections - forward to main view signals for GUI state machine
        self.plan_active_view.start_plan_execution.connect(self.start_plan_execution.emit)
        self.plan_active_view.stop_plan_execution.connect(self.stop_plan_execution.emit)
        self.plan_active_view.execute_plan_action.connect(self.execute_plan_action.emit)
        self.plan_active_view.map_load_requested.connect(self.load_map)
        
        # Plan executor connections
        self.plan_executor.start_nav.connect(self.start_nav.emit)
        self.plan_executor.dock_robot.connect(self.dock_robot.emit)
        self.plan_executor.dock_robot_at.connect(self.dock_robot_at.emit)
        self.plan_executor.undock_robot.connect(self.undock_robot.emit)
        self.plan_executor.action_completed.connect(self.plan_active_view.on_action_completed)
        self.plan_executor.execution_stopped.connect(self.plan_active_view.on_plan_execution_stopped)
        self.plan_executor.execution_stopped_due_to_failure.connect(self.plan_active_view.on_plan_execution_stopped_due_to_failure)
        self.plan_executor.single_action_completed.connect(self.plan_active_view.on_single_action_completed)
        
        # Connect navigation completion signal - this will be connected in GUI state machine
        # to the actual navigator.finished signal
        
        # Forward other signals
        self.plan_active_view.start_keys_vel.connect(self.start_keys_vel.emit)
        self.plan_active_view.stop_keys_vel.connect(self.stop_keys_vel.emit)
        self.plan_active_view.on_disconnection.connect(self.on_disconnection)
        self.plan_active_view.stop_navigation.connect(self.stop_navigation.emit)
        self.plan_active_view.signal_button_pressed.connect(self.signal_button_pressed.emit)
        self.plan_active_view.skip_to_wait_signal.connect(self.plan_executor.skip_to_wait_signal_action)
        
        # Connect signal button to plan executor
        self.signal_button_pressed.connect(self.plan_executor.on_signal_received)
        
        # Connect plan executor signals to UI
        self.plan_executor.waiting_for_signal.connect(self.on_waiting_for_signal)
        self.plan_executor.execution_stopped.connect(self.on_plan_stopped)
        self.plan_executor.execution_stopped_due_to_failure.connect(self.on_plan_stopped_due_to_failure)
        self.plan_executor.uart_signal_received.connect(self.on_uart_signal_received)
        
        # Connect plan executor status updates to plan status display
        self.plan_executor.status_update.connect(self.plan_active_view.set_plan_status)
        
        # Connect plan executor error status updates to robot status display (for errors like navigation failed)
        self.plan_executor.status_update.connect(self.update_robot_status_from_plan_executor)
        
    def on_plan_stopped(self, plan_name: str):
        """Handle when plan execution stops"""
        if self.use_plan_system:
            # Hide the signal button when plan stops
            self.plan_active_view.plan_tools.hide_signal_button()
    
    def on_plan_stopped_due_to_failure(self, plan_name: str, failure_reason: str):
        """Handle when plan execution stops due to failure - preserve failure status"""
        if self.use_plan_system:
            # Hide the signal button when plan stops
            self.plan_active_view.plan_tools.hide_signal_button()
            # Don't call any stop navigation - let the failure status remain displayed
            print(f"Plan stopped due to failure: {failure_reason} - preserving failure status")
        
    def on_waiting_for_signal(self, signal_name: str):
        """Handle when plan executor is waiting for a signal"""
        if self.use_plan_system:
            # Show the signal button in the plan tools
            self.plan_active_view.plan_tools.show_signal_button(signal_name)
    
    def on_uart_signal_received(self):
        """Handle when UART signal is received - immediately hide signal button"""
        if self.use_plan_system:
            # Force hide the signal button immediately when signal is received (bypassing protective conditions)
            self.plan_active_view.plan_tools.signal_btn.setVisible(False)
    
    def update_robot_status_from_plan_executor(self, status: str):
        """Update robot status display for error messages from plan executor"""
        if self.use_plan_system:
            # Only update robot status for error conditions, let normal status updates handle other cases
            if ("failed" in status.lower() or "error" in status.lower() or 
                "cancelled" in status.lower() or status in ["Failed", "Error"]):
                self.plan_active_view.set_current_status(status)


    def switch_to_disconnected(self):
        self.stacked_widget.setCurrentWidget(self.disconnected_view)
        self.disconnected_view.on_connection.connect(self.on_connection)

    def switch_to_planner(self):
        self.active_view.switch_to_planning()
        self.active_view.finish_planning.connect(self.on_finish_planning)

    def switch_to_active(self):
        if self.use_plan_system:
            self.stacked_widget.setCurrentWidget(self.plan_active_view)
            self.plan_active_view.switch_to_active()
            self.plan_active_view.map_view.goal_pose_set.connect(self.on_set_position)
        else:
            self.stacked_widget.setCurrentWidget(self.active_view)
            self.active_view.switch_to_active_tools()
            self.active_view.map_view.goal_pose_set.connect(self.on_set_position)
            self.active_view.start_planning.connect(self.on_start_planning)

    def switch_to_configuring(self):
        if self.use_plan_system:
            self.stacked_widget.setCurrentWidget(self.plan_active_view)
            self.plan_active_view.switch_to_configure()
            self.plan_active_view.map_view.goal_pose_set.connect(self.on_set_position)
        else:
            self.stacked_widget.setCurrentWidget(self.active_view)
            self.active_view.switch_to_configuring_tab()

    def on_connection(self):
        self.connection_signal.emit("connect emit from MainView")

    def on_disconnection(self):
        self.disconnection_signal.emit("disconnect emit from MainView")

    def on_set_position(self, x, y, theta):
        self.set_position_signal.emit(x,y,-theta)

    def on_start_planning(self):
        self.start_planning.emit()

    def on_finish_planning(self):
        self.finish_planning.emit()
    
    def update_robot_pose_plan_system(self, x: float, y: float, theta: float):
        """Update robot pose in plan system"""
        # Update internal position tracking
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
        
        if self.use_plan_system:
            self.plan_active_view.update_robot_pose(x, y, theta)
            self.plan_executor.update_robot_pose(x, y, theta)
    
    def load_routes_into_route_manager(self, routes: dict):
        """Load routes into the route manager for plan system"""
        if self.use_plan_system and hasattr(self, 'route_manager'):
            # The route manager will handle this when maps are loaded
            pass
    
    def load_maps_into_plan_system(self, maps: list):
        """Load maps into plan system"""
        if self.use_plan_system:
            # Maps are handled through route manager in plan system
            pass
    
    def save_robot_position(self):
        """Save current robot position to map-specific file every 10 seconds"""
        if not self.current_map_name:
            return  # No map loaded, can't save position
            
        try:
            # Ensure the directory exists
            self.positions_dir.mkdir(parents=True, exist_ok=True)
            
            position_file = self.positions_dir / f"{self.current_map_name}_position.txt"
            
            # Save in simple text format for easy reading
            position_text = f"x={self.robot_x:.3f}\ny={self.robot_y:.3f}\ntheta={self.robot_theta:.3f}\ntimestamp={time.time():.0f}\n"
            
            with open(position_file, 'w') as f:
                f.write(position_text)
                
        except Exception as e:
            print(f"Error saving robot position for map '{self.current_map_name}': {e}")
    
    def load_robot_position(self, map_name: str):
        """Load saved robot position for specific map"""
        try:
            position_file = self.positions_dir / f"{map_name}_position.txt"
            
            if position_file.exists():
                with open(position_file, 'r') as f:
                    lines = f.readlines()
                
                # Parse the text file
                x, y, theta = 0.0, 0.0, 0.0
                for line in lines:
                    line = line.strip()
                    if line.startswith('x='):
                        x = float(line.split('=')[1])
                    elif line.startswith('y='):
                        y = float(line.split('=')[1])
                    elif line.startswith('theta='):
                        theta = float(line.split('=')[1])
                
                print(f"Loading saved position for map '{map_name}': x={x}, y={y}, theta={theta}")
                # Apply same negation as on_set_position to maintain consistency
                self.set_position_signal.emit(x, y, -theta)
                return True
        except Exception as e:
            print(f"Error loading robot position for map '{map_name}': {e}")
        
        return False

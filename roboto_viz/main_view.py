
from __future__ import annotations
import sys
import os
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QPushButton, QStackedWidget, QLabel, QHBoxLayout, QListWidget, QListWidgetItem, QComboBox, QGridLayout
from abc import ABC, abstractmethod
import yaml
from PyQt5.QtCore import Qt, pyqtSignal, QObject, pyqtSlot

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
        # self.showFullScreen()
        self.setWindowTitle("Robot Control")
        self.setFixedSize(1024, 550)  # Ensure window fits your screen exactly
        self.showFullScreen()

        # self.setGeometry(100, 100, 300, 200)
        
        # self.map_view = MapView()
        self.map_view: MapView = MapView()
        
        # Initialize managers
        from roboto_viz.route_manager import RouteManager
        self.route_manager = RouteManager()
        self.plan_manager = PlanManager()
        self.plan_executor = PlanExecutor(self.plan_manager)

        self.disconnected_view = DisconnectedView()
        # self.planner_view = PlannerView()
        self.active_view = ActiveView(self.map_view)  # Keep old view for compatibility
        self.plan_active_view = PlanActiveView(self.map_view, self.plan_manager, self.route_manager)
        self.maps_dir = Path.home() / ".robotroutes" / "maps"
        self.current_map_path = None
        self.current_yaml_path = None
        self.use_plan_system = True  # Flag to use new plan system
        self.setup_ui()


    def setup_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        
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
            self.map_view.load_image(str(map_path), yaml_data['origin'])
            
            # Notify navigation system about map change
            self.map_changed_signal.emit(map_name)
            
            self.map_loaded_signal.emit(True, "")
        
            print(f"map name is {map_name}")

            if map_name == 'pokoj1':
                self.set_position_signal.emit(3.0,3.0,0.0)
            if map_name == 'sypialnia':
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
        # Plan active view connections
        self.plan_active_view.start_plan_execution.connect(self.plan_executor.start_plan_execution)
        self.plan_active_view.stop_plan_execution.connect(self.plan_executor.stop_plan_execution)
        self.plan_active_view.execute_plan_action.connect(self.plan_executor.execute_action)
        self.plan_active_view.map_load_requested.connect(self.load_map)
        
        # Plan executor connections
        self.plan_executor.start_nav.connect(self.start_nav.emit)
        self.plan_executor.dock_robot.connect(self.dock_robot.emit)
        self.plan_executor.undock_robot.connect(self.undock_robot.emit)
        self.plan_executor.action_completed.connect(self.plan_active_view.on_action_completed)
        self.plan_executor.execution_stopped.connect(self.plan_active_view.on_plan_execution_stopped)
        self.plan_executor.single_action_completed.connect(self.plan_active_view.on_single_action_completed)
        
        # Connect navigation completion signal - this will be connected in GUI state machine
        # to the actual navigator.finished signal
        
        # Forward other signals
        self.plan_active_view.start_keys_vel.connect(self.start_keys_vel.emit)
        self.plan_active_view.stop_keys_vel.connect(self.stop_keys_vel.emit)
        self.plan_active_view.on_disconnection.connect(self.on_disconnection)
        self.plan_active_view.stop_navigation.connect(self.stop_navigation.emit)
        self.plan_active_view.signal_button_pressed.connect(self.signal_button_pressed.emit)
        
        # Connect signal button to plan executor
        self.signal_button_pressed.connect(self.plan_executor.on_signal_received)
        
        # Connect plan executor signals to UI
        self.plan_executor.waiting_for_signal.connect(self.on_waiting_for_signal)
        self.plan_executor.execution_stopped.connect(self.on_plan_stopped)
        
    def on_plan_stopped(self, plan_name: str):
        """Handle when plan execution stops"""
        if self.use_plan_system:
            # Hide the signal button when plan stops
            self.plan_active_view.plan_tools.hide_signal_button()
        
    def on_waiting_for_signal(self, signal_name: str):
        """Handle when plan executor is waiting for a signal"""
        if self.use_plan_system:
            # Show the signal button in the plan tools
            self.plan_active_view.plan_tools.show_signal_button(signal_name)


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


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

class MainView(QMainWindow):
    map_loaded_signal = pyqtSignal(bool, str)  # success, error_message
    connection_signal = pyqtSignal(str)
    disconnection_signal = pyqtSignal(str)
    set_position_signal = pyqtSignal(float, float, float)

    start_planning = pyqtSignal()
    finish_planning = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control")
        self.setGeometry(100, 100, 300, 200)
        
        # self.map_view = MapView()
        self.map_view: MapView = MapView()

        self.disconnected_view = DisconnectedView()
        # self.planner_view = PlannerView()
        self.active_view = ActiveView(self.map_view)
        self.maps_dir = Path.home() / ".robotroutes" / "maps"
        self.current_map_path = None
        self.current_yaml_path = None
        self.setup_ui()


    def setup_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        
        self.stacked_widget = QStackedWidget()
        self.layout.addWidget(self.stacked_widget)

        self.stacked_widget.addWidget(self.disconnected_view)
        self.stacked_widget.addWidget(self.active_view)

        self.active_view.active_tools.draw_points.connect(self.map_view.display_points)
        self.active_view.active_tools.stop_drawing_points.connect(self.map_view.clear_points)

        self.active_view.planning_tools.draw_points.connect(self.map_view.display_points)
        self.active_view.planning_tools.stop_drawing_points.connect(self.map_view.clear_points)

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
            
            self.map_loaded_signal.emit(True, "")
            return True, ""
            
        except yaml.YAMLError as e:
            error_msg = f"Error parsing YAML file: {str(e)}"
            self.map_loaded_signal.emit(False, error_msg)
            return False, error_msg
        except Exception as e:
            error_msg = f"Error loading map: {str(e)}"
            self.map_loaded_signal.emit(False, error_msg)
            return (False, error_msg)

    def switch_to_disconnected(self):
        self.stacked_widget.setCurrentWidget(self.disconnected_view)
        self.disconnected_view.on_connection.connect(self.on_connection)

    def switch_to_planner(self):
        self.active_view.switch_to_planning()
        self.active_view.finish_planning.connect(self.on_finish_planning)

    def switch_to_active(self):
        self.stacked_widget.setCurrentWidget(self.active_view)
        self.active_view.switch_to_active_tools()
        self.active_view.map_view.goal_pose_set.connect(self.on_set_position)
        self.active_view.start_planning.connect(self.on_start_planning)

    def switch_to_configuring(self):
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

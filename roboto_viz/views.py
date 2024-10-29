
from __future__ import annotations
import sys
import os
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QPushButton, QStackedWidget, QLabel, QHBoxLayout, QListWidget, QListWidgetItem
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
        """
        Load a map from the .robotroutes/maps directory.
        
        Args:
            map_name: Name of the map (without extension)
            
        Returns:
            tuple[bool, str]: (Success flag, Error message if failed or empty string if successful)
        """
        try:
            # Construct paths
            map_path = self.maps_dir / f"{map_name}.pgm"
            yaml_path = self.maps_dir / f"{map_name}.yaml"
            
            # Check if files exist
            if not map_path.exists():
                error_msg = f"Map file not found: {map_path}"
                self.map_loaded_signal.emit(False, error_msg)
                return False, error_msg
                
            if not yaml_path.exists():
                error_msg = f"YAML file not found: {yaml_path}"
                self.map_loaded_signal.emit(False, error_msg)
                return False, error_msg
            
            # Load YAML data
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
        # self.stacked_widget.setCurrentWidget(self.planner_view)
        self.active_view.finish_planning.connect(self.on_finish_planning)
        # self.connected_view.on_disconnection.connect(self.on_disconnection)

    def switch_to_active(self):
        self.stacked_widget.setCurrentWidget(self.active_view)
        self.active_view.switch_to_active_tools()
        self.active_view.map_view.goal_pose_set.connect(self.on_set_position)
        self.active_view.start_planning.connect(self.on_start_planning)

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

class DisconnectedView(QWidget):
    on_connection = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        # self.main_view = main_view
        self.setup_ui()
        self.wait_for_connection: bool = False

    def setup_ui(self):
        layout = QVBoxLayout(self)

        self.service_state = QLabel("Service: Not Available")
        
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.on_connect)
        self.connect_button.setEnabled(False)

        layout.addWidget(self.service_state)
        layout.addWidget(self.connect_button)

    def on_connect(self):
        self.on_connection.emit("connect on Widget")

    def set_availability(self, available):
        if not self.wait_for_connection:
            status = "Available" if available else "Not Available"
            self.service_state.setText(f"Service: {status}")
            self.connect_button.setEnabled(available)

    def waiting_for_connection(self, state: bool):
        if state:
            self.connect_button.setEnabled(False)
            self.service_state.setText("Service: connecting")
        self.wait_for_connection = state

class ActiveTools(QWidget):
    on_disconnect = pyqtSignal()
    start_planning = pyqtSignal()

    save_current_routes = pyqtSignal(dict)

    draw_points = pyqtSignal(list)
    stop_drawing_points = pyqtSignal()

    start_nav = pyqtSignal(str, bool)
    stop_nav = pyqtSignal()

    def __init__(self, routes: dict):
        super().__init__()

        self.routes = routes

        # TABS
        main_layout = QVBoxLayout()
        tab_widget = QTabWidget()
        
        # Create first tab
        operation_tab = QWidget()
        first_layout = QVBoxLayout(operation_tab)
        self.route_list = QListWidget()


        self.route_list.setFixedHeight(200)
        first_layout.addWidget(self.route_list)
        
        # Create button layout for first row
        button_layout_1 = QHBoxLayout()
        self.button_add = QPushButton("Add Route")
        self.button_remove = QPushButton("Remove Route")
        button_layout_1.addWidget(self.button_add)
        button_layout_1.addWidget(self.button_remove)
        first_layout.addLayout(button_layout_1)
        
        # Create button layout for second row
        button_layout_2 = QHBoxLayout()
        self.button_set_active = QPushButton("Set Active")
        button_layout_2.addWidget(self.button_set_active)
        first_layout.addLayout(button_layout_2)
        
        # Add spacing
        # first_layout.addSpacing(20)
        first_layout.addStretch()

        # Add the original navigation buttons
        self.button_go_to_base = QPushButton("Go to base")
        self.button_go_to_dest = QPushButton("Go to dest")
        self.button_stop = QPushButton("Stop")
        first_layout.addWidget(self.button_go_to_base)
        first_layout.addWidget(self.button_go_to_dest)
        first_layout.addWidget(self.button_stop)
        
        tab_widget.addTab(operation_tab, "Operation")
        
        # Connect button signals to slots
        self.button_add.clicked.connect(lambda: self.start_planning.emit())
        self.button_remove.clicked.connect(self.remove_route)
        self.button_set_active.clicked.connect(self.set_active_route)

        # Change to a separate callback method, use bools for direction
        self.button_go_to_dest.clicked.connect(self.handle_navigate_to_dest)
        self.button_go_to_base.clicked.connect(self.handle_navigate_to_base)
        self.button_stop.clicked.connect(lambda: self.stop_nav.emit())

        # Create second tab (unchanged)
        planning_tab = QWidget()
        second_layout = QVBoxLayout(planning_tab)
        for i in range(2):
            button = QPushButton(f"Button {i+1} (Tab 2)")
            second_layout.addWidget(button)
        tab_widget.addTab(planning_tab, "Planning")
        
        main_layout.addWidget(tab_widget)
        
        self.setLayout(main_layout)

        # Keep track of active route, MOVE TO NAVDATA
        self.active_route = None

    def handle_navigate_to_dest(self):
        if self.active_route:
            self.start_nav.emit(self.active_route.text()[2:], True)

    def handle_navigate_to_base(self):
        if self.active_route:
            self.start_nav.emit(self.active_route.text()[2:], False)


    # def add_route(self):
    #     """Add a new example route to the list"""
    #     # count = self.route_list.count()
    #     # self.route_list.addItem(f"New Route {count + 1}")
    #     self.start_planning.emit()

    def update_routes(self):
        """Load routes from received list"""
        # Clear existing routes
        self.route_list.clear()
        self.active_route = None
        
        # Add new routes
        for route in list(self.routes.keys()):
            item = QListWidgetItem(route)
            self.route_list.addItem(item)

        self.stop_drawing_points.emit()


    def remove_route(self):
        """Remove the currently selected route"""
        current_item = self.route_list.currentItem()
        if current_item:
            route_name = current_item.text()
            if route_name.startswith("✓ "):
                route_name = route_name[2:]  # Remove the tick if present
            
            # If removing active route, clear the active route
            if current_item == self.active_route:
                self.active_route = None
            
            self.routes.pop(route_name)  # Use the clean route name
            self.update_routes()
            # self.route_list.takeItem(self.route_list.row(current_item))
            
            self.save_current_routes.emit(self.routes)
            self.stop_drawing_points.emit()


    def set_active_route(self):
        """Move the selected route to the top of the list and mark it with a green tick"""
        current_item = self.route_list.currentItem()
        if current_item:
            # Clear previous active route's tick
            if self.active_route:
                old_text = self.active_route.text()
                if old_text.startswith("✓ "):
                    self.active_route.setText(old_text[2:])
            
            # Get the current row
            current_row = self.route_list.row(current_item)
            
            # Take the item from the list
            item = self.route_list.takeItem(current_row)
            
            # Add tick to the text if it doesn't already have one
            if not item.text().startswith("✓ "):
                item.setText("✓ " + item.text())
            
            # Insert it at the top
            self.route_list.insertItem(0, item)
            # Select the moved item
            self.route_list.setCurrentRow(0)
            
            # Update the active route reference
            self.active_route = item
            
            # Optional: Set text color to green for active route
            item.setForeground(Qt.green)
            
            # Reset color of other items
            for i in range(1, self.route_list.count()):
                other_item = self.route_list.item(i)
                other_item.setForeground(Qt.black)
            
            self.draw_points.emit(self.routes[current_item.text()[2:]])
            # Draw on the map
            

class PlanningTools(QWidget):
    finish_planning = pyqtSignal()
    save_current_routes = pyqtSignal(dict)

    draw_points = pyqtSignal(list)
    stop_drawing_points = pyqtSignal()

    def __init__(self, routes: dict):
        super().__init__()

        self.routes: dict = routes

        self.new_route: list = list()

        self.setup_ui()
        
    def setup_ui(self):
        self.route_name = QLineEdit(self)
        self.main_layout = QVBoxLayout()
         
        self.done_button = QPushButton("Done")
        self.cancel_button = QPushButton("Cancel")

        self._last_point = None  # Store last point to prevent duplicates

        self.done_button.clicked.connect(self.exit_done)
        self.cancel_button.clicked.connect(self.exit_cancel)

        # layout.addWidget(self.map_view, 3)
        # layout.addWidget(self.finish_planning_button, 1)
        self.main_layout.addWidget(self.route_name)
        self.main_layout.addStretch()
        self.main_layout.addWidget(self.done_button)
        self.main_layout.addWidget(self.cancel_button)
        self.setLayout(self.main_layout)

    pyqtSlot(float,float,float)
    def setPoint(self, x, y, theta):
        # Check if this is a duplicate point
        new_point = [x, y, 0.0, theta]
        if self._last_point != new_point:  # Only append if different
            self.new_route.append(new_point)
            self._last_point = new_point
            self.draw_points.emit(self.new_route)

    def exit_cancel(self):
        self.new_route.clear()
        self.stop_drawing_points.emit()
        self.finish_planning.emit()
    
    def exit_done(self):
        if self.route_name.text().strip():  # Only save if name is not empty
            self.routes[self.route_name.text()] = self.new_route.copy()  # Make a copy
            self.save_current_routes.emit(self.routes)
            self.new_route.clear()
            self._last_point = None
            self.stop_drawing_points.emit()
            self.finish_planning.emit()
        

class ActiveView(QWidget):
    finish_planning = pyqtSignal()
    on_disconnection = pyqtSignal(str)
    start_planning = pyqtSignal()

    def __init__(self, map_view : MapView):
        super().__init__()

        self.map_view = map_view

        self.routes: dict = dict()

        self.active_tools = ActiveTools(self.routes)
        self.planning_tools = PlanningTools(self.routes)

        self.stacked_widget = QStackedWidget()
        # self.layout.addWidget(self.stacked_widget)

        self.stacked_widget.addWidget(self.active_tools)
        self.stacked_widget.addWidget(self.planning_tools)

        self.main_layout = QHBoxLayout()
        self.main_layout.addWidget(self.map_view, 3)
        self.main_layout.addWidget(self.stacked_widget, 1)
        self.setLayout(self.main_layout)

    pyqtSlot(dict)
    def load__routes(self, routes):
        self.routes.clear()
        self.routes.update(routes) 
        self.active_tools.update_routes()

    def switch_to_active_tools(self):
        self.stacked_widget.setCurrentWidget(self.active_tools)
        
        self.active_tools.on_disconnect.connect(lambda: self.on_disconnection.emit())
        self.active_tools.start_planning.connect(lambda: self.start_planning.emit())

    def switch_to_planning(self):
        self.stacked_widget.setCurrentWidget(self.planning_tools)

        # self.stacked_widget.setCurrentWidget(self.planning_tools)
        self.planning_tools.finish_planning.connect(lambda: self.finish_planning.emit())

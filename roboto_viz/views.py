
from __future__ import annotations
import sys
import os
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QPushButton, QStackedWidget, QLabel, QHBoxLayout, QListWidget
from abc import ABC, abstractmethod
import yaml
from PyQt5.QtCore import Qt, pyqtSignal, QObject

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger


from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QTabWidget
# internal imports:
from roboto_viz.map_view import MapView
from roboto_viz.goal_arrow import GoalArrow
from roboto_viz.robot_item import RobotItem


class MainView(QMainWindow):
    connection_signal = pyqtSignal(str)
    disconnection_signal = pyqtSignal(str)
    set_position_signal = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control")
        self.setGeometry(100, 100, 300, 200)
        
        self.map_view = MapView()

        self.disconnected_view = DisconnectedView()
        self.connected_view = ConnectedView()
        self.active_view = ActiveView(self.map_view)

        self.setup_ui()


    def setup_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        
        self.stacked_widget = QStackedWidget()
        self.layout.addWidget(self.stacked_widget)

        self.stacked_widget.addWidget(self.disconnected_view)
        self.stacked_widget.addWidget(self.connected_view)
        self.stacked_widget.addWidget(self.active_view)

    def switch_to_disconnected(self):
        self.stacked_widget.setCurrentWidget(self.disconnected_view)
        self.disconnected_view.on_connection.connect(self.on_connection)

    def switch_to_connected(self):
        self.stacked_widget.setCurrentWidget(self.connected_view)
        self.connected_view.on_disconnection.connect(self.on_disconnection)

    def switch_to_active(self):
        self.stacked_widget.setCurrentWidget(self.active_view)
        self.active_view.map_view.goal_pose_set.connect(self.on_set_position)

    def on_connection(self):
        self.connection_signal.emit("connect emit from MainView")

    def on_disconnection(self):
        self.disconnection_signal.emit("disconnect emit from MainView")

    def on_set_position(self, x, y, theta):
        self.set_position_signal.emit(x,y,-theta)

class DisconnectedView(QWidget):
    on_connection = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        # self.main_view = main_view
        self.setup_ui()

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
        status = "Available" if available else "Not Available"
        self.service_state.setText(f"Service: {status}")
        self.connect_button.setEnabled(available)

class ConnectedView(QWidget):
    on_disconnection = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        # self.main_view = main_view
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.on_disconnect)
        layout.addWidget(self.disconnect_button)

    def on_disconnect(self):
        print("Disconnecting...")
        self.on_disconnection.emit("disconnect on Widget")
        # self.main_view.switch_to_connecting()

class ActiveView(QWidget):
    on_disconnection = pyqtSignal(str)
    def __init__(self, map_view: MapView):
        super().__init__()

        self.map_view = map_view

        self.setup_ui()

    def setup_ui(self):
        package_name = 'roboto_diffbot'
        package_path = get_package_share_directory(package_name)
        self.image_path = os.path.join(package_path, 'sim', 'map', 'my_map.pgm')
        self.origin_path = os.path.join(package_path, 'sim', 'map', 'my_map.yaml')
        
        with open(self.origin_path, 'r') as file:
            origin_data = yaml.safe_load(file)

        self.map_origin = origin_data['origin']

        self.map_view.load_image(self.image_path, self.map_origin)

        # TABS
        tab_widget = QTabWidget()
        
        # Create first tab
        operation_tab = QWidget()
        first_layout = QVBoxLayout(operation_tab)
        self.route_list = QListWidget()
        # Add example routes
        example_routes = ["Route A: Warehouse to Loading Dock", 
                         "Route B: Assembly Line to Storage",
                         "Route C: Main Entrance to Office"]
        self.route_list.setFixedHeight(200)
        self.route_list.addItems(example_routes)
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
        self.button_add.clicked.connect(self.add_route)
        self.button_remove.clicked.connect(self.remove_route)
        self.button_set_active.clicked.connect(self.set_active_route)
        
        # Create second tab (unchanged)
        planning_tab = QWidget()
        second_layout = QVBoxLayout(planning_tab)
        for i in range(2):
            button = QPushButton(f"Button {i+1} (Tab 2)")
            second_layout.addWidget(button)
        tab_widget.addTab(planning_tab, "Planning")
        
        main_layout = QHBoxLayout()
        main_layout.addWidget(self.map_view, 3)
        main_layout.addWidget(tab_widget, 1)
        self.setLayout(main_layout)

        # Keep track of active route, MOVE TO NAVDATA
        self.active_route = None
        
    def on_disconnect(self):
        print("Disconnecting")
        self.on_disconnection.emit("disconnection")

    def add_route(self):
        """Add a new example route to the list"""
        count = self.route_list.count()
        self.route_list.addItem(f"New Route {count + 1}")
        
    def remove_route(self):
        """Remove the currently selected route"""
        current_item = self.route_list.currentItem()
        if current_item:
            # If removing active route, clear the active route
            if current_item == self.active_route:
                self.active_route = None
            self.route_list.takeItem(self.route_list.row(current_item))

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

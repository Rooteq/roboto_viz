
from __future__ import annotations
import sys
import os
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QPushButton, QStackedWidget, QLabel, QHBoxLayout
from abc import ABC, abstractmethod
import yaml
from PyQt5.QtCore import Qt, pyqtSignal, QObject

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger


from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
# internal imports:
from roboto_viz.map_view import MapView
from roboto_viz.goal_arrow import GoalArrow
from roboto_viz.robot_item import RobotItem


class MainView(QMainWindow):
    connection_signal = pyqtSignal(str)
    disconnection_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control")
        self.setGeometry(100, 100, 300, 200)

        self.disconnected_view = DisconnectedView()
        self.connected_view = ConnectedView()
        self.active_view = ActiveView()

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

    def on_connection(self):
        self.connection_signal.emit("connect emit from MainView")

    def on_disconnection(self):
        self.disconnection_signal.emit("disconnect emit from MainView")

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
    def __init__(self):
        super().__init__()

        self.map_view = MapView()

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
        # self.map_view.goal_pose_set.connect(self.handle_goal_pose)
        # self.map_view.mouse_moved.connect(self.print_coordinates)  # Connect to the new signal
        # self.map_view.mouse_clicked.connect(self.print_coordinates)

        # self.clicks_label = QLabel("Counting: 0 clicks", self)
        # self.clicks_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        # self.step_label = QLabel("Long-Running Step: 0")
        # self.step_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        # self.count_btn = QPushButton("Click me!", self)
        # self.count_btn.clicked.connect(self.countClicks)
        # self.long_running_btn = QPushButton("Long-Running Task!", self)
        # self.long_running_btn.clicked.connect(self.runLongTask)

        self.disconnect_btn = QPushButton("Disconnect", self)

        button_layout = QVBoxLayout()
        button_layout.addWidget(self.disconnect_btn)
        # button_layout.addWidget(self.clicks_label)
        # button_layout.addWidget(self.count_btn)
        # button_layout.addStretch()
        # button_layout.addWidget(self.step_label)
        # button_layout.addWidget(self.long_running_btn)

        main_layout = QHBoxLayout()
        main_layout.addWidget(self.map_view, 3)
        main_layout.addLayout(button_layout, 1)

        self.setLayout(main_layout)
        # self.centralWidget.setLayout(main_layout)
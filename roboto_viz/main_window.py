
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QPushButton, QHBoxLayout, QVBoxLayout
from ament_index_python.packages import get_package_share_directory
import os
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import math
from roboto_viz.map_view import MapView

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
import yaml
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class NavigatorData:
    def __init__(self):
        self.navigator = BasicNavigator()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = -2.0
        self.goal_pose.pose.position.y = -0.5
        self.goal_pose.pose.orientation.w = 1.0
    
    def update_pose(self):
        pass

class Worker(QThread):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def __init__(self, nav_data : NavigatorData):
        super().__init__()
        self.nav_data = nav_data

    def run(self):
        self.nav_data.goal_pose.header.stamp = self.nav_data.navigator.get_clock().now().to_msg()
        self.nav_data.navigator.goToPose(self.nav_data.goal_pose)
        self.nav_data.navigator.waitUntilNav2Active()

        while not self.nav_data.navigator.isTaskComplete():
            feedback = self.nav_data.navigator.getFeedback()

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.nav_data.navigator.cancelTask()

        self.finished.emit()


class PoseUpdater(QThread):
    update_pose = pyqtSignal(float, float, float)

    def __init__(self, executor: MultiThreadedExecutor):
        super().__init__()
        self.executor = executor
        self.node = rclpy.create_node('sub_to_qt')

        self.sub = self.node.create_subscription(
            TwistStamped,
            'diffbot_pose',
            self.pose_callback,
            10
        )

        self.executor.add_node(self.node)

    def run(self):
        self.executor.spin()
        # rclpy.spin(self.node)

    def pose_callback(self, msg):
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        # theta = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        theta = -msg.twist.angular.z

        self.update_pose.emit(x, y, theta)

class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.nav_data = NavigatorData()

        self.executor = MultiThreadedExecutor()

        self.clicks_count = 0
        
        self.setupUi()
        self.setupPoseSubscriber()

    def setupUi(self):
        self.setWindowTitle("Map Viewer")
        self.resize(800, 600)
        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)

        package_name = 'roboto_diffbot'
        package_path = get_package_share_directory(package_name)
        self.image_path = os.path.join(package_path, 'sim', 'map', 'my_map.pgm')
        self.origin_path = os.path.join(package_path, 'sim', 'map', 'my_map.yaml')
        
        with open(self.origin_path, 'r') as file:
            origin_data = yaml.safe_load(file)

        self.map_origin = origin_data['origin']

        self.map_view = MapView(self)
        self.map_view.load_image(self.image_path, self.map_origin)
        self.map_view.goal_pose_set.connect(self.handle_goal_pose)
        # self.map_view.mouse_moved.connect(self.print_coordinates)  # Connect to the new signal
        # self.map_view.mouse_clicked.connect(self.print_coordinates)

        self.clicks_label = QLabel("Counting: 0 clicks", self)
        self.clicks_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.step_label = QLabel("Long-Running Step: 0")
        self.step_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.count_btn = QPushButton("Click me!", self)
        self.count_btn.clicked.connect(self.countClicks)
        self.long_running_btn = QPushButton("Long-Running Task!", self)
        self.long_running_btn.clicked.connect(self.runLongTask)

        button_layout = QVBoxLayout()
        button_layout.addWidget(self.clicks_label)
        button_layout.addWidget(self.count_btn)
        button_layout.addStretch()
        button_layout.addWidget(self.step_label)
        button_layout.addWidget(self.long_running_btn)

        main_layout = QHBoxLayout()
        main_layout.addWidget(self.map_view, 3)
        main_layout.addLayout(button_layout, 1)

        self.centralWidget.setLayout(main_layout)

    def countClicks(self):
        self.clicks_count += 1
        self.clicks_label.setText(f"Counting: {self.clicks_count} clicks")

    def reportProgress(self, n):
        self.step_label.setText(f"Long-Running Step: {n}")

    def runLongTask(self):
        self.worker = Worker(self.nav_data)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker.progress.connect(self.reportProgress)
        self.worker.start()

        self.long_running_btn.setEnabled(False)
        self.worker.finished.connect(lambda: self.long_running_btn.setEnabled(True))
        self.worker.finished.connect(lambda: self.step_label.setText("Long-Running Step: 0"))

    def setupPoseSubscriber(self):
        self.pos_updater = PoseUpdater(self.executor)
        self.pos_updater.update_pose.connect(self.update_robot)
        self.pos_updater.start()

    # def print_coordinates(self, x, y):
    #     print(f"Mouse position: ({x:.2f}, {y:.2f})")

    def handle_goal_pose(self, x, y, theta):
        print(f"New goal pose set: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
        
        # Here you can update your navigation goal or perform any other necessary actions

        self.nav_data.goal_pose.pose.position.x = x
        self.nav_data.goal_pose.pose.position.y = y
        self.nav_data.goal_pose.pose.orientation.z = math.sin(theta / 2)
        self.nav_data.goal_pose.pose.orientation.w = math.cos(theta / 2)

        self.worker = Worker(self.nav_data)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker.progress.connect(self.reportProgress)
        self.worker.start()

        self.long_running_btn.setEnabled(False)
        self.worker.finished.connect(lambda: self.long_running_btn.setEnabled(True))
        self.worker.finished.connect(lambda: self.step_label.setText("Long-Running Step: 0"))

    def update_robot(self, x, y, theta):
        self.map_view.update_robot_pose(x,y,theta)

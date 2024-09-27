
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QPushButton, QHBoxLayout, QVBoxLayout
from ament_index_python.packages import get_package_share_directory
import os
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

from roboto_viz.map_view import MapView

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

    def __init__(self, nav_data):
        super().__init__()
        self.nav_data = nav_data

    def run(self):
        self.nav_data.goal_pose.header.stamp = self.nav_data.navigator.get_clock().now().to_msg()
        self.nav_data.navigator.goToPose(self.nav_data.goal_pose)
        self.nav_data.navigator.waitUntilNav2Active()

        i = 0
        while not self.nav_data.navigator.isTaskComplete():
            i += 1
            feedback = self.nav_data.navigator.getFeedback()
            if feedback and i % 5 == 0:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f'Estimated time of arrival: {eta:.0f} seconds.')
                self.progress.emit(i)

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.nav_data.navigator.cancelTask()

        self.finished.emit()

class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.nav_data = NavigatorData()
        self.clicks_count = 0
        self.setupUi()

    def setupUi(self):
        self.setWindowTitle("Map Viewer")
        self.resize(800, 600)
        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)

        package_name = 'roboto_diffbot'
        package_path = get_package_share_directory(package_name)
        self.image_path = os.path.join(package_path, 'sim', 'map', 'my_map.pgm')

        self.map_view = MapView(self)
        self.map_view.load_image(self.image_path)

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
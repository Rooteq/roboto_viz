#!/usr/bin/env python3

import sys
import subprocess
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton,
                             QVBoxLayout, QWidget, QLabel, QMessageBox)
from PyQt5.QtCore import Qt, QProcess
from PyQt5.QtGui import QFont


class LauncherApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Launcher")
        self.setFixedSize(800, 600)

        # Store process references
        self.gui_process = None
        self.diffbot_process = None

        # Apply large font scaling for 1920x1080 screens
        font = self.font()
        font.setPointSize(16)
        self.setFont(font)

        self.setStyleSheet("""
            QWidget {
                font-size: 16px;
            }
            QPushButton {
                font-size: 20px;
                min-height: 80px;
                min-width: 400px;
                padding: 20px;
                font-weight: bold;
                background-color: #4CAF50;
                color: white;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
            QLabel {
                font-size: 18px;
                color: #333333;
            }
        """)

        self.setup_ui()

    def setup_ui(self):
        """Setup the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(30)

        # Title label
        title_label = QLabel("Robot Control System")
        title_font = QFont()
        title_font.setPointSize(24)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # Add some spacing
        layout.addSpacing(50)

        # Launch button
        self.launch_btn = QPushButton("Uruchom tryb jazdy")
        self.launch_btn.clicked.connect(self.launch_robot_system)
        layout.addWidget(self.launch_btn, alignment=Qt.AlignCenter)

        # Status label
        self.status_label = QLabel("Gotowy do uruchomienia")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # Add stretch to push everything to center
        layout.addStretch()

    def launch_robot_system(self):
        """Launch both ROS2 packages"""
        try:
            # Source ROS2 environment and launch diffbot
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            workspace_setup = "/home/rooteq/ros2_ws/install/setup.bash"

            # Launch roboto_diffbot first
            diffbot_cmd = f"bash -c 'source {ros2_setup} && source {workspace_setup} && ros2 launch roboto_diffbot launch_roboto.launch.py'"
            self.diffbot_process = QProcess(self)
            self.diffbot_process.start("bash", ["-c", diffbot_cmd])

            # Wait a moment for diffbot to start
            QApplication.processEvents()

            # Launch GUI
            gui_cmd = f"bash -c 'source {ros2_setup} && source {workspace_setup} && ros2 launch roboto_viz gui_launch.py'"
            self.gui_process = QProcess(self)
            self.gui_process.finished.connect(self.on_gui_closed)
            self.gui_process.start("bash", ["-c", gui_cmd])

            # Update UI
            self.launch_btn.setEnabled(False)
            self.status_label.setText("System uruchomiony")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to launch: {str(e)}")
            self.status_label.setText("Błąd uruchamiania")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")

    def on_gui_closed(self):
        """Handle GUI process closure - terminate diffbot process too"""
        print("GUI closed, terminating diffbot process...")

        if self.diffbot_process and self.diffbot_process.state() == QProcess.Running:
            self.diffbot_process.terminate()
            # Wait up to 5 seconds for graceful termination
            if not self.diffbot_process.waitForFinished(5000):
                # Force kill if it doesn't terminate
                self.diffbot_process.kill()

        # Re-enable launch button
        self.launch_btn.setEnabled(True)
        self.status_label.setText("Gotowy do uruchomienia")
        self.status_label.setStyleSheet("color: #333333; font-weight: normal;")

    def closeEvent(self, event):
        """Handle application close - make sure all processes are terminated"""
        if self.gui_process and self.gui_process.state() == QProcess.Running:
            self.gui_process.terminate()
            self.gui_process.waitForFinished(5000)

        if self.diffbot_process and self.diffbot_process.state() == QProcess.Running:
            self.diffbot_process.terminate()
            self.diffbot_process.waitForFinished(5000)

        event.accept()


def main():
    app = QApplication(sys.argv)
    launcher = LauncherApp()
    launcher.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

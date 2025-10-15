#!/usr/bin/env python3

import sys
import subprocess
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton,
                             QVBoxLayout, QHBoxLayout, QWidget, QLabel, QMessageBox)
from PyQt5.QtCore import Qt, QProcess, QTimer
from PyQt5.QtGui import QFont


class LauncherApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Launcher")

        # Make fullscreen for 1920x1080
        self.showFullScreen()

        # Store process references
        self.gui_process = None
        self.diffbot_process = None

        # Apply large font scaling for 1920x1080 screens
        font = self.font()
        font.setPointSize(24)  # Larger font for fullscreen
        self.setFont(font)

        self.setStyleSheet("""
            QMainWindow {
                background-color: #ffffff;
            }
            QWidget {
                font-size: 24px;
                background-color: #ffffff;
            }
            QPushButton {
                font-size: 32px;
                min-height: 150px;
                min-width: 600px;
                padding: 30px;
                font-weight: bold;
                background-color: #4CAF50;
                color: white;
                border-radius: 15px;
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
                font-size: 28px;
                color: #333333;
            }
        """)

        self.setup_ui()

    def setup_ui(self):
        """Setup the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Add close button at the top right
        top_bar = QWidget()
        top_bar.setStyleSheet("background-color: #ffffff;")
        top_bar_layout = QHBoxLayout(top_bar)
        top_bar_layout.setContentsMargins(20, 20, 20, 20)
        top_bar_layout.addStretch()

        self.close_btn = QPushButton("✕")
        self.close_btn.setFixedSize(30, 30)
        self.close_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                border-radius: 15px;
                font-size: 16px;
                font-weight: bold;
                border: 1px solid white;
                max-width: 30px;
                max-height: 30px;
                min-width: 30px;
                min-height: 30px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:pressed {
                background-color: #a31409;
            }
        """)
        self.close_btn.clicked.connect(self.close_application)
        top_bar_layout.addWidget(self.close_btn)

        main_layout.addWidget(top_bar)

        # Content area
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(30)

        # Title label
        title_label = QLabel("Robot Control System")
        title_font = QFont()
        title_font.setPointSize(48)  # Much larger for fullscreen
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # Add some spacing
        layout.addSpacing(100)

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

        main_layout.addWidget(content_widget)

    def launch_robot_system(self):
        """Launch both ROS2 packages in visible terminals"""
        try:
            print("Launching robot system in terminals...")

            # Source ROS2 environment and workspace setup
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            workspace_setup = "~/ros2_ws/install/setup.bash"

            # Launch roboto_diffbot in a new terminal window
            # The bash -c command runs the launch and keeps terminal open even on error
            diffbot_terminal_cmd = f"source {ros2_setup} && source {workspace_setup} && echo 'Starting diffbot...' && ros2 launch roboto_diffbot launch_roboto.launch.py; echo '\\nProcess exited. Press Enter to close.'; read"

            self.diffbot_process = QProcess(self)
            # Use -- to separate gnome-terminal options from the command
            self.diffbot_process.start("gnome-terminal", [
                "--title=RobotoDiffbot",
                "--geometry=100x30+0+0",
                "--wait",
                "--",
                "bash", "-c", diffbot_terminal_cmd
            ])

            if not self.diffbot_process.waitForStarted(5000):
                raise Exception("Failed to start diffbot terminal")

            print("Diffbot terminal opened, waiting 2 seconds before launching GUI...")

            # Update UI to show we're waiting
            self.launch_btn.setEnabled(False)
            self.status_label.setText("Uruchamianie diffbot...")
            self.status_label.setStyleSheet("color: #ffff00; font-weight: bold;")

            # Store command info for delayed launch
            self.ros2_setup = ros2_setup
            self.workspace_setup = workspace_setup

            # Use QTimer to delay GUI launch without blocking the event loop
            QTimer.singleShot(2000, self.launch_gui_process)

        except Exception as e:
            error_msg = f"Failed to launch: {str(e)}"
            print(f"ERROR: {error_msg}")
            QMessageBox.critical(self, "Error", error_msg)
            self.status_label.setText("Błąd uruchamiania")
            self.status_label.setStyleSheet("color: #ff0000; font-weight: bold;")

            # Clean up processes on error
            if self.diffbot_process:
                self.diffbot_process.kill()
            if self.gui_process:
                self.gui_process.kill()

    def launch_gui_process(self):
        """Launch the GUI process in a terminal after diffbot has started"""
        try:
            print("Launching GUI process in terminal...")

            # Launch GUI in a new terminal window positioned to the right
            # The terminal will stay open after the GUI closes to show any errors
            gui_terminal_cmd = f"source {self.ros2_setup} && source {self.workspace_setup} && echo 'Starting GUI...' && ros2 launch roboto_viz gui_launch.py; echo '\\nProcess exited. Press Enter to close.'; read"

            self.gui_process = QProcess(self)

            # Connect signals BEFORE starting
            self.gui_process.finished.connect(self.on_gui_closed)
            self.gui_process.errorOccurred.connect(self.on_process_error)

            # Start GUI terminal with proper argument separation
            self.gui_process.start("gnome-terminal", [
                "--title=RobotoViz GUI",
                "--geometry=100x30+800+0",
                "--wait",
                "--",
                "bash", "-c", gui_terminal_cmd
            ])

            if not self.gui_process.waitForStarted(5000):
                raise Exception("Failed to start GUI terminal")

            print("GUI terminal opened successfully")

            # Update UI
            self.status_label.setText("System uruchomiony")
            self.status_label.setStyleSheet("color: #00ff00; font-weight: bold;")

        except Exception as e:
            error_msg = f"Failed to launch: {str(e)}"
            print(f"ERROR: {error_msg}")
            QMessageBox.critical(self, "Error", error_msg)
            self.status_label.setText("Błąd uruchamiania")
            self.status_label.setStyleSheet("color: #ff0000; font-weight: bold;")

            # Clean up processes on error
            if self.diffbot_process:
                self.diffbot_process.kill()
            if self.gui_process:
                self.gui_process.kill()

    def on_process_error(self, error):
        """Handle process errors"""
        error_strings = {
            QProcess.FailedToStart: "Failed to start",
            QProcess.Crashed: "Process crashed",
            QProcess.Timedout: "Process timed out",
            QProcess.WriteError: "Write error",
            QProcess.ReadError: "Read error",
            QProcess.UnknownError: "Unknown error"
        }
        error_msg = error_strings.get(error, "Unknown error")
        print(f"Process error: {error_msg}")
        self.status_label.setText(f"Błąd: {error_msg}")
        self.status_label.setStyleSheet("color: #ff0000; font-weight: bold;")

    def on_gui_closed(self, exit_code, exit_status):
        """Handle GUI terminal closure - terminate diffbot terminal too"""
        print(f"GUI terminal closed with exit code {exit_code}, status {exit_status}")
        print("Terminating all ROS2 processes and terminals...")

        # Kill all ros2 launch processes (this will terminate both diffbot and any remaining GUI processes)
        import subprocess
        try:
            # Find and kill all ros2 launch processes
            subprocess.run(["pkill", "-f", "ros2 launch roboto_diffbot"], check=False)
            subprocess.run(["pkill", "-f", "ros2 launch roboto_viz"], check=False)
            print("ROS2 processes terminated")
        except Exception as e:
            print(f"Error terminating ROS2 processes: {e}")

        # Terminate the terminal processes
        if self.diffbot_process and self.diffbot_process.state() == QProcess.Running:
            self.diffbot_process.terminate()
            self.diffbot_process.waitForFinished(2000)

        if self.gui_process and self.gui_process.state() == QProcess.Running:
            self.gui_process.terminate()
            self.gui_process.waitForFinished(2000)

        print("All processes terminated")

        # Re-enable launch button
        self.launch_btn.setEnabled(True)
        self.status_label.setText("Gotowy do uruchomienia")
        self.status_label.setStyleSheet("color: #333333; font-weight: normal;")

    def close_application(self):
        """Close the launcher application (from close button)"""
        print("Close button clicked, shutting down launcher...")
        self.close()

    def closeEvent(self, event):
        """Handle application close - make sure all processes and terminals are terminated"""
        print("Launcher closing, terminating all processes and terminals...")

        # Kill all ros2 launch processes
        import subprocess
        try:
            subprocess.run(["pkill", "-f", "ros2 launch roboto_diffbot"], check=False)
            subprocess.run(["pkill", "-f", "ros2 launch roboto_viz"], check=False)
            print("ROS2 processes terminated")
        except Exception as e:
            print(f"Error terminating ROS2 processes: {e}")

        # Terminate terminal processes
        if self.gui_process and self.gui_process.state() == QProcess.Running:
            print("Terminating GUI terminal...")
            self.gui_process.terminate()
            self.gui_process.waitForFinished(2000)

        if self.diffbot_process and self.diffbot_process.state() == QProcess.Running:
            print("Terminating diffbot terminal...")
            self.diffbot_process.terminate()
            self.diffbot_process.waitForFinished(2000)

        print("Launcher shutdown complete")
        event.accept()


def main():
    app = QApplication(sys.argv)
    launcher = LauncherApp()
    launcher.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

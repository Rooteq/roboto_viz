#!/usr/bin/env python3
"""
Test script for CAN status messaging functionality.
This script demonstrates how the CAN status system works and can be used for testing.
"""

import sys
import time
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtCore import Qt, pyqtSlot

from roboto_viz.can_status_manager import CANStatusManager, CANLEDType, StatusLevel


class CANStatusTester(QWidget):
    """
    Simple GUI for testing CAN status message transmission
    """
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN Status Tester")
        self.setGeometry(300, 300, 400, 300)
        
        # Initialize CAN manager
        self.can_manager = CANStatusManager("can0")
        
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Connection status
        self.status_label = QLabel("CAN Status: Disconnected")
        layout.addWidget(self.status_label)
        
        # Connect/Disconnect buttons
        self.connect_button = QPushButton("Connect to CAN")
        self.connect_button.clicked.connect(self.connect_can)
        layout.addWidget(self.connect_button)
        
        self.disconnect_button = QPushButton("Disconnect from CAN")
        self.disconnect_button.clicked.connect(self.disconnect_can)
        self.disconnect_button.setEnabled(False)
        layout.addWidget(self.disconnect_button)
        
        # LED control selector
        layout.addWidget(QLabel("LED Control:"))
        self.led_combo = QComboBox()
        self.led_combo.addItems(["Green LED (OK)", "Orange LED (WARNING)", "Red LED (ERROR)"])
        layout.addWidget(self.led_combo)
        
        # Test buttons for different LED states
        test_buttons = [
            ("Send: Green LED (Robot OK)", self.send_green_led),
            ("Send: Orange LED (Warning)", self.send_orange_led), 
            ("Send: Red LED (Error)", self.send_red_led),
            ("Test: Robot Status 'Idle'", self.test_robot_idle),
            ("Test: Robot Status 'Failed'", self.test_robot_failed),
            ("Test: Robot Status 'Low battery'", self.test_battery_warning),
        ]
        
        for button_text, callback in test_buttons:
            button = QPushButton(button_text)
            button.clicked.connect(callback)
            layout.addWidget(button)
        
        # Direct LED control test
        self.direct_led_button = QPushButton("Send Direct LED Control")
        self.direct_led_button.clicked.connect(self.send_direct_led)
        layout.addWidget(self.direct_led_button)
        
        self.setLayout(layout)
        
    def connect_can(self):
        if self.can_manager.connect_can():
            self.status_label.setText("CAN Status: Connected")
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
        else:
            self.status_label.setText("CAN Status: Connection Failed")
            
    def disconnect_can(self):
        self.can_manager.disconnect_can()
        self.status_label.setText("CAN Status: Disconnected")
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        
    def send_green_led(self):
        """Send green LED (OK status) directly"""
        self.can_manager._send_led_can_message(CANLEDType.GREEN_LED)
        
    def send_orange_led(self):
        """Send orange LED (WARNING status) directly"""
        self.can_manager._send_led_can_message(CANLEDType.ORANGE_LED)
        
    def send_red_led(self):
        """Send red LED (ERROR status) directly"""
        self.can_manager._send_led_can_message(CANLEDType.RED_LED)
        
    def test_robot_idle(self):
        """Test robot status 'Idle' -> should send GREEN LED"""
        self.can_manager.handle_robot_status("Idle")
        
    def test_robot_failed(self):
        """Test robot status 'Failed' -> should send RED LED"""
        self.can_manager.handle_robot_status("Failed")
        
    def test_battery_warning(self):
        """Test battery status 'Low battery' -> should send ORANGE LED"""
        self.can_manager.handle_battery_status("Low battery")
        
    def send_direct_led(self):
        """Send LED based on combo box selection"""
        led_mapping = {
            "Green LED (OK)": CANLEDType.GREEN_LED,
            "Orange LED (WARNING)": CANLEDType.ORANGE_LED,
            "Red LED (ERROR)": CANLEDType.RED_LED
        }
        
        selected = self.led_combo.currentText()
        led_type = led_mapping[selected]
        
        self.can_manager._send_led_can_message(led_type)


def main():
    """
    Main function to run the CAN status tester
    """
    app = QApplication(sys.argv)
    
    # Create and show the tester window
    tester = CANStatusTester()
    tester.show()
    
    print("CAN Status Tester started")
    print("Make sure you have a CAN interface (e.g., can0) available")
    print("You can create a virtual CAN interface with:")
    print("  sudo modprobe vcan")
    print("  sudo ip link add dev vcan0 type vcan")
    print("  sudo ip link set up vcan0")
    print("\nMonitor CAN messages with:")
    print("  candump vcan0")
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel

class ROS2Thread(QThread):
    twist_received = pyqtSignal(Twist)
    service_response = pyqtSignal(bool)
    service_availability = pyqtSignal(bool)

    def __init__(self, executor):
        super().__init__()
        self.executor = executor
        self.node = None

    def run(self):
        self.node = ROS2Node()
        self.executor.add_node(self.node)
        self.node.twist_callback = self.emit_twist
        self.node.service_response_callback = self.emit_service_response
        self.node.service_availability_callback = self.emit_service_availability
        self.executor.spin()

    def emit_twist(self, twist):
        self.twist_received.emit(twist)

    def emit_service_response(self, success):
        self.service_response.emit(success)

    def emit_service_availability(self, available):
        self.service_availability.emit(available)

    def stop(self):
        if self.node:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        self.executor.shutdown()

class ROS2Node(Node):
    def __init__(self):
        super().__init__('ros2_pyqt_node')
        self.cli = self.create_client(Trigger, 'trigger_service')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.twist_callback = None
        self.service_response_callback = None
        self.service_availability_callback = None
        self.listener_active = False
        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)

    def check_service_availability(self):
        available = self.cli.service_is_ready()
        if self.service_availability_callback:
            self.service_availability_callback(available)

    def send_request(self):
        if not self.cli.service_is_ready():
            self.get_logger().warn('Service is not available')
            if self.service_response_callback:
                self.service_response_callback(False)
            return

        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Service call successful. Activating Twist listener.')
                self.listener_active = True
            else:
                self.get_logger().info('Service call failed. Twist listener remains inactive.')
            if self.service_response_callback:
                self.service_response_callback(response.success)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            if self.service_response_callback:
                self.service_response_callback(False)

    def listener_callback(self, msg):
        if self.listener_active and self.twist_callback:
            self.twist_callback(msg)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 PyQt Integration")
        self.setGeometry(100, 100, 300, 200)

        layout = QVBoxLayout()
        
        self.service_button = QPushButton("Call Service")
        self.service_button.clicked.connect(self.call_service)
        layout.addWidget(self.service_button)

        self.status_label = QLabel("Status: Idle")
        layout.addWidget(self.status_label)

        self.service_status_label = QLabel("Service: Checking...")
        layout.addWidget(self.service_status_label)

        self.twist_label = QLabel("No Twist message received")
        layout.addWidget(self.twist_label)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.ros2_thread = ROS2Thread(self.executor)
        self.ros2_thread.twist_received.connect(self.update_twist)
        self.ros2_thread.service_response.connect(self.update_service_status)
        self.ros2_thread.service_availability.connect(self.update_service_availability)
        self.ros2_thread.start()

    def call_service(self):
        if self.ros2_thread.node:
            self.ros2_thread.node.send_request()
        else:
            print('ROS2 node not initialized')

    def update_twist(self, twist):
        self.twist_label.setText(f"Twist: Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}")

    def update_service_status(self, success):
        status = "Active" if success else "Inactive"
        self.status_label.setText(f"Status: {status}")

    def update_service_availability(self, available):
        status = "Available" if available else "Not Available"
        self.service_status_label.setText(f"Service: {status}")
        self.service_button.setEnabled(available)

    def closeEvent(self, event):
        self.ros2_thread.stop()
        self.ros2_thread.wait()
        rclpy.shutdown()
        super().closeEvent(event)

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
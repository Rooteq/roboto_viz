#Next make the node lifecycle so that pressing the connect button starts period service calls.

import sys
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel

class ManagerNode(LifecycleNode):
    def __init__(self):
        super().__init__('gui_manager_node')
        self.cli = self.create_client(Trigger, 'trigger_service')
        self.twist_callback = None
        self.service_response_callback = None
        self.service_availability_callback = None
        self.listener_pose_callback = None
        self.listener_active = False
        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)

    # On configure start sending service calls, activates service when receives callback (activates either setting initPos or just activates)
    # Call on_configure when the service is avaiable and the button for connection is clicked
    def on_configure(self, previous_state: LifecycleState):
        self.service_call_timer = self.create_timer(2.0, self.periodic_service_call)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state):
        self.destroy_timer(self.service_call_timer)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state):
        self.subscription = self.create_subscription(TwistStamped, 'diffbot_pose', self.listener_callback, 10)
        return super().on_activate(previous_state)

    def on_deactivate(self, state):
        self.destroy_subscription(self.subscription)
        return super().on_deactivate(state)

    def periodic_service_call(self):
        if self.cli.service_is_ready():
            self.send_request()
        else:
            pass
            # self.get_logger().info('Service is not available for periodic call')

    def check_service_availability(self):
        available = self.cli.service_is_ready()
        if not available:
            # self.trigger_deactivate()
            # self.trigger_shutdown()
            self.listener_active = False
        if self.service_availability_callback:
            self.service_availability_callback(available)

    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                if not self.listener_active:
                    self.trigger_activate()
                # self.get_logger().info('Service call successful. Activating Twist listener.')
                self.listener_active = True
            else:
                self.get_logger().info('Service call failed. Twist listener remains inactive.')
            if self.service_response_callback:
                self.service_response_callback(response.success)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            if self.service_response_callback:
                self.service_response_callback(False)

    def pose_callback(self, msg: TwistStamped):
        self.get_logger().indo("Publishing pose as callback")
        # if self.listener_active and self.twist_callback:
        self.listener_pose_callback(msg)



class GuiManager(QThread):
    service_response = pyqtSignal(bool)
    service_availability = pyqtSignal(bool)
    update_pose = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()

        self.node = None
        self.executor = MultiThreadedExecutor()

    def trigger_configure(self):
        self.node.trigger_configure()

    def run(self):
        self.node = ManagerNode()
        self.executor.add_node(self.node)
        self.node.service_response_callback = self.emit_service_response
        self.node.service_availability_callback = self.emit_service_availability
        self.node.listener_pose_callback = self.emit_pose 
        self.executor.spin()

    def emit_service_response(self, success):
        self.service_response.emit(success)

    def emit_service_availability(self, available):
        self.service_availability.emit(available)
    
    def emit_pose(self, msg: TwistStamped):
        # print("emit pose from gui manager!")
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        theta = -msg.twist.angular.z

        self.update_pose.emit(x, y, theta)

    def stop(self):
        if self.node:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        self.executor.shutdown()

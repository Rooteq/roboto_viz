#Next make the node lifecycle so that pressing the connect button starts period service calls.

import sys
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel

class ManagerNode(LifecycleNode):
    def __init__(self):
        super().__init__('gui_manager_node')
        self.cli = self.create_client(Trigger, 'trigger_service')
        self.twist_callback = None

        self.srv_available: bool = False
        
        self.service_response_callback = None
        self.service_availability_callback = None
        self.listener_pose_callback = None
        self.disconnect_callback = None

        self.listener_active = False
        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)

        self.pose_subscriber = None

        self.get_logger().info("Initialized Lifecycle node!")

    # On configure start sending service calls, activates service when receives callback (activates either setting initPos or just activates)
    # Call on_configure when the service is avaiable and the button for connection is clicked
    def on_configure(self, previous_state: LifecycleState):
        self.service_call_timer = self.create_timer(2.0, self.periodic_service_call)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state):
        self.destroy_timer(self.service_call_timer)
        self.get_logger().info("cleanup")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state):
        self.get_logger().info("creating subscription")
        if self.pose_subscriber is None:
            self.pose_subscriber = self.create_subscription(
                TwistStamped,
                '/diffbot_pose',
                self.pose_callback,
                10
            )
        return super().on_activate(previous_state)

    def on_deactivate(self, state):
        if self.pose_subscriber is not None:
            self.destroy_subscription(self.pose_subscriber)
            self.pose_subscriber = None
            self.get_logger().info('Destroyed subscriber')

            self.listener_active = False

        return super().on_deactivate(state)

    def periodic_service_call(self):
        if self.cli.service_is_ready():
            self.send_request()
        else:
            # pass
            # self.get_logger().info('Service is not available for periodic call')
            self.trigger_deactivate()
            self.trigger_cleanup()

    def check_service_availability(self):
        available = self.cli.service_is_ready()
        if available:
            self.srv_available = True
            self.service_availability_callback(available)
        
        if not available and self.srv_available:
            self.srv_available = False
            self.listener_active = False
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
                
                self.listener_active = True
            else:
                self.get_logger().info('Service call failed. Twist listener remains inactive.')
                self.listener_active = False
            if self.service_response_callback:
                self.service_response_callback(response.success)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            if self.service_response_callback:
                self.service_response_callback(False)

    def pose_callback(self, msg: TwistStamped):
        # if self.listener_active and self.twist_callback:
        self.listener_pose_callback(msg)



class GuiManager(QThread):
    service_response = pyqtSignal(bool)
    service_availability = pyqtSignal(bool)
    update_pose = pyqtSignal(float, float, float)

    trigger_disconnect = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.node: ManagerNode = None
        self.executor = MultiThreadedExecutor()

    def trigger_configure(self):
        self.node.trigger_configure()

    @pyqtSlot()
    def trigger_deactivate(self):
        self.node.trigger_deactivate()
        self.node.trigger_shutdown()

    def run(self):
        self.node = ManagerNode()
        self.executor.add_node(self.node)
        self.node.service_response_callback = self.emit_service_response
        self.node.service_availability_callback = self.emit_service_availability
        self.node.listener_pose_callback = self.emit_pose 
        self.node.disconnect_callback = self.emit_disconnect_call
        self.executor.spin()

    def emit_service_response(self, success):
        self.service_response.emit(success)

    def emit_service_availability(self, available):
        self.service_availability.emit(available)
    
    def emit_pose(self, msg: TwistStamped):
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        theta = -msg.twist.angular.z
        # print("emitting pose!")
        self.update_pose.emit(x, y, theta)

    def emit_disconnect_call(self):
        self.trigger_disconnect.emit(True)

    def stop(self):
        if self.node:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        self.executor.shutdown()

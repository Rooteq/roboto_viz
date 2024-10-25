#Next make the node lifecycle so that pressing the connect button starts period service calls.

import sys
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped, Quaternion
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, pyqtSlot, QObject
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
import threading
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

import math

from roboto_viz.route_manager import RouteManager
from typing import Dict, List, Tuple

from copy import deepcopy

from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

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

class NavData(QObject):
    # send_routes = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.route_manager = RouteManager()
        self.navigator: BasicNavigator = BasicNavigator('gui_navigator_node')

        self.routes: Dict[str, List[Tuple[float, float]]]

    def save_routes(self, new_routes: Dict[str, List[Tuple[float, float]]]):
        self.route_manager.save_routes(new_routes)
    
    def load_routes(self):
        self.routes = self.route_manager.load_routes()

        # self.send_routes.emit(list(self.routes.keys()))

class Navigator(QThread):
    finished = pyqtSignal()  # Emitted when a goal is reached
    navigation_status = pyqtSignal(str)  # Optional: to inform GUI about status

    def __init__(self, nav_data: NavData):
        super().__init__()
        self.nav_data = nav_data
        self._running = True
        self._new_goal: list = None
        self._goal_lock = threading.Lock()  # For thread-safe goal updates
        
    def stop(self):
        self._running = False
        if self.nav_data.navigator.isTaskComplete() is False:
            self.nav_data.navigator.cancelTask()
        self.wait()

    def set_goal(self, route: str):
        """Set a new goal, replacing any existing one"""
        with self._goal_lock:
            self._new_goal = self.nav_data.routes[route]
            # Cancel current navigation if there is one
            if not self.nav_data.navigator.isTaskComplete():
                self.nav_data.navigator.cancelTask()
        
    def run(self):
        while self._running:
            # Check if we have a new goal
            with self._goal_lock:
                current_goal = self._new_goal
                self._new_goal = None
            
            if current_goal is None:
                # No goal to process, wait a bit
                self.msleep(100)  # Sleep for 100ms
                continue
                
            try:
                points_on_route = []

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.nav_data.navigator.get_clock().now().to_msg()

                for point in current_goal:
                    goal_pose.pose.position.x = point[0]
                    goal_pose.pose.position.y = point[1]
                    goal_pose.pose.position.z = 0.0

                    q = quaternion_from_euler(0,0,point[3])
                    goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    points_on_route.append(deepcopy(goal_pose))

                # Update goal pose
                
                # Start navigation

                # self.nav_data.navigator.goToPose(self.nav_data.goal_pose)
                self.nav_data.navigator.goThroughPoses(points_on_route)
                self.nav_data.navigator.waitUntilNav2Active()
                
                while not self.nav_data.navigator.isTaskComplete() and self._running:
                    # Check if there's a new goal
                    if self._new_goal is not None:
                        break  # Exit this loop to process the new goal
                        
                    feedback = self.nav_data.navigator.getFeedback()
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        self.nav_data.navigator.cancelTask()
                        break
                
                if self._running and self.nav_data.navigator.isTaskComplete():
                    self.finished.emit()
                    
            except Exception as e:
                print(f"Navigation error: {e}")
                self.navigation_status.emit(f"Error: {str(e)}")


class GuiManager(QThread):
    service_response = pyqtSignal(bool)
    service_availability = pyqtSignal(bool)
    update_pose = pyqtSignal(float, float, float)

    trigger_disconnect = pyqtSignal(bool)

    send_route_names = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.node: ManagerNode = None
        self.executor = MultiThreadedExecutor()
        self.nav_data: NavData = NavData()
        self.navigator: Navigator = Navigator(self.nav_data)

        # self.nav_data.send_routes.connect(lambda: self.send_route_names)

        self.navigator.start()

    def send_routes(self):
        self.nav_data.load_routes()
        self.send_route_names.emit(self.nav_data.routes)

    pyqtSignal(dict)
    def save_routes(self, new_routes: dict):
        self.nav_data.save_routes(new_routes)

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

    def stop(self):
        if self.navigator:
            self.navigator.stop()
        if self.node:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        self.executor.shutdown()

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

    pyqtSlot(str)
    def handle_set_route(self, route: str):
        print("New goal set!")
        self.navigator.set_goal(route)
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

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration

import math
from enum import Enum
from roboto_viz.route_manager import RouteManager
from typing import Dict, List, Tuple

from copy import deepcopy

from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

class LState(Enum):
    UNCONFIGURED = 0
    INACTIVE = 1
    ACTIVE = 2

class ManagerNode(LifecycleNode):
    def __init__(self):
        super().__init__('gui_manager_node')
        self.cli = self.create_client(Trigger, 'trigger_service')
        self.twist_callback = None

        self.pose_subscriber = None
        self.init_pose_pub = None

        #CALLBACKS: 
        self.service_availability_callback = None
        self.listener_pose_callback = None
        self.disconnect_callback = None
        self.connect_callback = None

        #INTERNAL STATES:
        self.srv_available: bool = False
        

        self._current_state: LState = LState.UNCONFIGURED
        self._current_state = LState.UNCONFIGURED

        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)
        self.get_logger().info("Initialized Lifecycle node!")

    def set_initial_pose(self, x: float, y: float, orientation_w: float):
        """
        Publish the initial pose for AMCL localization
        Args:
            x: x position in map frame
            y: y position in map frame
            orientation_w: w quaternion component (default 1.0 for no rotation)
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set the pose
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0  # Assuming 2D navigation
        
        # Set orientation (default is no rotation)
        q = quaternion_from_euler(0,0,orientation_w)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Set covariance matrix (you might want to adjust these values)
        msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.25, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.06853891945200942
        ]
        
        # Publish the message
        self.init_pose_pub.publish(msg)
        self.get_logger().info(f'Published initial pose: x={x}, y={y}, w={orientation_w}')

    # On configure start sending service calls, activates service when receives callback (activates either setting initPos or just activates)
    # Call on_configure when the service is avaiable and the button for connection is clicked
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("configure")
        self.service_call_timer = self.create_timer(2.0, self.periodic_service_call)

        self._current_state = LState.INACTIVE

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state):
        self.get_logger().info("cleanup")
        self.destroy_timer(self.service_call_timer)

        self._current_state = LState.UNCONFIGURED

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state):
        self.get_logger().info("activate")
        if self.pose_subscriber is None:
            self.pose_subscriber = self.create_subscription(
                TwistStamped,
                '/diffbot_pose',
                self.pose_callback,
                10
            )

        if self.init_pose_pub is None:
            self.init_pose_pub = self.create_lifecycle_publisher(
                PoseWithCovarianceStamped,
                'initialpose',
                10
            )
            self.get_logger().info("Created initial pose publisher")
            

        self._current_state = LState.ACTIVE

        return super().on_activate(previous_state)

    def on_deactivate(self, state):
        self.get_logger().info('deactivate')
        if self.pose_subscriber is not None:
            self.destroy_subscription(self.pose_subscriber)
            self.pose_subscriber = None
            self.get_logger().info("Destroyed pose subscriber")

        if self.init_pose_pub is not None:
            self.destroy_lifecycle_publisher(self.init_pose_pub)
            self.init_pose_pub = None
            self.get_logger().info("Destroyed initial pose publisher")

        self._current_state = LState.INACTIVE

        return super().on_deactivate(state)

    def periodic_service_call(self):
        if self.cli.service_is_ready():
            self.send_request()
        else:
            self.trigger_deactivate()
            self.trigger_cleanup()
            self.disconnect_callback()

    def check_service_availability(self):
        available = self.cli.service_is_ready()
        self.service_availability_callback(available)
        
    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                if self._current_state == LState.INACTIVE:
                    self.trigger_activate()
                    self.connect_callback()
            else:
                self.get_logger().info('Service call failed. Twist listener remains inactive.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def pose_callback(self, msg: TwistStamped):
        self.listener_pose_callback(msg)

class NavData(QObject):
    def __init__(self):
        super().__init__()
        self.route_manager = RouteManager()
        self.navigator: BasicNavigator = BasicNavigator('gui_navigator_node')

        self.routes: Dict[str, List[Tuple[float, float]]]
        self.maps: List[str]

    def save_routes(self, new_routes: Dict[str, List[Tuple[float, float]]]):
        self.route_manager.save_routes(new_routes)
    
    def load_routes(self):
        self.routes = self.route_manager.load_routes()

    def load_maps(self):
        self.maps = self.route_manager.get_map_names()

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

        self._to_dest: bool = True

        self.curr_x: float = 0
        self.curr_y: float = 0
        
    def stop(self):
        # self._running = False
        if self.nav_data.navigator.isTaskComplete() is False:
            self.nav_data.navigator.cancelTask()
        # self.wait() # wait needed?

    def set_goal(self, route: str, to_dest: bool, x:float, y:float):
        """Set a new goal, replacing any existing one"""
        with self._goal_lock:
            self.curr_x = x
            self.curr_y = y
            self._to_dest = to_dest
            # self._running = True
            self._new_goal = deepcopy(self.nav_data.routes[route])
            if not to_dest:
                self._new_goal.reverse()
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
                # Choosing the shortest path
                # dist: float = sys.float_info.max
                distances = [math.dist((self.curr_x,self.curr_y), (waypoint[0], waypoint[1])) for waypoint in current_goal]

                # Find the index of the closest waypoint
                closest_index = distances.index(min(distances))

                # Reorder waypoints starting from the closest one
                waypoints = current_goal[closest_index:]

                points_on_route = []

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.nav_data.navigator.get_clock().now().to_msg()

                for point in waypoints:
                    goal_pose.pose.position.x = point[0]
                    goal_pose.pose.position.y = point[1]
                    goal_pose.pose.position.z = 0.0

                    if self._to_dest:
                        q = quaternion_from_euler(0,0,point[3])
                        goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    else:
                        q = quaternion_from_euler(0,0,(point[3] + math.pi))
                        goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                    points_on_route.append(deepcopy(goal_pose))

                # Update goal pose
                
                # Start navigation

                # self.nav_data.navigator.goToPose(self.nav_data.goal_pose)
                self.nav_data.navigator.followWaypoints(points_on_route)
                # self.nav_data.navigator.goThroughPoses(points_on_route)
                self.nav_data.navigator.waitUntilNav2Active()
                
                while not self.nav_data.navigator.isTaskComplete() and self._running:
                    # Check if there's a new goal
                    if self._new_goal is not None:
                        break  # Exit this loop to process the new goal
                        
                    feedback = self.nav_data.navigator.getFeedback()
                    print(f"feedback: {feedback}")
                    # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    #     self.nav_data.navigator.cancelTask()
                    #     break
                
                if self._running and self.nav_data.navigator.isTaskComplete():
                    print(f"{self.nav_data.navigator.getResult()}")
                    self.finished.emit()
                    
            except Exception as e:
                print(f"Navigation error: {e}")
                self.navigation_status.emit(f"Error: {str(e)}")


class GuiManager(QThread):
    service_response = pyqtSignal(bool)
    update_pose = pyqtSignal(float, float, float)

    """ 
    DISCONNECTED STATE:
        service availability dictates if user should be able to connect 
        after calling the self.trigger_configure() the gui should send trigger_connection() which would change the app state to active

        trigger_disconnection disconnected
    ACTIVE STATE:
        service availability disconnected
        trigger_connection disconnected
        
        trigger_disconnection should change the app state to disconnected (service availabilty connected in DISCONNECTED STATE)

    PLANNING STATE:
        as ACTIVE_STATE
    """
    service_availability = pyqtSignal(bool) #Periodic call
    trigger_disconnect = pyqtSignal()
    trigger_connection = pyqtSignal()

    send_route_names = pyqtSignal(dict)
    send_map_names = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.node: ManagerNode = None
        self.executor = MultiThreadedExecutor()
        self.nav_data: NavData = NavData()
        self.navigator: Navigator = Navigator(self.nav_data)

        # self.nav_data.send_routes.connect(lambda: self.send_route_names)

        self.navigator.start()

    def send_maps(self):
        self.nav_data.load_maps()
        self.nav_data.route_manager.load_map("robots_map")
        # self.nav_data.route_manager.load_map_onto_robot("test")
        
        self.send_map_names.emit(self.nav_data.maps)

    def send_routes(self):
        self.nav_data.load_routes()
        self.send_route_names.emit(self.nav_data.routes)

    pyqtSignal(dict)
    def save_routes(self, new_routes: dict):
        self.nav_data.save_routes(new_routes)

    @pyqtSlot()
    def trigger_configure(self):
        self.node.trigger_configure()

    @pyqtSlot()
    def trigger_deactivate(self):
        self.node.trigger_deactivate()
        self.node.trigger_shutdown()
    
    @pyqtSlot(float,float,float)
    def set_init_pose(self, x, y, w):
        self.node.set_initial_pose(x,y,w)

    def run(self):
        self.node = ManagerNode()
        self.executor.add_node(self.node)

        self.node.service_availability_callback = self.emit_service_availability
        self.node.listener_pose_callback = self.emit_pose 
        self.node.disconnect_callback = self.emit_disconnect_call
        self.node.connect_callback = self.emit_connect_call

        self.executor.spin()

    def stop(self):
        if self.navigator:
            self.navigator.stop()
        if self.node:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        self.executor.shutdown()

    def emit_service_availability(self, available):
        self.service_availability.emit(available)

    def emit_disconnect_call(self):
        self.trigger_disconnect.emit()
    
    def emit_connect_call(self):
        self.trigger_connection.emit()

    def emit_pose(self, msg: TwistStamped):
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        theta = -msg.twist.angular.z
        self.update_pose.emit(x, y, theta)

    pyqtSlot(str)
    def handle_set_route(self, route: str, to_dest: bool, x:float, y:float):
        print(f"New goal set!, To_dest: {to_dest}")
        self.navigator.set_goal(route, to_dest, x, y)

    pyqtSlot()
    def stop_nav(self):
        self.navigator.stop()
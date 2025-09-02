#Next make the node lifecycle so that pressing the connect button starts period service calls.

import sys
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionClient

from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped, Quaternion
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, pyqtSlot, QObject
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
import threading
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus

import math
from enum import Enum
from roboto_viz.route_manager import RouteManager, BezierRoute
from typing import Dict, List, Tuple

try:
    from lidar_auto_docking_messages.action import Dock, Undock
except ImportError:
    print("Warning: lidar_auto_docking_messages not found. Docking functionality will be disabled.")
    Dock = None
    Undock = None

from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from roboto_viz.can_status_manager import CANStatusManager
from roboto_viz.can_battery_receiver import CANBatteryReceiver
from roboto_viz.can_signal_receiver import CANSignalReceiver

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
        self.cmd_vel_pub = None

        # Docking action clients
        self.dock_action_client = None
        self.undock_action_client = None
        self.current_dock_goal_handle = None
        self.current_undock_goal_handle = None
        
        #CALLBACKS: 
        self.service_availability_callback = None
        self.listener_pose_callback = None
        self.disconnect_callback = None
        self.connect_callback = None
        self.docking_status_callback = None

        #INTERNAL STATES:
        self.srv_available: bool = False
        self.suppress_docking_status = False  # Flag to suppress docking status during general stop
        
        self.cmd_vel_msg: Twist = Twist()

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
        
        self.vel_pub_timer = self.create_timer(0.3, self.publish_cmd_vel)
        self.vel_pub_timer.cancel()

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
            

        if self.cmd_vel_pub is None:
            self.cmd_vel_pub = self.create_lifecycle_publisher(
                Twist,
                'cmd_vel_key',
                10
            )

        # Create docking action clients if messages are available
        if Dock is not None and self.dock_action_client is None:
            self.dock_action_client = ActionClient(self, Dock, 'Dock')
            self.get_logger().info("Created dock action client")
            
        if Undock is not None and self.undock_action_client is None:
            self.undock_action_client = ActionClient(self, Undock, 'Undock')
            self.get_logger().info("Created undock action client")

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

        if self.cmd_vel_pub is not None:
            self.destroy_lifecycle_publisher(self.cmd_vel_pub)
            self.cmd_vel_pub = None
            self.get_logger().info("Destroyed cmd_vel publisher")
        self._current_state = LState.INACTIVE

        return super().on_deactivate(state)

    def periodic_service_call(self):
        if self.cli.service_is_ready():
            self.send_request()
        else:
            if(self._current_state != LState.INACTIVE):
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
            response: Trigger.Response = future.result()
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


    def publish_cmd_vel(self):
        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def start_publishing(self, dir: str, vel: float = 0.5):
        if self.vel_pub_timer.is_canceled():
            match dir:
                case 'f':
                    self.cmd_vel_msg.linear.x = vel
                case 'b':
                    self.cmd_vel_msg.linear.x = -vel
                case 'l':
                    self.cmd_vel_msg.angular.z = vel
                case 'r':
                    self.cmd_vel_msg.angular.z = -vel
                case _:
                    self.get_logger().info("Wrong cmd command")
            self.vel_pub_timer.reset()  # Resets the timer if it's canceled
        # self.get_logger().info("Started publishing to cmd_vel")

    def stop_publishing(self):

        self.vel_pub_timer.cancel()

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.linear.y = 0.0
        self.cmd_vel_msg.linear.z = 0.0

        self.cmd_vel_msg.angular.x = 0.0
        self.cmd_vel_msg.angular.y = 0.0
        self.cmd_vel_msg.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel_msg)

        self.get_logger().info("Stopped publishing to cmd_vel")
        
    def send_dock_goal(self, dock_x=None, dock_y=None):
        """Send a docking goal to the robot with optional coordinates"""
        if Dock is None or self.dock_action_client is None:
            self.get_logger().error("Docking action client not available")
            if self.docking_status_callback:
                self.docking_status_callback("Dock Error")
            return
            
        goal_msg = Dock.Goal()
        
        if dock_x is not None and dock_y is not None:
            # Create dock pose with specific coordinates
            goal_msg.dock_pose = PoseStamped()
            goal_msg.dock_pose.header.frame_id = "map"
            goal_msg.dock_pose.pose.position.x = dock_x
            goal_msg.dock_pose.pose.position.y = dock_y
            goal_msg.dock_pose.pose.position.z = 0.0
            goal_msg.dock_pose.pose.orientation.w = 1.0  # No rotation
            goal_msg.dock_id = "specific_location"
            self.get_logger().info('DOCK SPECIFIC LOCATION')
        else:
            # Create empty dock pose - server will detect live
            goal_msg.dock_pose = PoseStamped()
            goal_msg.dock_pose.header.frame_id = "base_link"
            goal_msg.dock_id = "live_detection"
            self.get_logger().info('DOCK LIVE LOCATION')
        
        self.get_logger().info('Waiting for dock action server...')
        if self.docking_status_callback:
            self.docking_status_callback("Docking...")
            
        # Check if server is available, then send goal
        if self.dock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Sending dock goal...')
            send_goal_future = self.dock_action_client.send_goal_async(
                goal_msg, feedback_callback=self._dock_feedback_callback)
            send_goal_future.add_done_callback(self._dock_goal_response_callback)
        else:
            self.get_logger().error('Dock action server not available')
            if self.docking_status_callback:
                self.docking_status_callback("Dock Server Unavailable")

    def _dock_goal_response_callback(self, future):
        """Handle dock goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Dock goal rejected')
                if self.docking_status_callback:
                    self.docking_status_callback("Dock Rejected")
                return

            self.get_logger().info('Dock goal accepted')
            # Store the goal handle for potential cancellation
            self.current_dock_goal_handle = goal_handle
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self._dock_result_callback)
        except Exception as e:
            self.get_logger().error(f'Error in dock goal response: {e}')
            if self.docking_status_callback:
                self.docking_status_callback("Dock Error")

    def _dock_result_callback(self, future):
        """Handle dock result"""
        try:
            result = future.result().result
            status = future.result().status
            # Clear the goal handle since docking is complete
            self.current_dock_goal_handle = None
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Docking succeeded! Docked: {result.docked}')
                if self.docking_status_callback:
                    self.docking_status_callback("Docked" if result.docked else "Dock Failed")
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Docking was cancelled')
                if self.docking_status_callback and not self.suppress_docking_status:
                    self.docking_status_callback("Dock Cancelled")
            else:
                self.get_logger().info('Docking failed')
                if self.docking_status_callback:
                    self.docking_status_callback("Dock Failed")
        except Exception as e:
            self.get_logger().error(f'Error in dock result: {e}')
            if self.docking_status_callback:
                self.docking_status_callback("Dock Error")

    def _dock_feedback_callback(self, feedback_msg):
        """Handle dock feedback"""
        try:
            feedback = feedback_msg.feedback
            # Log dock pose updates
            # pose = feedback.dock_pose.pose
            # self.get_logger().info(
            #     f'Dock detected at: x={pose.position.x:.2f}, y={pose.position.y:.2f}')
        except Exception as e:
            self.get_logger().error(f'Error in dock feedback: {e}')

    def cancel_dock_goal(self):
        """Cancel any ongoing docking operation"""
        if self.current_dock_goal_handle is not None:
            try:
                # Cancel the specific goal handle
                cancel_future = self.current_dock_goal_handle.cancel_goal_async()
                self.get_logger().info('Sent cancellation request for docking operation')
                # The result callback will handle the "Dock Cancelled" status when cancellation completes
            except Exception as e:
                self.get_logger().error(f"Error cancelling dock goal: {e}")
                # Still emit cancelled status in case of error
                if self.docking_status_callback:
                    self.docking_status_callback("Dock Cancelled")
        else:
            # No active docking goal to cancel - don't emit status message
            self.get_logger().debug("No active docking operation to cancel")

    def send_undock_goal(self):
        """Send an undocking goal to the robot"""
        if Undock is None or self.undock_action_client is None:
            self.get_logger().error("Undocking action client not available")
            if self.docking_status_callback:
                self.docking_status_callback("Undock Error")
            return
            
        goal_msg = Undock.Goal()
        goal_msg.rotate_in_place = True
        
        self.get_logger().info('Waiting for undock action server...')
        if self.docking_status_callback:
            self.docking_status_callback("Undocking...")
            
        # Check if server is available, then send goal
        if self.undock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Sending undock goal...')
            send_goal_future = self.undock_action_client.send_goal_async(
                goal_msg, feedback_callback=self._undock_feedback_callback)
            send_goal_future.add_done_callback(self._undock_goal_response_callback)
        else:
            self.get_logger().error('Undock action server not available')
            if self.docking_status_callback:
                self.docking_status_callback("Undock Server Unavailable")

    def _undock_goal_response_callback(self, future):
        """Handle undock goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Undock goal rejected')
                if self.docking_status_callback:
                    self.docking_status_callback("Undock Rejected")
                return

            self.get_logger().info('Undock goal accepted')
            # Store the goal handle for potential cancellation
            self.current_undock_goal_handle = goal_handle
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self._undock_result_callback)
        except Exception as e:
            self.get_logger().error(f'Error in undock goal response: {e}')
            if self.docking_status_callback:
                self.docking_status_callback("Undock Error")

    def _undock_result_callback(self, future):
        """Handle undock result"""
        try:
            result = future.result().result
            status = future.result().status
            # Clear the goal handle since undocking is complete
            self.current_undock_goal_handle = None
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Undocking succeeded! Undocked: {result.undocked}')
                if self.docking_status_callback:
                    self.docking_status_callback("Undocked" if result.undocked else "Undock Failed")
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Undocking was cancelled')
                if self.docking_status_callback and not self.suppress_docking_status:
                    self.docking_status_callback("Undock Cancelled")
            else:
                self.get_logger().info('Undocking failed')
                if self.docking_status_callback:
                    self.docking_status_callback("Undock Failed")
        except Exception as e:
            self.get_logger().error(f'Error in undock result: {e}')
            if self.docking_status_callback:
                self.docking_status_callback("Undock Error")

    def _undock_feedback_callback(self, feedback_msg):
        """Handle undock feedback"""
        try:
            feedback = feedback_msg.feedback
            # Log undock feedback updates if needed
            # self.get_logger().info(f'Undocking feedback: {feedback}')
        except Exception as e:
            self.get_logger().error(f'Error in undock feedback: {e}')

    def cancel_undock_goal(self):
        """Cancel any ongoing undocking operation"""
        if self.current_undock_goal_handle is not None:
            try:
                # Cancel the specific goal handle
                cancel_future = self.current_undock_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._undock_cancel_callback)
                self.get_logger().info('Sent cancellation request for undocking operation')
                # The result callback will handle the "Undock Cancelled" status when cancellation completes
            except Exception as e:
                self.get_logger().error(f"Error cancelling undock goal: {e}")
                # Still emit cancelled status in case of error
                if self.docking_status_callback:
                    self.docking_status_callback("Undock Cancelled")
        else:
            # No active undocking goal to cancel - don't emit status message
            self.get_logger().debug("No active undocking operation to cancel")

    def _undock_cancel_callback(self, future):
        """Handle undock cancellation response"""
        try:
            cancel_response = future.result()
            if cancel_response.return_code == 0:  # SUCCESS
                self.get_logger().info('Undocking cancellation accepted by server')
            else:
                self.get_logger().warning(f'Undocking cancellation rejected by server: {cancel_response.return_code}')
                # Still emit cancelled status for UI feedback
                if self.docking_status_callback:
                    self.docking_status_callback("Undock Cancelled")
        except Exception as e:
            self.get_logger().error(f'Error in undock cancel callback: {e}')
            if self.docking_status_callback:
                self.docking_status_callback("Undock Cancelled")
        
class NavData(QObject):
    def __init__(self):
        super().__init__()
        self.route_manager = RouteManager()
        self.navigator: BasicNavigator = BasicNavigator('gui_navigator_node')

        self.routes: Dict[str, 'BezierRoute']  # Changed to BezierRoute objects
        self.maps: List[str]
        self.current_map: str = None

    def save_routes(self, new_routes: Dict[str, 'BezierRoute']):
        self.routes = new_routes
        self.route_manager.save_routes(new_routes)
    
    def load_routes(self):
        self.routes = self.route_manager.load_routes()

    def load_maps(self):
        self.maps = self.route_manager.get_map_names()

    def set_current_map(self, map_name: str):
        """Set current map and load its routes"""
        self.current_map = map_name
        self.route_manager.set_current_map(map_name)
        self.load_routes()  # Reload routes for new map

class Navigator(QThread):
    navStatus = pyqtSignal(str)

    finished = pyqtSignal()  # Emitted when a goal is reached
    navigation_status = pyqtSignal(str)  # Optional: to inform GUI about status

    def __init__(self, nav_data: NavData):
        super().__init__()
        self.nav_data = nav_data
        self._running = True
        self._docked = False
        self._new_goal: list = None
        self._goal_lock = threading.Lock()  # For thread-safe goal updates

        self._to_dest: bool = True

        self.curr_x: float = 0
        self.curr_y: float = 0
        self._last_status = None  # Track last emitted status to avoid overriding failures
        
    def stop(self):
        # self._running = False
        # Always try to cancel any ongoing navigation task
        try:
            if self.nav_data.navigator.isTaskComplete() is False:
                self.nav_data.navigator.cancelTask()
        except Exception as e:
            print(f"Warning: Error canceling navigation task: {e}")
        
        # Always emit "Stopped" status when explicitly stopping navigation
        # This ensures the UI always updates when STOP is pressed
        self._last_status = "Stopped"
        self.navStatus.emit("Stopped")


    def set_goal(self, route: str, to_dest: bool, x:float, y:float):
        """Set a new goal, replacing any existing one"""
        # Reset last status when setting a new goal
        self._last_status = None
        with self._goal_lock:
            self.curr_x = x
            self.curr_y = y
            self._to_dest = to_dest
            
            # Get the BezierRoute object and convert to waypoints
            if route in self.nav_data.routes:
                bezier_route = self.nav_data.routes[route]
                waypoints = bezier_route.generate_waypoints(points_per_segment=20)
                self._new_goal = waypoints
                
                # Don't reverse here - let the run() method handle direction
                    
                # Cancel current navigation if there is one
                if not self.nav_data.navigator.isTaskComplete():
                    self.nav_data.navigator.cancelTask()
                
                if to_dest == True:
                    self._last_status = "Nav to dest"
                    self.navStatus.emit("Nav to dest")
                else:
                    self._last_status = "Nav to base"
                    self.navStatus.emit("Nav to base")
            else:
                print(f"Route '{route}' not found in routes")

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
                # Simplify direction logic: just reverse path order for return trip
                if self._to_dest:
                    # Going to destination: use waypoints in original order
                    waypoints = current_goal
                else:
                    # Going to base: use waypoints in reverse order
                    waypoints = list(reversed(current_goal))
                
                # Find closest waypoint to start from
                distances = [math.dist((self.curr_x, self.curr_y), (waypoint[0], waypoint[1])) for waypoint in waypoints]
                closest_index = distances.index(min(distances))
                waypoints = waypoints[closest_index:]
                
                # Create Path message for followPath
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                # Don't set timestamp - let nav2 handle it
                
                print(f"DEBUG: Creating path with {len(waypoints)} waypoints, _to_dest={self._to_dest}")
                
                for i, point in enumerate(waypoints):
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = 'map'
                    # Don't set timestamp - let nav2 handle it
                    
                    pose_stamped.pose.position.x = point[0]
                    pose_stamped.pose.position.y = point[1]
                    pose_stamped.pose.position.z = 0.0
                    
                    # Calculate orientation based on direction of travel
                    if i == len(waypoints) - 1 and len(waypoints) > 1:
                        # For final waypoint, calculate orientation from direction to previous point
                        prev_point = waypoints[i-1]
                        curr_point = waypoints[i]
                        
                        dx = curr_point[0] - prev_point[0]
                        dy = curr_point[1] - prev_point[1]
                        
                        if self._to_dest:
                            # Going to destination: direction from previous to current
                            orientation = math.atan2(dy, dx)
                        # else:
                        #     # Going back to base: direction from current to previous (reverse)
                        #     # orientation = math.atan2(-dy, -dx)
                            
                        print(f"DEBUG: Final waypoint calculated orientation: {orientation} radians ({math.degrees(orientation)} degrees)")
                    else:
                        # For non-final waypoints, use original or reverse orientation
                        if self._to_dest:
                            # Going to destination: use original curve tangent
                            orientation = point[3]
                        else:
                            # Going back to base: reverse the tangent direction
                            orientation = point[3] + math.pi
                    
                    q = quaternion_from_euler(0, 0, orientation)
                    pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    
                    path_msg.poses.append(pose_stamped)
                
                # Wait for nav2 to be active first (with timeout)
                print("DEBUG: Waiting for nav2 to become active...")
                try:
                    # Add timeout to prevent infinite blocking
                    nav2_ready = threading.Event()
                    
                    def wait_for_nav2():
                        try:
                            self.nav_data.navigator.waitUntilNav2Active()
                            nav2_ready.set()
                        except Exception as e:
                            print(f"DEBUG: Error waiting for nav2: {e}")
                    
                    wait_thread = threading.Thread(target=wait_for_nav2)
                    wait_thread.daemon = True
                    wait_thread.start()
                    
                    # Wait up to 10 seconds for nav2 to be ready
                    if not nav2_ready.wait(timeout=10.0):
                        print("DEBUG: Timeout waiting for nav2 to become active, proceeding anyway...")
                    else:
                        print("DEBUG: Nav2 is now active!")
                        
                except Exception as e:
                    print(f"DEBUG: Exception in nav2 wait: {e}")
                    # Continue anyway
                
                # Start path navigation with specific controller and goal checker
                # Try with precise_goal_checker to maintain orientation
                print(f"DEBUG: Starting navigation with followPath, {len(path_msg.poses)} poses")
                self.nav_data.navigator.followPath(
                    path_msg,
                    controller_id='FollowPath',
                    goal_checker_id='goal_checker'
                )
                print("DEBUG: followPath() call completed, navigation should now be active")
                
                while not self.nav_data.navigator.isTaskComplete() and self._running:
                    # Check if there's a new goal
                    if self._new_goal is not None:
                        break  # Exit this loop to process the new goal
                        
                    self.msleep(100)  # Small delay to prevent CPU hogging
                
                if self._running and self.nav_data.navigator.isTaskComplete():
                    if self.nav_data.navigator.getResult() is TaskResult.SUCCEEDED:
                        print(f"DEBUG: Navigation completed, _to_dest = {self._to_dest}")
                        if self._to_dest:
                            self._last_status = "At destination"
                            self.navStatus.emit("At destination")
                        else:
                            self._last_status = "At base"
                            self.navStatus.emit("At base")
                    elif self.nav_data.navigator.getResult() is TaskResult.FAILED:
                        self._last_status = "Failed"
                        self.navStatus.emit("Failed")
                        print(f"Navigation result: {self.nav_data.navigator.getResult()}")
                    
                    self.finished.emit()
                    
            except Exception as e:
                print(f"Navigation error: {e}")
                self._last_status = "Navigation Error"
                self.navStatus.emit("Navigation Error")

class GuiManager(QThread):
    manualStatus = pyqtSignal(str)
    dockingStatus = pyqtSignal(str)
    
    # Additional status signals for CAN forwarding
    robotStatusCAN = pyqtSignal(str)  # For robot status updates 
    batteryStatusCAN = pyqtSignal(str)  # For battery status updates
    planStatusCAN = pyqtSignal(str)  # For plan status updates

    service_response = pyqtSignal(bool)
    update_pose = pyqtSignal(float, float, float)
    
    # Battery ADC value signal
    battery_adc_update = pyqtSignal(int)  # Raw ADC value (0-1023)
    battery_percentage_update = pyqtSignal(int, str)  # Battery percentage and status string
    
    # CAN wait signal
    can_signal_received = pyqtSignal()  # CAN signal for wait actions
    wait_action_status = pyqtSignal(str)  # Wait action status for CAN WARNING messages
    
    # Plan execution signals
    plan_execution_start = pyqtSignal(str)  # plan_name
    plan_execution_stop = pyqtSignal()
    plan_action_execute = pyqtSignal(str, int)  # plan_name, action_index

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

    def __init__(self, dock_manager=None, can_interface="can0", enable_can=True):
        super().__init__()
        self.node: ManagerNode = None
        self.executor = MultiThreadedExecutor()
        self.nav_data: NavData = NavData()
        self.navigator: Navigator = Navigator(self.nav_data)
        self.dock_manager = dock_manager

        # Initialize CAN status manager
        self.can_manager = None
        # Initialize CAN receivers
        self.can_battery_receiver = None
        self.can_signal_receiver = None
        if enable_can:
            self.can_manager = CANStatusManager(can_interface)
            self.can_battery_receiver = CANBatteryReceiver(can_interface)
            self.can_signal_receiver = CANSignalReceiver(can_interface)
            self._connect_can_signals()

        # self.nav_data.send_routes.connect(lambda: self.send_route_names)

        self.navigator.start()

    def _connect_can_signals(self):
        """Connect all status signals to CAN manager and battery receiver"""
        if not self.can_manager:
            return
            
        # Connect the various status signals to CAN handlers
        self.manualStatus.connect(self.can_manager.handle_manual_status)
        self.dockingStatus.connect(self.can_manager.handle_docking_status)
        self.robotStatusCAN.connect(self.can_manager.handle_robot_status)
        self.batteryStatusCAN.connect(self.can_manager.handle_battery_status)
        self.planStatusCAN.connect(self.can_manager.handle_plan_status)
        self.wait_action_status.connect(self.can_manager.handle_wait_action_status)
        
        # Connect navigation status from navigator if available
        if hasattr(self.navigator, 'navStatus'):
            self.navigator.navStatus.connect(self.can_manager.handle_navigation_status)
            
        # Connect battery receiver if available
        if self.can_battery_receiver:
            self.can_battery_receiver.battery_status_update.connect(self.handle_battery_adc_update)
            self.can_battery_receiver.battery_percentage_update.connect(self.handle_battery_percentage_update)
            
        # Connect signal receiver if available
        if self.can_signal_receiver:
            self.can_signal_receiver.signal_received.connect(self.handle_can_signal_received)
    
    def send_robot_status_to_can(self, status: str):
        """Send robot status to CAN bus via signal"""
        self.robotStatusCAN.emit(status)
    
    def send_battery_status_to_can(self, status: str):
        """Send battery status to CAN bus via signal"""
        self.batteryStatusCAN.emit(status)
    
    def send_plan_status_to_can(self, status: str):
        """Send plan status to CAN bus via signal"""
        self.planStatusCAN.emit(status)
    
    def handle_battery_adc_update(self, adc_value: int):
        """Handle battery ADC updates from CAN"""
        # Print debug information
        # print(f"DEBUG: Battery ADC received: {adc_value} (0-1023 range)")
        
        # Emit the raw ADC value for GUI updates
        self.battery_adc_update.emit(adc_value)
    
    def handle_battery_percentage_update(self, percentage: int, status_string: str):
        """Handle battery percentage updates and emit warning CAN messages if needed"""
        # print(f"DEBUG: Battery percentage: {percentage}%, Status: {status_string}")
        
        # Emit the percentage update for GUI
        self.battery_percentage_update.emit(percentage, status_string)
        
        # Send battery status to CAN bus (for LED control)
        self.send_battery_status_to_can(status_string)
    
    def handle_can_signal_received(self):
        """Handle CAN wait signal received from ID 0x69"""
        print("DEBUG: CAN Signal received on ID 0x69 - forwarding to plan executor")
        self.can_signal_received.emit()

    def send_maps(self):
        """Load and send available maps"""
        self.nav_data.load_maps()
        # Load default map if needed
        if self.nav_data.maps:
            default_map = "robots_map"
            if default_map in self.nav_data.maps:
                success, error_msg = self.nav_data.route_manager.load_map_onto_robot(default_map)
                if success:
                    self.nav_data.set_current_map(default_map)
                    print(f"Default map '{default_map}' loaded successfully during connection")
                else:
                    print(f"Warning: Failed to load default map '{default_map}' during connection: {error_msg}")
                    # Still set it as current map so the GUI works, but don't block connection
                    self.nav_data.set_current_map(default_map)
            else:
                # If default map doesn't exist, use first available map
                self.nav_data.set_current_map(self.nav_data.maps[0])
        
        self.send_map_names.emit(self.nav_data.maps)


    def send_routes(self):
        """Send routes for current map"""
        self.nav_data.load_routes()
        self.send_route_names.emit(self.nav_data.routes)

    @pyqtSlot(dict)
    def save_routes(self, new_routes: dict):
        """Save routes for current map"""
        self.nav_data.save_routes(new_routes)

    @pyqtSlot(str)
    def handle_map_selected(self, map_name: str):
        """Handle map selection from GUI"""
        self.nav_data.set_current_map(map_name)
        
        # Emit status update to inform user that map loading is starting
        self.manualStatus.emit(f"Loading map '{map_name}'...")
        
        # Load the map into nav2 navigation stack
        success, error_msg = self.nav_data.route_manager.load_map_onto_robot(map_name)
        if success:
            print(f"Map '{map_name}' loaded into nav2 successfully")
            self.manualStatus.emit(f"Map '{map_name}' loaded successfully")
        else:
            print(f"Failed to load map '{map_name}' into nav2: {error_msg}")
            self.manualStatus.emit(f"Failed to load map '{map_name}': {error_msg}")
        
        self.send_routes()  # Send updated routes for new map
        
    @pyqtSlot()
    def trigger_configure(self):
        self.node.trigger_configure()
        
        # Connect CAN interface when configuring
        if self.can_manager and not self.can_manager.socket_fd:
            self.can_manager.connect_can()
            
        # Start CAN receivers when configuring
        if self.can_battery_receiver:
            self.can_battery_receiver.start_receiving()
            
        if self.can_signal_receiver:
            self.can_signal_receiver.start_receiving()

    @pyqtSlot()
    def trigger_deactivate(self):
        # Stop CAN receivers when deactivating
        if self.can_battery_receiver:
            self.can_battery_receiver.stop_receiving()
            
        if self.can_signal_receiver:
            self.can_signal_receiver.stop_receiving()
            
        # Disconnect CAN interface when deactivating
        if self.can_manager:
            self.can_manager.disconnect_can()
            
        self.node.trigger_deactivate()
        self.node.trigger_shutdown()
    
    @pyqtSlot(float,float,float)
    def set_init_pose(self, x, y, w):
        self.node.set_initial_pose(x,y,w)

    @pyqtSlot(str, float)
    def start_cmd_vel_pub(self, dir: str, vel: float):
        self.manualStatus.emit("Manual move")

        self.node.start_publishing(dir,vel)

    @pyqtSlot()
    def stop_cmd_vel_pub(self):
        self.manualStatus.emit("Idle")

        self.node.stop_publishing()
        
    @pyqtSlot()
    def dock_robot(self, dock_name=None):
        """Slot to handle dock button click with optional dock name"""
        if self.node:
            if self.dock_manager:
                docks = self.dock_manager.load_docks()
                
                if dock_name and dock_name in docks:
                    # Use specific dock coordinates
                    dock = docks[dock_name]
                    print(f"Docking at specific dock '{dock_name}' at coordinates ({dock.x}, {dock.y})")
                    self.node.send_dock_goal(dock.x, dock.y)
                    return
                elif docks:
                    # If no specific dock name but docks exist, use the first available dock
                    first_dock_name = next(iter(docks))
                    dock = docks[first_dock_name]
                    print(f"No specific dock specified, using first available dock '{first_dock_name}' at ({dock.x}, {dock.y})")
                    self.node.send_dock_goal(dock.x, dock.y)
                    return
            
            # Only fallback to live detection if no docks are available at all
            print("No docks available, falling back to live detection")
            self.node.send_dock_goal()
            
    @pyqtSlot()
    def cancel_docking(self):
        """Slot to cancel any ongoing docking operation"""
        if self.node:
            self.node.cancel_dock_goal()

    @pyqtSlot()
    def undock_robot(self):
        """Slot to handle undock button click"""
        if self.node:
            self.node.send_undock_goal()

    @pyqtSlot()
    def cancel_undocking(self):
        """Slot to cancel any ongoing undocking operation"""
        if self.node:
            self.node.cancel_undock_goal()

    def run(self):
        self.node = ManagerNode()
        self.executor.add_node(self.node)

        self.node.service_availability_callback = self.emit_service_availability
        self.node.listener_pose_callback = self.emit_pose 
        self.node.disconnect_callback = self.emit_disconnect_call
        self.node.connect_callback = self.emit_connect_call
        self.node.docking_status_callback = self.emit_docking_status

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
        
    def emit_docking_status(self, status: str):
        self.dockingStatus.emit(status)

    @pyqtSlot(str, bool, float, float)
    def handle_set_route(self, route: str, to_dest: bool, x:float, y:float):
        print(f"New goal set!, To_dest: {to_dest}")
        self.navigator.set_goal(route, to_dest, x, y)
        
        # Send OK CAN message for navigation start (respects battery warning)
        if self.can_manager:
            self.can_manager.send_navigation_start_ok_message()

    @pyqtSlot()
    def stop_nav(self):
        """Stop navigation and cancel any ongoing docking/undocking operations"""
        # Set flag to suppress docking status messages during general stop
        self.node.suppress_docking_status = True
        
        # Force stop navigation
        self.navigator.stop()
        # Also cancel any ongoing docking operations
        self.cancel_docking()
        # Also cancel any ongoing undocking operations
        self.cancel_undocking()
        
        # Always emit "Stopped" status to ensure UI updates
        self.navigator.navStatus.emit("Stopped")
        
        # Send OK CAN message for STOP command (only if battery warning is not active)
        if self.can_manager:
            self.can_manager.send_stop_ok_message()
        
        # Reset the flag after a short delay to allow normal operation to resume
        QTimer.singleShot(500, self.reset_suppress_flag)
    
    def reset_suppress_flag(self):
        """Reset the suppress docking status flag"""
        self.node.suppress_docking_status = False
    
    @pyqtSlot(str)
    def handle_start_plan_execution(self, plan_name: str):
        """Handle plan execution start request"""
        print(f"Starting plan execution: {plan_name}")
        # Forward to plan executor
        self.plan_execution_start.emit(plan_name)
    
    @pyqtSlot()
    def handle_stop_plan_execution(self):
        """Handle plan execution stop request"""
        print("Stopping plan execution")
        
        # Check if we're currently docking/undocking and emit status before suppression
        if self.node and hasattr(self.node, 'current_dock_goal_handle') and self.node.current_dock_goal_handle is not None:
            # We're docking - emit cancellation status before suppression
            self.dockingStatus.emit("Dock Cancelled")
        elif self.node and hasattr(self.node, 'current_undock_goal_handle') and self.node.current_undock_goal_handle is not None:
            # We're undocking - emit cancellation status before suppression  
            self.dockingStatus.emit("Undock Cancelled")
        
        # Stop navigation and cancel any ongoing docking/undocking operations
        self.stop_nav()
        # Forward to plan executor
        self.plan_execution_stop.emit()
    
    @pyqtSlot(str, int)
    def handle_execute_plan_action(self, plan_name: str, action_index: int):
        """Handle execution of a specific plan action"""
        print(f"Executing action {action_index} from plan {plan_name}")
        # Forward to plan executor
        self.plan_action_execute.emit(plan_name, action_index)
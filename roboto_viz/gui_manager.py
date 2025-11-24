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

# Try to import CollisionDetectorState message from various possible packages
try:
    from nav2_msgs.msg import CollisionDetectorState
except ImportError:
    try:
        from collision_detector_msgs.msg import CollisionDetectorState
    except ImportError:
        try:
            from custom_msgs.msg import CollisionDetectorState
        except ImportError:
            try:
                # Try to import a local definition as fallback
                from roboto_viz.collision_detector_state import CollisionDetectorState
                print("Using local CollisionDetectorState definition")
            except ImportError:
                print("Warning: CollisionDetectorState message not found in any known package. Collision detection functionality will be disabled.")
                CollisionDetectorState = None
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
from roboto_viz.env_expression_manager import CANStatusManager
from roboto_viz.modbus_battery_receiver import ModbusBatteryReceiver
from roboto_viz.modbus_button_receiver import ModbusButtonReceiver
from roboto_viz.music_player import MusicPlayer
from roboto_viz.collision_monitor_manager import CollisionMonitorManager


class MapLoadWorker(QThread):
    """Worker thread for loading maps onto the robot in the background"""
    loading_complete = pyqtSignal(bool, str, str)  # success, map_name, error_msg

    def __init__(self, route_manager, map_name):
        super().__init__()
        self.route_manager = route_manager
        self.map_name = map_name

    def run(self):
        """Load the map in background thread"""
        try:
            success, error_msg = self.route_manager.load_map_onto_robot(self.map_name)
            self.loading_complete.emit(success, self.map_name, error_msg)
        except Exception as e:
            self.loading_complete.emit(False, self.map_name, str(e))


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
        self.collision_subscriber = None
        self.cmd_vel_subscriber = None
        self.init_pose_pub = None
        self.cmd_vel_pub = None

        # Velocity tracking for 3Hz updates
        self.last_velocity = 0.0
        self.velocity_update_timer = None

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
        self.collision_detection_callback = None
        self.velocity_update_callback = None

        #INTERNAL STATES:
        self.srv_available: bool = False
        self.suppress_docking_status = False  # Flag to suppress docking status during general stop
        
        self.cmd_vel_msg: Twist = Twist()

        self._current_state: LState = LState.UNCONFIGURED
        self._current_state = LState.UNCONFIGURED

        # Collision monitor manager (initialized but not started yet)
        self.collision_monitor_manager = None

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

        if self.collision_subscriber is None and CollisionDetectorState is not None:
            self.collision_subscriber = self.create_subscription(
                CollisionDetectorState,
                '/collision_detector_state',
                self.collision_callback,
                10
            )
            self.get_logger().info("Created collision detector subscriber")

        if self.cmd_vel_subscriber is None:
            self.cmd_vel_subscriber = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                10
            )
            self.get_logger().info("Created cmd_vel subscriber")

        # Create 3Hz velocity update timer
        if self.velocity_update_timer is None:
            self.velocity_update_timer = self.create_timer(0.333, self.publish_velocity_update)
            self.get_logger().info("Created velocity update timer (3Hz)")

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

        if self.collision_subscriber is not None:
            self.destroy_subscription(self.collision_subscriber)
            self.collision_subscriber = None
            self.get_logger().info("Destroyed collision detector subscriber")

        if self.cmd_vel_subscriber is not None:
            self.destroy_subscription(self.cmd_vel_subscriber)
            self.cmd_vel_subscriber = None
            self.get_logger().info("Destroyed cmd_vel subscriber")

        if self.velocity_update_timer is not None:
            self.destroy_timer(self.velocity_update_timer)
            self.velocity_update_timer = None
            self.get_logger().info("Destroyed velocity update timer")

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

    def cmd_vel_callback(self, msg: Twist):
        """Callback for /cmd_vel topic - stores the current linear velocity"""
        self.last_velocity = abs(msg.linear.x)  # Use absolute value for speed

    def publish_velocity_update(self):
        """Timer callback to publish velocity updates at 3Hz"""
        if self.velocity_update_callback:
            self.velocity_update_callback(self.last_velocity)


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

        # Move wait_for_server to background thread to avoid blocking GUI
        def wait_and_send_dock():
            if self.dock_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info('Sending dock goal...')
                send_goal_future = self.dock_action_client.send_goal_async(
                    goal_msg, feedback_callback=self._dock_feedback_callback)
                send_goal_future.add_done_callback(self._dock_goal_response_callback)
            else:
                self.get_logger().error('Dock action server not available')
                if self.docking_status_callback:
                    self.docking_status_callback("Dock Server Unavailable")

        # Execute in background thread to prevent blocking
        dock_thread = threading.Thread(target=wait_and_send_dock, daemon=True)
        dock_thread.start()

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
                    self.docking_status_callback("Zadokowany" if result.docked else "Dokowanie Nieudane")
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Docking was cancelled')
                if self.docking_status_callback and not self.suppress_docking_status:
                    self.docking_status_callback("Dokowanie Anulowane")
            else:
                self.get_logger().info('Docking failed')
                if self.docking_status_callback:
                    self.docking_status_callback("Dokowanie Nieudane")
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

        # Move wait_for_server to background thread to avoid blocking GUI
        def wait_and_send_undock():
            if self.undock_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info('Sending undock goal...')
                send_goal_future = self.undock_action_client.send_goal_async(
                    goal_msg, feedback_callback=self._undock_feedback_callback)
                send_goal_future.add_done_callback(self._undock_goal_response_callback)
            else:
                self.get_logger().error('Undock action server not available')
                if self.docking_status_callback:
                    self.docking_status_callback("Undock Server Unavailable")

        # Execute in background thread to prevent blocking
        undock_thread = threading.Thread(target=wait_and_send_undock, daemon=True)
        undock_thread.start()

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
        
    def collision_callback(self, msg):
        """Handle collision detector state messages"""
        if msg.detections:
            # Check if any detection is true
            collision_detected = any(msg.detections)
            
            # Call the callback if set
            if self.collision_detection_callback:
                self.collision_detection_callback(collision_detected)
        else:
            # No detections array, assume no collision
            if self.collision_detection_callback:
                self.collision_detection_callback(False)
        
class NavData(QObject):
    def __init__(self):
        super().__init__()
        self.route_manager = RouteManager()
        self.navigator: BasicNavigator = BasicNavigator('gui_navigator_node')

        # Set navigator node log level to WARN to suppress verbose output
        try:
            from rclpy.logging import LoggingSeverity
            if hasattr(self.navigator, 'node'):
                self.navigator.node.get_logger().set_level(LoggingSeverity.WARN)
        except Exception:
            pass  # If setting log level fails, continue anyway

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
    navigation_started = pyqtSignal()  # Emitted when navigation actually starts (after followPath completes)

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
        
        # Always emit "Stopped" status when stop is called - do not preserve error status
        # This ensures the STOP button always sends an OK CAN message
        self._last_status = "Zatrzymany"
        self.navStatus.emit("Zatrzymany")


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

                print(f"NAV DEBUG: Set goal for route '{route}', generated {len(waypoints)} waypoints, to_dest={to_dest}")

                # Don't reverse here - let the run() method handle direction

                # Cancel current navigation if there is one
                if not self.nav_data.navigator.isTaskComplete():
                    self.nav_data.navigator.cancelTask()

                if to_dest == True:
                    self._last_status = "Nav to dest"
                    self.navStatus.emit("Nawigacja do celu")
                else:
                    self._last_status = "Nav to base"
                    self.navStatus.emit("Nawigacja do bazy")
            else:
                print(f"NAV ERROR: Route '{route}' not found in routes. Available routes: {list(self.nav_data.routes.keys())}")

    def run(self):
        print("NAV DEBUG: Navigator thread started")
        while self._running:
            # Check if we have a new goal
            with self._goal_lock:
                current_goal = self._new_goal
                self._new_goal = None

            if current_goal is None:
                # No goal to process, wait a bit
                self.msleep(100)  # Sleep for 100ms
                continue

            print(f"NAV DEBUG: Navigator thread processing goal with {len(current_goal)} waypoints")
                
            try:
                # Simplify direction logic: just reverse path order for return trip
                if self._to_dest:
                    # Going to destination: use waypoints in original order
                    waypoints = current_goal
                else:
                    # Going to base: use waypoints in reverse order
                    waypoints = list(reversed(current_goal))

                print(f"NAV DEBUG: Robot position: ({self.curr_x}, {self.curr_y})")
                print(f"NAV DEBUG: Waypoints before filtering: {len(waypoints)}")

                # Find closest waypoint to start from
                distances = [math.dist((self.curr_x, self.curr_y), (waypoint[0], waypoint[1])) for waypoint in waypoints]
                closest_index = distances.index(min(distances))
                print(f"NAV DEBUG: Closest waypoint index: {closest_index}, distance: {distances[closest_index]:.2f}m")
                waypoints = waypoints[closest_index:]
                
                # Create Path message for followPath
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                # Don't set timestamp - let nav2 handle it

                print(f"NAV DEBUG: Creating path with {len(waypoints)} waypoints after filtering")

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

                        # print(f"DEBUG: Final waypoint calculated orientation: {orientation} radians ({math.degrees(orientation)} degrees}")
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

                # Start navigation - CAN is already paused so we can block here safely
                print(f"NAV DEBUG: About to call followPath with {len(path_msg.poses)} poses")
                if len(path_msg.poses) > 0:
                    print(f"NAV DEBUG: First pose: x={path_msg.poses[0].pose.position.x}, y={path_msg.poses[0].pose.position.y}")
                    print(f"NAV DEBUG: Last pose: x={path_msg.poses[-1].pose.position.x}, y={path_msg.poses[-1].pose.position.y}")
                else:
                    print("NAV ERROR: path_msg.poses is EMPTY! This is the bug!")

                try:
                    # Wait for nav2 to be active
                    self.nav_data.navigator.waitUntilNav2Active()

                    # Start path navigation
                    self.nav_data.navigator.followPath(
                        path_msg,
                        controller_id='FollowPath',
                        goal_checker_id='goal_checker'
                    )
                    print("NAV DEBUG: followPath() completed, navigation active")

                    # Emit signal that navigation has actually started
                    self.navigation_started.emit()

                except Exception as e:
                    print(f"NAV ERROR: followPath() failed: {e}")
                    self._last_status = "Błąd - Nawigacja"
                    self.navStatus.emit("Błąd - Nawigacja")
                    continue

                while not self.nav_data.navigator.isTaskComplete() and self._running:
                    # Check if there's a new goal
                    if self._new_goal is not None:
                        break  # Exit this loop to process the new goal
                        
                    self.msleep(100)  # Small delay to prevent CPU hogging
                
                if self._running and self.nav_data.navigator.isTaskComplete():
                    if self.nav_data.navigator.getResult() is TaskResult.SUCCEEDED:
                        if self._to_dest:
                            self._last_status = "Na miejscu docelowym"
                            self.navStatus.emit("Na miejscu docelowym")
                        else:
                            self._last_status = "W bazie"
                            self.navStatus.emit("W bazie")
                    elif self.nav_data.navigator.getResult() is TaskResult.FAILED:
                        # Try to get detailed error message from nav2
                        error_msg = "Failed"
                        try:
                            # Get detailed feedback from the navigator
                            if hasattr(self.nav_data.navigator, '_action_client'):
                                action_client = self.nav_data.navigator._action_client
                                if hasattr(action_client, '_goal_handle') and action_client._goal_handle:
                                    # Try to get the result with more detail
                                    result = action_client._goal_handle.get_result()
                                    if hasattr(result, 'result') and hasattr(result.result, 'error_code'):
                                        error_code = result.result.error_code
                                        if error_code != 0:
                                            error_msg = f"Błąd Nav2 {error_code}"
                            
                            # Also check for common nav2 error patterns in logs
                            # Since we can't directly access nav2 logs, provide more context
                            if hasattr(self.nav_data.navigator, 'getFeedback'):
                                feedback = self.nav_data.navigator.getFeedback()
                                if feedback and hasattr(feedback, 'distance_remaining'):
                                    if feedback.distance_remaining > 100.0:  # Very far from goal
                                        error_msg = "Błąd - Cel nieosiągalny"
                                    else:
                                        error_msg = "Błąd - Przekroczono czas nawigacji"
                        except Exception as e:
                            # print(f"DEBUG: Could not extract detailed nav2 error: {e}")
                            pass
                            error_msg = "Błąd - Błąd Nav2"
                        
                        self._last_status = error_msg
                        self.navStatus.emit(error_msg)
                    
                    self.finished.emit()
                    
            except Exception as e:
                self._last_status = "Błąd Nawigacji"
                self.navStatus.emit("Błąd Nawigacji")
                # Only print critical navigation errors
                if "timeout" not in str(e).lower():
                    print(f"Navigation error: {e}")

class GuiManager(QThread):
    manualStatus = pyqtSignal(str)
    dockingStatus = pyqtSignal(str)

    # Additional status signals for CAN forwarding
    robotStatusCAN = pyqtSignal(str)  # For robot status updates
    batteryStatusCAN = pyqtSignal(str)  # For battery status updates
    planStatusCAN = pyqtSignal(str)  # For plan status updates

    service_response = pyqtSignal(bool)
    update_pose = pyqtSignal(float, float, float)

    # Velocity update signal
    velocity_update = pyqtSignal(float)  # Linear velocity in m/s
    
    # Battery ADC value signal
    battery_adc_update = pyqtSignal(int)  # Raw ADC value (0-1023)
    battery_percentage_update = pyqtSignal(int, str)  # Battery percentage and status string
    
    # CAN wait signal
    can_signal_received = pyqtSignal()  # CAN signal for wait actions
    wait_action_status = pyqtSignal(str)  # Wait action status for CAN WARNING messages
    
    # Collision detection signal
    collision_detected = pyqtSignal(bool)  # Collision detection status

    # Navigation lifecycle signals
    navigation_actually_started = pyqtSignal()  # Emitted when navigation actually begins (after nav2 service completes)

    # Plan execution signals
    plan_execution_start = pyqtSignal(str)  # plan_name
    plan_execution_stop = pyqtSignal()
    plan_action_execute = pyqtSignal(str, int)  # plan_name, action_index

    # Collision zone display signals
    show_collision_zones = pyqtSignal(str, object)  # route_name, color_cache - show zones for this route
    hide_collision_zones = pyqtSignal()  # hide all collision zones

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

    def __init__(self, dock_manager=None, can_interface="can0", enable_can=True,
                 modbus_port="/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if00-port0", modbus_slave_id=1,
                 button_poll_interval=0.1, battery_poll_interval=0.5):
        super().__init__()
        self.node: ManagerNode = None
        self.executor = MultiThreadedExecutor()
        self.nav_data: NavData = NavData()
        self.navigator: Navigator = Navigator(self.nav_data)
        self.dock_manager = dock_manager

        # Map loading worker thread
        self.map_load_worker = None

        # Initialize Modbus status manager (for LEDs and buzzer)
        # This creates the shared Modbus instrument and lock
        self.can_manager = None
        # Initialize Modbus receivers (for battery and button)
        self.modbus_battery_receiver = None
        self.modbus_button_receiver = None
        if enable_can:
            # Create shared LED/buzzer manager with instrument and lock
            self.can_manager = CANStatusManager(modbus_port, modbus_slave_id)

            # Create receivers using the shared instrument and lock
            # Note: instrument will be None until connect_can() is called
            self.modbus_battery_receiver = ModbusBatteryReceiver(
                shared_instrument=None,  # Will be set in trigger_configure
                instrument_lock=self.can_manager.instrument_lock,
                poll_interval=battery_poll_interval)
            self.modbus_button_receiver = ModbusButtonReceiver(
                shared_instrument=None,  # Will be set in trigger_configure
                instrument_lock=self.can_manager.instrument_lock,
                poll_interval=button_poll_interval)
            self._connect_modbus_signals()

            # Start battery receiver immediately so battery status is available from app start
            # Note: Will only work after trigger_configure() sets instrument
            if self.modbus_battery_receiver:
                pass  # Don't start until instrument is connected

        # Initialize music player
        self.music_player = MusicPlayer()

        # Connect navigation status to music player
        if hasattr(self.navigator, 'navStatus'):
            self.navigator.navStatus.connect(self.music_player.handle_navigation_status)

        # self.nav_data.send_routes.connect(lambda: self.send_route_names)

        self.navigator.start()

    def _connect_modbus_signals(self):
        """Connect all status signals to Modbus manager and receivers"""
        if not self.can_manager:
            return

        # Connect the various status signals to Modbus handlers
        self.manualStatus.connect(self.can_manager.handle_manual_status)
        self.dockingStatus.connect(self.can_manager.handle_docking_status)
        self.robotStatusCAN.connect(self.can_manager.handle_robot_status)
        self.batteryStatusCAN.connect(self.can_manager.handle_battery_status)
        self.planStatusCAN.connect(self.can_manager.handle_plan_status)
        self.wait_action_status.connect(self.can_manager.handle_wait_action_status)

        # Connect collision detection signal to buzzer control
        self.collision_detected.connect(self.can_manager.handle_collision_detection)

        # Connect navigation status from navigator if available
        if hasattr(self.navigator, 'navStatus'):
            self.navigator.navStatus.connect(self.can_manager.handle_navigation_status)

        # Connect navigation_started signal to send Modbus message after nav starts
        if hasattr(self.navigator, 'navigation_started'):
            self.navigator.navigation_started.connect(self.handle_navigation_started)

        # Connect battery receiver if available
        if self.modbus_battery_receiver:
            self.modbus_battery_receiver.battery_status_update.connect(self.handle_battery_adc_update)
            self.modbus_battery_receiver.battery_percentage_update.connect(self.handle_battery_percentage_update)
            # Connect navigation signals to battery receiver
            if hasattr(self.navigator, 'navigation_started'):
                self.navigator.navigation_started.connect(
                    lambda: self.modbus_battery_receiver.set_navigation_state(True))
            self.navigator.finished.connect(
                lambda: self.modbus_battery_receiver.set_navigation_state(False))

        # Connect button receiver if available
        if self.modbus_button_receiver:
            self.modbus_button_receiver.signal_received.connect(self.handle_button_click_received)
    
    def send_robot_status_to_can(self, status: str):
        """Send robot status to CAN bus via signal"""
        self.robotStatusCAN.emit(status)

    def send_battery_status_to_can(self, status: str):
        """Send battery status to CAN bus via signal"""
        self.batteryStatusCAN.emit(status)

    def send_plan_status_to_can(self, status: str):
        """Send plan status to CAN bus via signal"""
        self.planStatusCAN.emit(status)

    @pyqtSlot()
    def handle_navigation_started(self):
        """Handle navigation actually starting - resume CAN and send start message"""
        # Emit signal to notify all systems that navigation has actually started
        self.navigation_actually_started.emit()

        # Resume CAN messages now that navigation has started
        if self.can_manager:
            self.can_manager.resume_can_messages()

        # Use QTimer to add a delay before sending CAN messages
        # This prevents overwhelming the CAN bus when nav2 service calls complete
        QTimer.singleShot(500, self._send_navigation_start_can_message)

    def _send_navigation_start_can_message(self):
        """Send CAN message for navigation start after delay"""
        # print("NAV DEBUG: Sending navigation start CAN message now")
        if self.can_manager:
            self.can_manager.send_navigation_start_ok_message()
    
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
    
    def handle_button_click_received(self):
        """Handle Modbus button click signal"""
        self.can_signal_received.emit()

    def send_maps(self):
        """Load and send available maps"""
        self.nav_data.load_maps()
        # Set default map for GUI (but don't load it onto robot yet)
        if self.nav_data.maps:
            default_map = "robots_map"
            if default_map in self.nav_data.maps:
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
        """Handle map selection from GUI - loads map in background thread"""
        # print(f"DEBUG: GuiManager.handle_map_selected called with map: {map_name}")
        self.nav_data.set_current_map(map_name)
        # print(f"DEBUG: nav_data.current_map set to: {self.nav_data.current_map}")
        # print(f"DEBUG: nav_data routes loaded: {list(self.nav_data.routes.keys())}")

        # Emit status update to inform user that map loading is starting
        self.manualStatus.emit(f"Ładowanie mapy '{map_name}'...")

        # Cancel any existing map loading operation
        if self.map_load_worker and self.map_load_worker.isRunning():
            # print("DEBUG: Cancelling previous map load operation")
            pass
            self.map_load_worker.quit()
            self.map_load_worker.wait()

        # Create and start worker thread to load map in background
        self.map_load_worker = MapLoadWorker(self.nav_data.route_manager, map_name)
        self.map_load_worker.loading_complete.connect(self._on_map_load_complete)
        self.map_load_worker.start()

    def _on_map_load_complete(self, success: bool, map_name: str, error_msg: str):
        """Handle completion of background map loading"""
        if success:
            self.manualStatus.emit(f"Mapa '{map_name}' załadowana pomyślnie")
        else:
            self.manualStatus.emit(f"Nie udało się załadować mapy '{map_name}': {error_msg}")

        # Initialize/update collision monitor manager for this map
        self._setup_collision_monitor(map_name)

        self.send_routes()  # Send updated routes for new map

    def _setup_collision_monitor(self, map_name: str):
        """Setup collision monitor manager for the current map"""
        from pathlib import Path
        import yaml

        try:
            maps_dir = Path.home() / ".robotroutes" / "maps"
            yaml_path = maps_dir / f"{map_name}.yaml"

            if not yaml_path.exists():
                # print(f"DEBUG: YAML file not found for collision monitor setup: {yaml_path}")
                return

            with open(yaml_path, 'r') as file:
                yaml_data = yaml.safe_load(file)

            origin = yaml_data.get('origin')
            resolution = yaml_data.get('resolution')

            if not origin or not resolution:
                # print(f"DEBUG: Missing origin or resolution in YAML for collision monitor")
                return

            # Create collision monitor manager if it doesn't exist
            if not self.node.collision_monitor_manager:
                self.node.collision_monitor_manager = CollisionMonitorManager(self.node)

            # Set the current map
            self.node.collision_monitor_manager.set_current_map(map_name, origin, resolution)

            # Start monitoring
            self.node.collision_monitor_manager.start_monitoring()

        except Exception as e:
            print(f"ERROR: Failed to setup collision monitor: {e}")
            import traceback
            traceback.print_exc()

    @pyqtSlot()
    def trigger_configure(self):
        self.node.trigger_configure()

        # Connect Modbus interface when configuring
        if self.can_manager and not self.can_manager.instrument:
            self.can_manager.connect_can()

            # Share the connected instrument with battery and button receivers
            if self.modbus_battery_receiver:
                self.modbus_battery_receiver.instrument = self.can_manager.instrument
                self.modbus_battery_receiver.start_receiving()

            if self.modbus_button_receiver:
                self.modbus_button_receiver.instrument = self.can_manager.instrument
                self.modbus_button_receiver.start_receiving()

    @pyqtSlot()
    def trigger_deactivate(self):
        # Stop button receiver when deactivating
        # Note: Battery receiver keeps running to show battery status even when disconnected
        if self.modbus_button_receiver:
            self.modbus_button_receiver.stop_receiving()

        # Disconnect Modbus interface when deactivating
        if self.can_manager:
            self.can_manager.disconnect_can()

        self.node.trigger_deactivate()
        self.node.trigger_shutdown()
    
    @pyqtSlot(float,float,float)
    def set_init_pose(self, x, y, w):
        self.node.set_initial_pose(x,y,w)

    @pyqtSlot(str, float)
    def start_cmd_vel_pub(self, dir: str, vel: float):
        self.manualStatus.emit("Ruch ręczny")

        self.node.start_publishing(dir,vel)

    @pyqtSlot()
    def stop_cmd_vel_pub(self):
        self.manualStatus.emit("Bezczynny")

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
        self.node.collision_detection_callback = self.emit_collision_detection
        self.node.velocity_update_callback = self.emit_velocity_update

        self.executor.spin()

    def stop(self):
        if self.navigator:
            self.navigator.stop()
        if self.music_player:
            self.music_player.cleanup()
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

        # Update collision monitor manager with robot position
        if self.node.collision_monitor_manager:
            self.node.collision_monitor_manager.update_robot_pose(x, y)
        
    def emit_docking_status(self, status: str):
        self.dockingStatus.emit(status)
    
    def emit_collision_detection(self, collision_detected: bool):
        """Emit collision detection status"""
        self.collision_detected.emit(collision_detected)

        # Note: The collision_detected signal is handled by plan_active_view.handle_collision_detection
        # which displays "Wykryto przeszkodę!" in the robot status with orange background
        # We don't need to emit navStatus here as handle_collision_detection handles the display

    def emit_velocity_update(self, velocity: float):
        """Emit velocity update signal"""
        self.velocity_update.emit(velocity)

    @pyqtSlot(str, bool, float, float)
    def handle_set_route(self, route: str, to_dest: bool, x:float, y:float):
        # Pause CAN messages before starting navigation to prevent buffer overflow
        # Navigation start can take several seconds and CAN messages would accumulate
        if self.can_manager:
            self.can_manager.pause_can_messages()

        # Load collision zones for this route FIRST (builds the color cache)
        # This is now FAST due to numpy optimization
        color_cache = None
        if self.node and self.node.collision_monitor_manager:
            self.node.collision_monitor_manager.load_collision_zones_for_route(route)
            self.node.collision_monitor_manager.start_monitoring()
            # Get the color cache for fast minimap rendering
            color_cache = self.node.collision_monitor_manager.color_cache

        # Show collision zones on minimap using the pre-built cache (FAST)
        # Emit signal with color cache (or None if not available)
        print(f"INFO: GuiManager emitting show_collision_zones signal for route '{route}', cache available: {color_cache is not None}")
        self.show_collision_zones.emit(route, color_cache)

        # Set navigation goal
        self.navigator.set_goal(route, to_dest, x, y)

        # Note: CAN messages will be resumed after navigation actually starts
        # via the navigation_started signal connection

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

        # Hide collision zones when navigation stops
        self.hide_collision_zones.emit()

        # Only emit "Stopped" if navigator didn't preserve a failure status
        if not (self.navigator._last_status and ("fail" in self.navigator._last_status.lower() or "error" in self.navigator._last_status.lower())):
            self.navigator.navStatus.emit("Zatrzymany")

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
        # Forward to plan executor
        self.plan_execution_start.emit(plan_name)
    
    @pyqtSlot()
    def handle_stop_plan_execution(self):
        """Handle plan execution stop request"""
        
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
        # Forward to plan executor
        self.plan_action_execute.emit(plan_name, action_index)
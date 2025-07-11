from __future__ import annotations
from roboto_viz.gui_manager import GuiManager
from roboto_viz.main_view import MainView

from abc import ABC, abstractmethod

import rclpy

class Gui:
    def __init__(self) -> None:
        rclpy.init()

        self._state = None
        self.gui_manager: GuiManager = GuiManager()
        self.main_view: MainView = MainView()
        self.transition_to(DisconnectedState())

    def setup(self):
        self.handleGui()
        self.gui_manager.start()

    def transition_to(self, state: State):
        print(f"Context: Transition to {type(state).__name__}")
        if self._state is not None:
            self._state.cleanup_connections()
        self._state = state
        self._state.gui = self

    def handleGui(self):
        self._state.handleGui()

class State(ABC):
    def __init__(self):
        super().__init__()
        self._connections = []
    
    @property
    def gui(self) -> Gui:
        return self._gui

    @gui.setter
    def gui(self, gui: Gui) -> None:
        self._gui = gui

    def cleanup_connections(self):
        """Disconnect all signals registered in this state"""
        for connection in self._connections:
            try:
                connection.disconnect()
            except TypeError:
                # Handle case where connection might already be disconnected
                pass
        self._connections.clear()

    def connect_and_store_connections(self, signal, slot):
        """Connect a signal to a slot and store the connection"""
        connection = signal.connect(slot)
        self._connections.append(signal)
        return connection

    @abstractmethod
    def handleGui(self) -> None:
        pass

class DisconnectedState(State):
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_disconnected()

        self.gui.main_view.map_view.enable_drawing = False

        self.connect_and_store_connections(
            self.gui.gui_manager.service_availability,
            self.gui.main_view.disconnected_view.set_availability
        )

        self.connect_and_store_connections(
            self.gui.main_view.connection_signal,
            self.handleConfigure
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.trigger_connection,
            self.handleConnection
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.send_map_names,
            self.gui.main_view.active_view.load_maps
        )
        
        # Also load maps for plan system
        self.connect_and_store_connections(
            self.gui.gui_manager.send_map_names,
            self.gui.main_view.load_maps_into_plan_system
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.trigger_disconnect,
            self.handleDisconnection
        )

    def handleDisconnection(self):
        self.gui.main_view.disconnected_view.waiting_for_connection(False)
        self.gui.handleGui()

    def handleConfigure(self):
        self.gui.main_view.disconnected_view.waiting_for_connection(True)
        self.gui.gui_manager.send_maps()
        self.gui.gui_manager.send_routes()
        self.gui.gui_manager.trigger_configure()

    def handleConnection(self):
        self.gui.main_view.load_map("robots_map")
        if hasattr(self.gui.main_view, 'use_plan_system') and self.gui.main_view.use_plan_system:
            self.gui.transition_to(PlanActiveState())
        else:
            self.gui.transition_to(ActiveState())
        self.gui.main_view.disconnected_view.waiting_for_connection(False)
        self.gui.handleGui()


class ConfiguringState(State):
    def handleGui(self):
        self.gui.main_view.switch_to_configuring()

        self.gui.main_view.map_view.enable_drawing = True


        if hasattr(self.gui.main_view, 'use_plan_system') and self.gui.main_view.use_plan_system:
            # Use plan system configure connections
            pass  # Plan system handles this differently
        else:
            self.connect_and_store_connections(
                self.gui.main_view.active_view.active_tools.switch_to_active,
                self.handle_activate
            )

        self.connect_and_store_connections(
            self.gui.gui_manager.update_pose,
            self.gui.main_view.active_view.update_robot_pose
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.send_map_names,
            self.gui.main_view.active_view.active_tools.update_maps
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.map_selected,
            self.gui.gui_manager.handle_map_selected # KEEP IT IN MAIN_VIEW!!
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.map_selected,
            self.gui.main_view.load_map # KEEP IT IN MAIN_VIEW!!
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.start_keys_vel,
            self.gui.gui_manager.start_cmd_vel_pub 
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.stop_keys_vel,
            self.gui.gui_manager.stop_cmd_vel_pub 
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.map_selected,
            self.gui.gui_manager.nav_data.route_manager.load_map_onto_robot # KEEP IT IN MAIN_VIEW!!
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.manualStatus,
            self.gui.main_view.active_view.active_tools.set_current_status
        )

        self.connect_and_store_connections(
            self.gui.main_view.set_position_signal,
            self.gui.gui_manager.set_init_pose
        )

        # Connect docking signals
        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.dock_robot,
            self.gui.gui_manager.dock_robot
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.undock_robot,
            self.gui.gui_manager.undock_robot
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.dockingStatus,
            self.gui.main_view.active_view.active_tools.set_current_status
        )

    def handle_activate(self):
        if hasattr(self.gui.main_view, 'use_plan_system') and self.gui.main_view.use_plan_system:
            self.gui.transition_to(PlanActiveState())
        else:
            self.gui.transition_to(ActiveState())
        self.gui.handleGui()

class ActiveState(State):
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_active()

        self.gui.main_view.map_view.enable_drawing = False

        if hasattr(self.gui.main_view, 'use_plan_system') and self.gui.main_view.use_plan_system:
            # Plan system uses different connections
            pass  # Will be handled in PlanActiveState
        else:
            self.connect_and_store_connections(
                self.gui.main_view.active_view.active_tools.switch_to_configure,
                self.handle_configuring
            )

        self.connect_and_store_connections(
            self.gui.gui_manager.trigger_disconnect,
            self.handleDisconnection
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.start_nav,
            self.gui.gui_manager.handle_set_route
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.stop_nav,
            self.gui.gui_manager.stop_nav
        )

        if hasattr(self.gui.main_view, 'use_plan_system') and self.gui.main_view.use_plan_system:
            self.connect_and_store_connections(
                self.gui.gui_manager.update_pose,
                self.gui.main_view.update_robot_pose_plan_system
            )
        else:
            self.connect_and_store_connections(
                self.gui.gui_manager.update_pose,
                self.gui.main_view.active_view.update_robot_pose
            )

        self.connect_and_store_connections(
            self.gui.main_view.start_planning,
            self.startPlanning
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.send_route_names,
            self.gui.main_view.active_view.load__routes
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.save_current_routes,
            self.gui.gui_manager.save_routes
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.navigator.navStatus,
            self.gui.main_view.active_view.active_tools.set_current_status
        )

        # Connect docking signals
        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.dock_robot,
            self.gui.gui_manager.dock_robot
        )

        self.connect_and_store_connections(
            self.gui.main_view.active_view.active_tools.undock_robot,
            self.gui.gui_manager.undock_robot
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.dockingStatus,
            self.gui.main_view.active_view.active_tools.set_current_status
        )

        self.gui.gui_manager.send_routes()

    def handleDisconnection(self):
        self.gui.transition_to(DisconnectedState())
        self.gui.gui_manager.stop_nav()
        self.gui.main_view.active_view.active_tools.set_current_status("Idle")
        self.gui.handleGui()
    
    def startPlanning(self):
        # self.gui.main_view.set_position_signal.disconnect()
        self.gui.transition_to(PlannerState())
        self.gui.gui_manager.stop_nav()
        self.gui.main_view.active_view.active_tools.set_current_status("Idle")
        self.gui.handleGui()

    def handle_configuring(self):
        self.gui.transition_to(ConfiguringState())
        self.gui.gui_manager.stop_nav()
        self.gui.main_view.active_view.active_tools.set_current_status("Idle")
        self.gui.handleGui()

#TODO: handleDisconnection in the planner mode
class PlannerState(State):
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_planner()
        self.gui.main_view.map_view.clear_route()  # Clear route visualization for fresh start

        self.gui.main_view.map_view.enable_drawing = True

        self.connect_and_store_connections(
            self.gui.gui_manager.send_route_names,
            self.gui.main_view.active_view.load__routes
        )

        self.connect_and_store_connections(
            self.gui.main_view.finish_planning,
            self.finishPlanning
        )

        # Note: setPoint connection removed - users now click directly on map to add nodes
        
        self.connect_and_store_connections(
            self.gui.gui_manager.update_pose,
            self.gui.main_view.active_view.update_robot_pose
        )
        self.connect_and_store_connections(
            self.gui.main_view.active_view.planning_tools.save_current_routes,
            self.gui.gui_manager.save_routes
        )

        self.gui.gui_manager.send_routes()

    def finishPlanning(self):
        # Note: setPoint disconnection removed - connection no longer exists
        if hasattr(self.gui.main_view, 'use_plan_system') and self.gui.main_view.use_plan_system:
            self.gui.transition_to(PlanActiveState())
        else:
            self.gui.transition_to(ActiveState())
        self.gui.handleGui()


class PlanActiveState(State):
    """New state for plan-based robot control"""
    
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_active()

        self.gui.main_view.map_view.enable_drawing = False

        # Plan execution connections
        self.connect_and_store_connections(
            self.gui.main_view.start_plan_execution,
            self.gui.gui_manager.handle_start_plan_execution
        )
        
        self.connect_and_store_connections(
            self.gui.main_view.stop_plan_execution,
            self.gui.gui_manager.handle_stop_plan_execution
        )
        
        self.connect_and_store_connections(
            self.gui.main_view.execute_plan_action,
            self.gui.gui_manager.handle_execute_plan_action
        )

        # Robot control connections
        self.connect_and_store_connections(
            self.gui.main_view.start_nav,
            self.gui.gui_manager.handle_set_route
        )
        
        self.connect_and_store_connections(
            self.gui.main_view.dock_robot,
            self.gui.gui_manager.dock_robot
        )

        self.connect_and_store_connections(
            self.gui.main_view.undock_robot,
            self.gui.gui_manager.undock_robot
        )

        # Manual control connections
        self.connect_and_store_connections(
            self.gui.main_view.start_keys_vel,
            self.gui.gui_manager.start_cmd_vel_pub
        )

        self.connect_and_store_connections(
            self.gui.main_view.stop_keys_vel,
            self.gui.gui_manager.stop_cmd_vel_pub
        )

        # Pose updates
        self.connect_and_store_connections(
            self.gui.gui_manager.update_pose,
            self.gui.main_view.update_robot_pose_plan_system
        )

        # Map and position connections
        self.connect_and_store_connections(
            self.gui.main_view.set_position_signal,
            self.gui.gui_manager.set_init_pose
        )
        
        # Map change notification for navigation system
        self.connect_and_store_connections(
            self.gui.main_view.map_changed_signal,
            self.gui.gui_manager.handle_map_selected
        )

        # Disconnection handling
        self.connect_and_store_connections(
            self.gui.gui_manager.trigger_disconnect,
            self.handleDisconnection
        )

        # Status updates
        self.connect_and_store_connections(
            self.gui.gui_manager.manualStatus,
            lambda status: self.gui.main_view.plan_active_view.update_robot_status(status)
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.dockingStatus,
            lambda status: self.gui.main_view.plan_active_view.update_robot_status(status)
        )

        self.connect_and_store_connections(
            self.gui.gui_manager.navigator.navStatus,
            lambda status: self.gui.main_view.plan_active_view.update_robot_status(status)
        )
        
        # Connect navigation completion to plan executor
        self.connect_and_store_connections(
            self.gui.gui_manager.navigator.finished,
            self.gui.main_view.plan_executor.on_navigation_completed
        )

    def handleDisconnection(self):
        self.gui.transition_to(DisconnectedState())
        self.gui.gui_manager.stop_nav()
        self.gui.main_view.plan_active_view.update_robot_status("Idle")
        self.gui.handleGui()
# model should be the guiController -> is notifies view of any changes (updates the position for example), the Gui class is a controller to be fair xdd

from __future__ import annotations
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QStackedWidget
from abc import ABC, abstractmethod

from PyQt5.QtCore import pyqtSignal, QObject

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

from roboto_viz.gui_manager import GuiManager
from roboto_viz.views import MainView

class Gui:
    _state = None

    def __init__(self) -> None:
        rclpy.init()
        # self.transition_to(state)

        self.transition_to(DisconnectedState())

        self.gui_manager: GuiManager = GuiManager()
        self.main_view: MainView = MainView()

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

    def request2(self):
        self._state.handle2()

class State(ABC):
    def __init__(self):
        super().__init__()
        self._connections = []  # Store signal connections
    
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

    def connect_and_store(self, signal, slot):
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

        self.connect_and_store(
            self.gui.gui_manager.service_availability,
            self.gui.main_view.disconnected_view.set_availability
        )

        self.connect_and_store(
            self.gui.main_view.connection_signal,
            self.handleConfigure
        )

        self.connect_and_store(
            self.gui.gui_manager.trigger_connection,
            self.handleConnection
        )

        self.connect_and_store(
            self.gui.gui_manager.send_map_names,
            self.gui.main_view.active_view.load_maps
        )

    def handleConfigure(self):
        self.gui.gui_manager.nav_data.route_manager.load_map("robots_map")
        # self.gui.gui_manager.nav_data.route_manager.load_map_onto_robot("test")
        self.gui.gui_manager.send_maps()
        self.gui.main_view.load_map("robots_map")
        self.gui.gui_manager.trigger_configure()
        self.gui.main_view.disconnected_view.waiting_for_connection(True)

    def handleConnection(self):
        self.gui.transition_to(ConfiguringState())
        self.gui.main_view.disconnected_view.waiting_for_connection(False)
        self.gui.handleGui()


class ConfiguringState(State):
    def handleGui(self):
        self.gui.main_view.switch_to_configuring()

        self.connect_and_store(
            self.gui.main_view.set_position_signal,
            self.gui.gui_manager.set_init_pose
        )

        self.connect_and_store(
            self.gui.main_view.active_view.active_tools.switch_to_active,
            self.handle_activate
        )

        self.connect_and_store(
            self.gui.gui_manager.update_pose,
            self.gui.main_view.active_view.map_view.update_robot_pose
        )

        self.connect_and_store(
            self.gui.main_view.active_view.active_tools.map_selected,
            self.gui.main_view.load_map # KEEP IT IN MAIN_VIEW!!
        )

        self.connect_and_store(
            self.gui.main_view.active_view.active_tools.map_selected,
            self.gui.gui_manager.nav_data.route_manager.load_map_onto_robot # KEEP IT IN MAIN_VIEW!!
        )
    def handle_activate(self):
        self.gui.transition_to(ActiveState())
        self.gui.handleGui()

class ActiveState(State):
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_active()

        self.connect_and_store(
            self.gui.main_view.active_view.active_tools.switch_to_configure,
            self.handle_configuring
        )

        self.connect_and_store(
            self.gui.gui_manager.trigger_disconnect,
            self.handleDisconnection
        )

        self.connect_and_store(
            self.gui.main_view.active_view.active_tools.start_nav,
            self.gui.gui_manager.handle_set_route
        )

        self.connect_and_store(
            self.gui.main_view.active_view.active_tools.stop_nav,
            self.gui.gui_manager.stop_nav
        )

        self.connect_and_store(
            self.gui.gui_manager.update_pose,
            self.gui.main_view.active_view.map_view.update_robot_pose
        )

        self.connect_and_store(
            self.gui.main_view.start_planning,
            self.startPlanning
        )

        self.connect_and_store(
            self.gui.gui_manager.send_route_names,
            self.gui.main_view.active_view.load__routes
        )

        self.connect_and_store(
            self.gui.main_view.active_view.active_tools.save_current_routes,
            self.gui.gui_manager.save_routes
        )

        self.gui.gui_manager.send_routes()

    def handleDisconnection(self):
        self.gui.transition_to(DisconnectedState())
        self.gui.handleGui()
    
    def startPlanning(self):
        # self.gui.main_view.set_position_signal.disconnect()
        self.gui.transition_to(PlannerState())
        self.gui.handleGui()

    def handle_configuring(self):
        self.gui.transition_to(ConfiguringState())
        self.gui.handleGui()

#TODO: handleDisconnection in the planner mode
class PlannerState(State):
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_planner()

        self.connect_and_store(
            self.gui.gui_manager.send_route_names,
            self.gui.main_view.active_view.load__routes
        )

        self.connect_and_store(
            self.gui.main_view.finish_planning,
            self.finishPlanning
        )

        self.connect_and_store(
            self.gui.main_view.set_position_signal,
            self.gui.main_view.active_view.planning_tools.setPoint
        )

        self.connect_and_store(
            self.gui.gui_manager.update_pose,
            self.gui.main_view.map_view.update_robot_pose
        )
        self.connect_and_store(
            self.gui.main_view.active_view.planning_tools.save_current_routes,
            self.gui.gui_manager.save_routes
        )

        self.gui.gui_manager.send_routes()

    def finishPlanning(self):
        self.gui.main_view.set_position_signal.disconnect(self.gui.main_view.active_view.planning_tools.setPoint)
        self.gui.transition_to(ActiveState())
        self.gui.handleGui()
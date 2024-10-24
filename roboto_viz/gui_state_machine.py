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
        self._state = state
        self._state.gui = self

    def handleGui(self):
        self._state.handleGui()

    def request2(self):
        self._state.handle2()

class State(ABC):
    def __init__(self):
        super().__init__()
    
    @property
    def gui(self) -> Gui:
        return self._gui

    @gui.setter
    def gui(self, gui: Gui) -> None:
        self._gui = gui

    @abstractmethod
    def handleGui(self) -> None:
        pass

class DisconnectedState(State):
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_disconnected()
        self.gui.main_view.connection_signal.connect(self.handleConnection)

        self.gui.gui_manager.service_availability.connect(self.gui.main_view.disconnected_view.set_availability)
        
        self.gui.gui_manager.send_route_names.connect(self.gui.main_view.active_view.active_tools.load_routes)
        self.gui.main_view.active_view.active_tools.save_current_routes.connect(self.gui.gui_manager.save_routes)
        self.gui.gui_manager.send_routes()
        # self.gui.main_view.show()
    
    def handleConnection(self):
        print("handling connection")
        self.gui.gui_manager.trigger_configure()
        self.gui.transition_to(ActiveState())
        self.gui.handleGui()

    def printAvailable(self, available):
        print("Available") if available else print("Not Available")
        # self.service_status_label.setText(f"Service: {status}")
        # self.service_button.setEnabled(available)

class ActiveState(State):
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_active()
        self.gui.gui_manager.update_pose.connect(self.gui.main_view.active_view.map_view.update_robot_pose)
        self.gui.gui_manager.service_availability.connect(self.handleDisconnection)
        self.gui.main_view.set_position_signal.connect(self.gui.gui_manager.handle_goal_pose)

        self.gui.main_view.start_planning.connect(self.startPlanning)

    def handleDisconnection(self, availability: bool):
        if(not availability):
            print("handling disconnection!")
            self.gui.transition_to(DisconnectedState())
            self.gui.handleGui()
        else:
            pass 
    
    def startPlanning(self):
        self.gui.transition_to(PlannerState())
        self.gui.handleGui()

#TODO: handleDisconnection in the planner mode
class PlannerState(State):
    def handleGui(self) -> None:
        self.gui.main_view.switch_to_planner()
        self.gui.main_view.finish_planning.connect(self.finishPlanning)

        self.gui.gui_manager.update_pose.connect(self.gui.main_view.map_view.update_robot_pose)

    def finishPlanning(self):
        self.gui.transition_to(ActiveState())
        self.gui.handleGui()
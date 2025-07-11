from PyQt5.QtWidgets import QWidget, QHBoxLayout
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from typing import Optional

from roboto_viz.map_view import MapView
from roboto_viz.plan_view_tools import PlanTools
from roboto_viz.plan_manager import PlanManager, ExecutionPlan
from roboto_viz.route_manager import RouteManager
from roboto_viz.plan_editor import PlanEditor


class PlanActiveView(QWidget):
    on_disconnection = pyqtSignal(str)
    start_plan_execution = pyqtSignal(str)  # plan_name
    stop_plan_execution = pyqtSignal()
    execute_plan_action = pyqtSignal(str, int)  # plan_name, action_index
    
    # Robot control signals
    dock_robot = pyqtSignal()
    undock_robot = pyqtSignal()
    start_nav = pyqtSignal(str, bool, float, float)  # route_name, to_dest, curr_x, curr_y
    
    # Manual control signals
    start_keys_vel = pyqtSignal(str, float)
    stop_keys_vel = pyqtSignal()
    
    # Map signals
    map_load_requested = pyqtSignal(str)  # map_name

    def __init__(self, map_view: MapView, plan_manager: PlanManager, route_manager: RouteManager):
        super().__init__()
        
        self.map_view = map_view
        self.plan_manager = plan_manager
        self.route_manager = route_manager
        
        # Current robot position
        self.curr_x: float = 0
        self.curr_y: float = 0
        
        # Plan editor window (created on demand)
        self.plan_editor: Optional[PlanEditor] = None
        
        # Create plan tools
        self.plan_tools = PlanTools(self.plan_manager)
        
        self.setup_ui()
        self.setup_connections()
    
    def setup_ui(self):
        # Main horizontal layout
        main_layout = QHBoxLayout(self)
        
        # Add map view (takes 3/4 of space)
        main_layout.addWidget(self.map_view, 3)
        
        # Add plan tools (takes 1/4 of space)
        main_layout.addWidget(self.plan_tools, 1)
    
    def setup_connections(self):
        # Plan control signals
        self.plan_tools.start_plan_execution.connect(self.handle_start_plan_execution)
        self.plan_tools.stop_plan_execution.connect(self.stop_plan_execution.emit)
        self.plan_tools.execute_action.connect(self.handle_execute_action)
        
        # Robot control signals
        self.plan_tools.dock_robot.connect(self.dock_robot.emit)
        self.plan_tools.undock_robot.connect(self.undock_robot.emit)
        
        # Manual control signals
        self.plan_tools.start_keys_vel.connect(self.start_keys_vel.emit)
        self.plan_tools.stop_keys_vel.connect(self.stop_keys_vel.emit)
        
        # Map signals
        self.plan_tools.map_selected.connect(self.handle_map_selected)
        
        # Plan editor signals
        self.plan_tools.open_plan_editor.connect(self.open_plan_editor)
        
        # Disconnection signal
        self.plan_tools.on_disconnect.connect(lambda: self.on_disconnection.emit("Disconnect from PlanActiveView"))
    
    def handle_start_plan_execution(self, plan_name: str):
        """Handle start plan execution request"""
        self.start_plan_execution.emit(plan_name)
    
    def handle_execute_action(self, plan_name: str, action_index: int):
        """Handle execute specific action request"""
        plan = self.plan_manager.get_plan(plan_name)
        if not plan or action_index >= len(plan.actions):
            return
        
        action = plan.actions[action_index]
        
        # Handle different action types
        if action.action_type.value == "route":
            route_name = action.parameters.get('route_name', action.name)
            # Emit navigation signal with current robot position
            self.start_nav.emit(route_name, True, self.curr_x, self.curr_y)
        elif action.action_type.value == "dock":
            self.dock_robot.emit()
        elif action.action_type.value == "undock":
            self.undock_robot.emit()
        elif action.action_type.value == "wait_for_signal":
            # For now, just print - will be implemented later
            print(f"Waiting for signal: {action.parameters.get('signal_name', 'default')}")
        elif action.action_type.value == "stop_and_wait":
            # For now, just print - will be implemented later
            message = action.parameters.get('message', 'Waiting for manual start')
            print(f"Stop and wait: {message}")
        
        # Also emit the general execute action signal
        self.execute_plan_action.emit(plan_name, action_index)
    
    def handle_map_selected(self, map_name: str):
        """Handle map selection request"""
        self.map_load_requested.emit(map_name)
    
    def open_plan_editor(self):
        """Open the plan editor window"""
        if self.plan_editor is None:
            self.plan_editor = PlanEditor(self.plan_manager, self.route_manager)
            self.plan_editor.plan_updated.connect(self.on_plan_updated)
        
        self.plan_editor.show()
        self.plan_editor.raise_()
        self.plan_editor.activateWindow()
    
    @pyqtSlot()
    def on_plan_updated(self):
        """Handle plan update from editor"""
        # Refresh the plan tools to show updated plans
        self.plan_tools.refresh_plans()
    
    @pyqtSlot(float, float, float)
    def update_robot_pose(self, x: float, y: float, theta: float):
        """Update robot position"""
        self.map_view.update_robot_pose(x, y, theta)
        self.curr_x = x
        self.curr_y = y
    
    def switch_to_active(self):
        """Switch to active tab in plan tools"""
        self.plan_tools.switch_to_active()
    
    def switch_to_configure(self):
        """Switch to configure tab in plan tools"""
        self.plan_tools.switch_to_configure()
    
    def on_action_completed(self):
        """Called when a plan action is completed"""
        self.plan_tools.on_action_completed()
    
    def on_plan_execution_stopped(self):
        """Called when plan execution is stopped"""
        self.plan_tools.on_plan_execution_stopped()
    
    def update_robot_status(self, status: str):
        """Update robot status display"""
        self.plan_tools.update_robot_status(status)
    
    def close_plan_editor(self):
        """Close the plan editor if open"""
        if self.plan_editor:
            self.plan_editor.close()
            self.plan_editor = None
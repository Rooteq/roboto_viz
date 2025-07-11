from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QTimer
from typing import Optional
import time

from roboto_viz.plan_manager import PlanManager, ExecutionPlan, ActionType


class PlanExecutor(QObject):
    # Execution signals
    action_started = pyqtSignal(str, int)  # plan_name, action_index
    action_completed = pyqtSignal(str, int)  # plan_name, action_index
    plan_completed = pyqtSignal(str)  # plan_name
    execution_stopped = pyqtSignal(str)  # plan_name
    
    # Robot control signals
    start_nav = pyqtSignal(str, bool, float, float)  # route_name, to_dest, curr_x, curr_y
    dock_robot = pyqtSignal()
    undock_robot = pyqtSignal()
    
    # Status signals
    status_update = pyqtSignal(str)  # status_message

    def __init__(self, plan_manager: PlanManager):
        super().__init__()
        self.plan_manager = plan_manager
        
        # Execution state
        self.is_executing = False
        self.current_plan: Optional[ExecutionPlan] = None
        self.current_action_index = 0
        
        # Action completion tracking
        self.waiting_for_completion = False
        self.current_action_type = None
        
        # Robot position tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
    
    @pyqtSlot(str)
    def start_plan_execution(self, plan_name: str):
        """Start executing a plan"""
        if self.is_executing:
            self.status_update.emit("Already executing a plan. Stop current execution first.")
            return
        
        plan = self.plan_manager.get_plan(plan_name)
        if not plan:
            self.status_update.emit(f"Plan '{plan_name}' not found!")
            return
        
        if len(plan.actions) == 0:
            self.status_update.emit(f"Plan '{plan_name}' has no actions!")
            return
        
        self.current_plan = plan
        self.is_executing = True
        self.current_action_index = plan.current_action_index
        
        self.status_update.emit(f"Starting execution of plan '{plan_name}'")
        self.execute_current_action()
    
    @pyqtSlot()
    def stop_plan_execution(self):
        """Stop current plan execution"""
        if not self.is_executing:
            return
        
        self.is_executing = False
        self.action_timer.stop()
        
        plan_name = self.current_plan.name if self.current_plan else "Unknown"
        self.status_update.emit(f"Stopped execution of plan '{plan_name}'")
        self.execution_stopped.emit(plan_name)
        
        self.current_plan = None
    
    @pyqtSlot(str, int)
    def execute_action(self, plan_name: str, action_index: int):
        """Execute a specific action from a plan"""
        plan = self.plan_manager.get_plan(plan_name)
        if not plan or action_index >= len(plan.actions):
            self.status_update.emit(f"Invalid action index {action_index} for plan '{plan_name}'")
            return
        
        # Set current action and execute
        plan.set_current_action(action_index)
        self.current_plan = plan
        self.current_action_index = action_index
        
        action = plan.actions[action_index]
        self.status_update.emit(f"Executing action: {action.name}")
        self.action_started.emit(plan_name, action_index)
        
        self.perform_action(action)
    
    def execute_current_action(self):
        """Execute the current action in the current plan"""
        if not self.current_plan or not self.is_executing:
            return
        
        if self.current_action_index >= len(self.current_plan.actions):
            # Plan completed, loop back to start
            self.current_action_index = 0
            self.current_plan.set_current_action(0)
        
        action = self.current_plan.actions[self.current_action_index]
        self.status_update.emit(f"Executing action {self.current_action_index + 1}: {action.name}")
        self.action_started.emit(self.current_plan.name, self.current_action_index)
        
        self.perform_action(action)
    
    def perform_action(self, action):
        """Perform the specified action"""
        action_type = action.action_type
        
        if action_type == ActionType.ROUTE:
            self.execute_route_action(action)
        elif action_type == ActionType.DOCK:
            self.execute_dock_action(action)
        elif action_type == ActionType.UNDOCK:
            self.execute_undock_action(action)
        elif action_type == ActionType.WAIT_FOR_SIGNAL:
            self.execute_wait_signal_action(action)
        elif action_type == ActionType.STOP_AND_WAIT:
            self.execute_stop_wait_action(action)
        else:
            self.status_update.emit(f"Unknown action type: {action_type}")
            self.on_action_completed()
    
    def execute_route_action(self, action):
        """Execute a route navigation action"""
        route_name = action.parameters.get('route_name', action.name)
        self.status_update.emit(f"Navigating to route: {route_name}")
        
        # Set waiting state
        self.waiting_for_completion = True
        self.current_action_type = ActionType.ROUTE
        
        # Emit navigation signal
        self.start_nav.emit(route_name, True, self.robot_x, self.robot_y)
    
    def execute_dock_action(self, action):
        """Execute a dock action"""
        self.status_update.emit("Docking robot...")
        
        # Set waiting state
        self.waiting_for_completion = True
        self.current_action_type = ActionType.DOCK
        
        self.dock_robot.emit()
    
    def execute_undock_action(self, action):
        """Execute an undock action"""
        self.status_update.emit("Undocking robot...")
        
        # Set waiting state
        self.waiting_for_completion = True
        self.current_action_type = ActionType.UNDOCK
        
        self.undock_robot.emit()
    
    def execute_wait_signal_action(self, action):
        """Execute a wait for signal action"""
        signal_name = action.parameters.get('signal_name', 'default')
        self.status_update.emit(f"Waiting for signal: {signal_name}")
        
        # For now, just complete immediately
        # In a real implementation, this would wait for an external signal
        QTimer.singleShot(1000, self.on_action_completed)  # 1 second
    
    def execute_stop_wait_action(self, action):
        """Execute a stop and wait action"""
        message = action.parameters.get('message', 'Waiting for manual start')
        self.status_update.emit(f"Stop and wait: {message}")
        
        # This action waits indefinitely until manually continued
        # For simulation, we'll wait 2 seconds
        QTimer.singleShot(2000, self.on_action_completed)  # 2 seconds
    
    def on_action_completed(self):
        """Called when current action is completed"""
        if not self.current_plan:
            return
        
        # Clear waiting state
        self.waiting_for_completion = False
        self.current_action_type = None
        
        plan_name = self.current_plan.name
        action_index = self.current_action_index
        
        self.action_completed.emit(plan_name, action_index)
        
        if self.is_executing:
            # Move to next action
            self.current_action_index += 1
            self.current_plan.set_current_action(self.current_action_index)
            
            if self.current_action_index >= len(self.current_plan.actions):
                # Plan completed, start over for continuous execution
                self.status_update.emit(f"Plan '{plan_name}' completed. Restarting...")
                self.current_action_index = 0
                self.current_plan.set_current_action(0)
                self.plan_completed.emit(plan_name)
            
            # Execute next action after a short delay
            QTimer.singleShot(1000, self.execute_current_action)
    
    @pyqtSlot(float, float, float)
    def update_robot_pose(self, x: float, y: float, theta: float):
        """Update robot pose tracking"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
    
    @pyqtSlot()
    def on_navigation_completed(self):
        """Called when navigation action is completed"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.ROUTE):
            self.on_action_completed()
    
    @pyqtSlot()
    def on_docking_completed(self):
        """Called when docking action is completed"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.DOCK):
            self.on_action_completed()
    
    @pyqtSlot()
    def on_undocking_completed(self):
        """Called when undocking action is completed"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.UNDOCK):
            self.on_action_completed()
    
    def get_current_status(self) -> str:
        """Get current execution status"""
        if not self.is_executing:
            return "Idle"
        
        if self.current_plan:
            action = self.current_plan.get_current_action()
            if action:
                return f"Executing: {action.name}"
        
        return "Executing..."
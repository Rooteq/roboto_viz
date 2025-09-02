from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QTimer
from typing import Optional
import time

from roboto_viz.plan_manager import PlanManager, ExecutionPlan, ActionType


class CANSignalForwarder(QObject):
    """Forwards CAN signals from GUI manager to plan executor (replaces UART)"""
    can_signal_received = pyqtSignal()  # CAN signal received from ID 0x69
    
    def __init__(self):
        super().__init__()
        
    def forward_signal(self):
        """Forward CAN signal (called by GUI manager)"""
        print("CAN Signal: Forwarding signal to plan executor")
        self.can_signal_received.emit()


class PlanExecutor(QObject):
    # Execution signals
    action_started = pyqtSignal(str, int)  # plan_name, action_index
    action_completed = pyqtSignal(str, int)  # plan_name, action_index
    plan_completed = pyqtSignal(str)  # plan_name
    execution_stopped = pyqtSignal(str)  # plan_name
    execution_stopped_due_to_failure = pyqtSignal(str, str)  # plan_name, failure_reason
    
    # Robot control signals
    start_nav = pyqtSignal(str, bool, float, float)  # route_name, to_dest, curr_x, curr_y
    dock_robot = pyqtSignal()
    dock_robot_at = pyqtSignal(str)  # dock_name
    undock_robot = pyqtSignal()
    
    # Status signals
    status_update = pyqtSignal(str)  # status_message
    waiting_for_signal = pyqtSignal(str)  # signal_name - emitted when waiting for external signal
    signal_received = pyqtSignal()  # emitted when signal button is pressed
    uart_signal_received = pyqtSignal()  # emitted when UART signal is received to hide button
    wait_status_update = pyqtSignal(str)  # Wait action status for CAN messages
    single_action_completed = pyqtSignal(str, int)  # plan_name, action_index - for single action execution

    def __init__(self, plan_manager: PlanManager):
        super().__init__()
        self.plan_manager = plan_manager
        
        # Execution state
        self.is_executing = False
        self.continuous_execution = False  # True for plan execution, False for single action
        self.current_plan: Optional[ExecutionPlan] = None
        self.current_action_index = 0
        
        # Action completion tracking
        self.waiting_for_completion = False
        self.current_action_type = None
        self.waiting_for_signal_name = None
        
        # Robot position tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # CAN signal support (replaces UART)
        self.can_signal_forwarder = CANSignalForwarder()
        self.can_signal_forwarder.can_signal_received.connect(self.on_can_signal_received)
        
        # CAN wait parameters (simplified from UART)
        self.waiting_for_can_signal = False  # Wait for CAN signal on ID 0x69
    
    def __del__(self):
        """Destructor to clean up CAN signal forwarder"""
        pass  # No cleanup needed for CAN signal forwarder
    
    @pyqtSlot(str)
    def start_plan_execution(self, plan_name: str):
        """Start executing a plan"""
        if self.is_executing:
            self.status_update.emit("Plan już w toku. Najpierw zatrzymaj bieżące wykonanie.")
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
        self.continuous_execution = True  # This is continuous plan execution
        # Start from where the plan left off (current_action_index)
        self.current_action_index = plan.current_action_index
        
        self.status_update.emit(f"Starting execution of plan '{plan_name}' from action {self.current_action_index + 1}")
        self.execute_current_action()
    
    @pyqtSlot()
    def stop_plan_execution(self):
        """Stop current plan execution"""
        if not self.is_executing:
            return
        
        print("Plan Executor: stop_plan_execution() called - manual stop")
        self.is_executing = False
        self.continuous_execution = False
        
        plan_name = self.current_plan.name if self.current_plan else "Unknown"
        self.status_update.emit(f"Zatrzymano wykonywanie planu '{plan_name}'")
        self.execution_stopped.emit(plan_name)
        
        # Clear waiting states
        self.waiting_for_completion = False
        self.current_action_type = None
        self.waiting_for_signal_name = None
        self.waiting_for_can_signal = False
    
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
        
        # Set execution flag to true, but this is single action execution
        self.is_executing = True
        self.continuous_execution = False  # This is single action execution
        
        action = plan.actions[action_index]
        self.status_update.emit(f"Wykonywanie akcji: {action.name}")
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
        self.status_update.emit(f"Wykonywanie akcji {self.current_action_index + 1}: {action.name}")
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
        else:
            self.status_update.emit(f"Unknown action type: {action_type}")
            self.on_action_completed()
    
    def execute_route_action(self, action):
        """Execute a route navigation action"""
        route_name = action.parameters.get('route_name', action.name)
        reverse = action.parameters.get('reverse', False)
        to_dest = not reverse  # If reverse is True, to_dest should be False
        
        direction_text = "in reverse" if reverse else "forward"
        self.status_update.emit(f"Navigating route {route_name} {direction_text}")
        
        # Set waiting state
        self.waiting_for_completion = True
        self.current_action_type = ActionType.ROUTE
        
        # Emit navigation signal with proper direction
        self.start_nav.emit(route_name, to_dest, self.robot_x, self.robot_y)
    
    def execute_dock_action(self, action):
        """Execute a dock action"""
        dock_name = action.parameters.get('dock_name')
        
        if dock_name:
            self.status_update.emit(f"Docking robot at {dock_name}...")
        else:
            self.status_update.emit("Docking robot...")
        
        # Set waiting state
        self.waiting_for_completion = True
        self.current_action_type = ActionType.DOCK
        
        # Emit dock signal with dock name if available
        if dock_name:
            self.dock_robot_at.emit(dock_name)
        else:
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
        
        self.status_update.emit(f"Oczekiwanie na sygnał CAN: {signal_name}")
        
        # Set waiting state and emit signal for UI to show signal button
        self.waiting_for_completion = True
        self.current_action_type = ActionType.WAIT_FOR_SIGNAL
        self.waiting_for_signal_name = signal_name
        
        # Set CAN wait parameters (simplified - just wait for any CAN signal on ID 0x69)
        self.waiting_for_can_signal = True
        
        # Emit wait status for CAN WARNING message
        self.wait_status_update.emit(f"Oczekiwanie na sygnał CAN: {signal_name}")
        
        self.waiting_for_signal.emit(signal_name)
    
    
    def on_action_completed(self):
        """Called when current action is completed"""
        if not self.current_plan:
            return
        
        # Clear waiting state
        self.waiting_for_completion = False
        self.current_action_type = None
        self.waiting_for_signal_name = None
        self.waiting_for_can_signal = False
        
        plan_name = self.current_plan.name
        action_index = self.current_action_index
        
        self.action_completed.emit(plan_name, action_index)
        
        if self.is_executing:
            if self.continuous_execution:
                # Continuous plan execution - move to next action
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
            else:
                # Single action execution - automatically continue with remaining actions
                self.current_action_index += 1
                self.current_plan.set_current_action(self.current_action_index)
                
                if self.current_action_index >= len(self.current_plan.actions):
                    # All actions completed
                    self.status_update.emit(f"All actions in plan '{plan_name}' completed")
                    self.is_executing = False
                    self.continuous_execution = False
                    self.single_action_completed.emit(plan_name, action_index)
                else:
                    # Continue with next action automatically
                    self.status_update.emit(f"Action '{self.current_plan.actions[action_index].name}' completed, continuing with next action...")
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
    
    @pyqtSlot()
    def on_signal_received(self):
        """Called when signal button is pressed"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.WAIT_FOR_SIGNAL):
            self.status_update.emit(f"Signal '{self.waiting_for_signal_name}' received (button)")
            
            # Emit wait completion status for CAN message
            self.wait_status_update.emit(f"Signal '{self.waiting_for_signal_name}' received (button)")
            
            self.on_action_completed()
    
    @pyqtSlot()
    def on_can_signal_received(self):
        """Called when a CAN signal is received on ID 0x69"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.WAIT_FOR_SIGNAL and
            self.waiting_for_can_signal):
            
            print(f"CAN Signal: Received signal on ID 0x69 for '{self.waiting_for_signal_name}'")
            self.status_update.emit(f"Signal '{self.waiting_for_signal_name}' received (CAN)")
            
            # Emit wait completion status for CAN message
            self.wait_status_update.emit(f"Signal '{self.waiting_for_signal_name}' received (CAN)")
            
            self.waiting_for_can_signal = False
            self.uart_signal_received.emit()  # Hide signal button (keeping same signal name for UI compatibility)
            self.on_action_completed()
        else:
            print(f"CAN Signal: Received signal on ID 0x69 but not waiting for signal - ignoring")
    
    @pyqtSlot(str)
    def on_navigation_failed(self, status: str):
        """Called when navigation fails"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.ROUTE and 
            ("Failed" in status or "Error" in status)):
            print(f"Plan Executor: Navigation failed with status: {status}")
            
            # Don't emit any status update from plan executor - let the navigation status flow through
            # The navigation status will be displayed directly via the navStatus connection
            
            # Stop execution state but preserve failure status
            self.is_executing = False
            self.continuous_execution = False
            self.waiting_for_completion = False
            self.current_action_type = None
            self.waiting_for_signal_name = None
            self.waiting_for_can_signal = False
            
            plan_name = self.current_plan.name if self.current_plan else "Unknown"
            self.execution_stopped_due_to_failure.emit(plan_name, status)
    
    @pyqtSlot(str)
    def on_docking_status_changed(self, status: str):
        """Called when docking status changes"""
        if not (self.is_executing and self.waiting_for_completion):
            return
            
        if self.current_action_type == ActionType.DOCK:
            if status == "Docked":
                self.status_update.emit("Docking completed successfully")
                self.on_docking_completed()
            elif status == "Dock Cancelled":
                self.status_update.emit("Docking cancelled - stopping plan execution")
                self.stop_plan_execution()
            elif status in ["Dock Failed", "Dock Error"]:
                self.status_update.emit(f"Docking failed: {status} - stopping plan execution")
                self.stop_plan_execution()
        elif self.current_action_type == ActionType.UNDOCK:
            if status == "Undocked":
                self.status_update.emit("Undocking completed successfully")
                self.on_undocking_completed()
            elif status in ["Undock Failed", "Undock Error"]:
                self.status_update.emit(f"Undocking failed: {status} - stopping plan execution")
                self.stop_plan_execution()
    
    def get_current_status(self) -> str:
        """Get current execution status"""
        if not self.is_executing:
            return "Bezczynny"
        
        if self.current_plan:
            action = self.current_plan.get_current_action()
            if action:
                return f"Wykonywanie: {action.name}"
        
        return "Wykonywanie..."
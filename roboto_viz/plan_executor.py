from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QTimer, QThread
from typing import Optional
import time
import serial
import threading

from roboto_viz.plan_manager import PlanManager, ExecutionPlan, ActionType


class UARTListener(QThread):
    """Thread for listening to UART messages"""
    uart_message_received = pyqtSignal(str)  # UART message ('0' or '1')
    
    def __init__(self, port='/dev/ttyUSB1', baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial_connection = None
        
    def run(self):
        """Main thread loop for UART listening"""
        try:
            # Create UART connection
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0  # 1 second timeout for checking running flag
            )
            self.running = True
            
            while self.running:
                try:
                    if self.serial_connection.in_waiting > 0:
                        # Read line and decode
                        line = self.serial_connection.readline().decode('utf-8').strip()
                        print(f"UART RAW: Received '{line}' from {self.port}")
                        if line in ['0', '1']:
                            print(f"UART: Valid message '{line}' - emitting signal")
                            self.uart_message_received.emit(line)
                        else:
                            print(f"UART: Invalid message '{line}' - ignoring")
                except serial.SerialTimeoutException:
                    continue  # Check running flag
                except Exception as e:
                    print(f"UART receive error: {e}")
                    break
                    
        except Exception as e:
            print(f"UART connection error: {e}")
        finally:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()
                
    def stop(self):
        """Stop the UART listener"""
        self.running = False
        if self.isRunning():
            self.wait(3000)  # Wait up to 3 seconds for thread to finish


class PlanExecutor(QObject):
    # Execution signals
    action_started = pyqtSignal(str, int)  # plan_name, action_index
    action_completed = pyqtSignal(str, int)  # plan_name, action_index
    plan_completed = pyqtSignal(str)  # plan_name
    execution_stopped = pyqtSignal(str)  # plan_name
    
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
        
        # UART support
        self.uart_listener = UARTListener()
        self.uart_listener.uart_message_received.connect(self.on_uart_message_received)
        self.uart_listener.start()
        
        # UART wait parameters
        self.waiting_for_uart_high = False  # Wait for "1" message
        self.waiting_for_uart_low = False   # Wait for "0" message
    
    def __del__(self):
        """Destructor to clean up UART listener"""
        if hasattr(self, 'uart_listener'):
            self.uart_listener.stop()
    
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
        
        self.is_executing = False
        self.continuous_execution = False
        
        plan_name = self.current_plan.name if self.current_plan else "Unknown"
        self.status_update.emit(f"Stopped execution of plan '{plan_name}'")
        self.execution_stopped.emit(plan_name)
        
        # Clear waiting states
        self.waiting_for_completion = False
        self.current_action_type = None
        self.waiting_for_signal_name = None
        self.waiting_for_uart_high = False
        self.waiting_for_uart_low = False
    
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
        
        # Extract UART parameters
        activate_on_high = action.parameters.get('activate_on_high', False)  # Wait for "1"
        activate_on_low = action.parameters.get('activate_on_low', False)    # Wait for "0"
        
        self.status_update.emit(f"Waiting for signal: {signal_name}")
        
        # Set waiting state and emit signal for UI to show signal button
        self.waiting_for_completion = True
        self.current_action_type = ActionType.WAIT_FOR_SIGNAL
        self.waiting_for_signal_name = signal_name
        
        # Set UART wait parameters
        self.waiting_for_uart_high = activate_on_high
        self.waiting_for_uart_low = activate_on_low
        
        self.waiting_for_signal.emit(signal_name)
    
    
    def on_action_completed(self):
        """Called when current action is completed"""
        if not self.current_plan:
            return
        
        # Clear waiting state
        self.waiting_for_completion = False
        self.current_action_type = None
        self.waiting_for_signal_name = None
        self.waiting_for_uart_high = False
        self.waiting_for_uart_low = False
        
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
                # Single action execution - stop after completion
                self.status_update.emit(f"Action '{self.current_plan.actions[action_index].name}' completed")
                self.is_executing = False
                self.continuous_execution = False
                self.single_action_completed.emit(plan_name, action_index)
    
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
            self.on_action_completed()
    
    @pyqtSlot(str)
    def on_uart_message_received(self, message: str):
        """Called when a UART message is received"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.WAIT_FOR_SIGNAL):
            
            # Debug: Print all UART messages received while waiting for signal
            print(f"UART DEBUG: Received message '{message}' while waiting for signal '{self.waiting_for_signal_name}'")
            print(f"UART DEBUG: Waiting for high={self.waiting_for_uart_high}, waiting for low={self.waiting_for_uart_low}")
            
            # Check if this UART message matches our waiting criteria
            signal_triggered = False
            trigger_type = ""
            
            if self.waiting_for_uart_high and message == "1":
                signal_triggered = True
                trigger_type = "high (1)"
                print(f"UART DEBUG: Signal triggered by high condition")
            elif self.waiting_for_uart_low and message == "0":
                signal_triggered = True  
                trigger_type = "low (0)"
                print(f"UART DEBUG: Signal triggered by low condition")
            else:
                print(f"UART DEBUG: Message '{message}' did not match waiting criteria")
            
            if signal_triggered:
                self.status_update.emit(f"Signal '{self.waiting_for_signal_name}' received (UART {trigger_type})")
                self.uart_signal_received.emit()  # Immediately hide signal button
                self.on_action_completed()
        else:
            # Debug: Print UART messages received when not waiting for signal
            print(f"UART DEBUG: Received message '{message}' (not waiting for signal)")
    
    @pyqtSlot(str)
    def on_navigation_failed(self, status: str):
        """Called when navigation fails"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.ROUTE and status == "Failed"):
            self.status_update.emit("Navigation failed - stopping plan execution")
            self.stop_plan_execution()
    
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
            return "Idle"
        
        if self.current_plan:
            action = self.current_plan.get_current_action()
            if action:
                return f"Executing: {action.name}"
        
        return "Executing..."
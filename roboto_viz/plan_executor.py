from typing import Optional

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QTimer

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
        
        # Navigation preparation state
        self._pending_navigation = None
        self._preparation_timer = None
        self._preparation_countdown = 0
        
        # Signal preparation state
        self._pending_signal_type = None
        self._signal_timer = None
        self._signal_countdown = 0
        
        # Plan preparation state
        self._pending_plan_name = None
        self._plan_timer = None
        self._plan_countdown = 0
        
        # Signals for buzzer and warning during preparation
        self.buzzer_on = pyqtSignal()
        self.buzzer_off = pyqtSignal() 
        self.warning_on = pyqtSignal()
        self.warning_off = pyqtSignal()
    
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
        
        # Start 5-second preparation phase before plan execution
        self.status_update.emit(f"OBSTACLE DETECTED - Przygotowanie do wykonania planu '{plan_name}' (5 sek)")
        
        # Start preparation countdown
        self._start_plan_preparation(plan_name)
        
        # Execute the plan after 5 seconds
        QTimer.singleShot(5000, self._execute_plan_after_delay)
    
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
        
        # Start 5-second preparation phase with buzzer and warning
        self.status_update.emit(f"Przygotowanie do nawigacji: {route_name} {direction_text} (5 sek)")
        
        # Set waiting state for preparation
        self.waiting_for_completion = True
        self.current_action_type = ActionType.ROUTE
        
        # Store the navigation parameters for later execution
        self._pending_navigation = {
            'route_name': route_name,
            'to_dest': to_dest,
            'robot_x': self.robot_x,
            'robot_y': self.robot_y
        }
        
        # Emit buzzer and warning signals for 5 seconds
        self._start_navigation_preparation()
        
        # Schedule actual navigation start after 5 seconds
        QTimer.singleShot(5000, self._execute_navigation_after_delay)
    
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
                    # All actions completed, loop back to start for continuous execution
                    self.status_update.emit(f"Plan '{plan_name}' completed. Restarting...")
                    self.current_action_index = 0
                    self.current_plan.set_current_action(0)
                    self.plan_completed.emit(plan_name)
                    
                # Continue with next action automatically (or restart from beginning)
                action_name = self.current_plan.actions[action_index].name if action_index < len(self.current_plan.actions) else "Plan restarting"
                self.status_update.emit(f"Action '{action_name}' completed, continuing...")
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
            
            # Start 5-second preparation phase after receiving signal
            self.status_update.emit(f"OBSTACLE DETECTED - Signal '{self.waiting_for_signal_name}' received, przygotowanie (5 sek)")
            
            # Emit wait completion status for CAN message
            self.wait_status_update.emit(f"Signal '{self.waiting_for_signal_name}' received (button)")
            
            # Start preparation phase before completing the action
            self._start_signal_preparation('button')
            
            # Complete the action after 5 seconds
            QTimer.singleShot(5000, self._complete_signal_action)
            
        elif (self.is_executing and self.waiting_for_completion and 
              self.current_action_type == ActionType.ROUTE):
            # Navigation cancellation - treat as if navigation completed and proceed to next action
            self.status_update.emit("Navigation cancelled - proceeding to next action")
            self.on_action_completed()
    
    @pyqtSlot()
    def on_can_signal_received(self):
        """Called when a CAN signal is received on ID 0x69"""
        if (self.is_executing and self.waiting_for_completion and 
            self.current_action_type == ActionType.WAIT_FOR_SIGNAL and
            self.waiting_for_can_signal):
            
            print(f"CAN Signal: Received signal on ID 0x69 for '{self.waiting_for_signal_name}'")
            
            # Start 5-second preparation phase after receiving CAN signal
            self.status_update.emit(f"OBSTACLE DETECTED - Signal '{self.waiting_for_signal_name}' received (CAN), przygotowanie (5 sek)")
            
            # Emit wait completion status for CAN message
            self.wait_status_update.emit(f"Signal '{self.waiting_for_signal_name}' received (CAN)")
            
            self.waiting_for_can_signal = False
            self.uart_signal_received.emit()  # Hide signal button (keeping same signal name for UI compatibility)
            
            # Start preparation phase before completing the action
            self._start_signal_preparation('CAN')
            
            # Complete the action after 5 seconds
            QTimer.singleShot(5000, self._complete_signal_action)
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
    
    def _start_navigation_preparation(self):
        """Start 5-second preparation phase with buzzer and warning signals"""
        # Emit obstacle detected status (same as detecting an obstacle)
        self.status_update.emit("OBSTACLE DETECTED - Przygotowanie do nawigacji")
        
        # The buzzer and warning signals will be handled by the CAN status manager
        # through the collision detection system in GUI manager
        # We emit this status to trigger the same behavior as obstacle detection
        
        # Start a timer to update the countdown
        self._preparation_countdown = 5
        self._preparation_timer = QTimer()
        self._preparation_timer.timeout.connect(self._update_preparation_countdown)
        self._preparation_timer.start(1000)  # Update every second
    
    def _update_preparation_countdown(self):
        """Update the preparation countdown display"""
        if self._preparation_countdown > 1:
            self._preparation_countdown -= 1
            nav_info = self._pending_navigation
            if nav_info:
                direction_text = "in reverse" if not nav_info['to_dest'] else "forward"
                self.status_update.emit(f"OBSTACLE DETECTED - Start nawigacji za {self._preparation_countdown} sek: {nav_info['route_name']} {direction_text}")
        else:
            # Stop the countdown timer
            if self._preparation_timer:
                self._preparation_timer.stop()
                self._preparation_timer = None
    
    def _execute_navigation_after_delay(self):
        """Execute the navigation after 5-second delay"""
        if self._pending_navigation and self.is_executing and self.waiting_for_completion:
            nav_info = self._pending_navigation
            
            # Send OK signal to stop buzzer and warning
            self.status_update.emit("OK - Starting navigation")
            
            # Clear obstacle detected status and send OK
            # This will be handled by CAN status manager to stop buzzer and send OK signal
            
            # Clear pending navigation
            self._pending_navigation = None
            
            # Now emit the actual navigation signal
            self.start_nav.emit(nav_info['route_name'], nav_info['to_dest'], nav_info['robot_x'], nav_info['robot_y'])
            
            # Update status to show actual navigation
            direction_text = "in reverse" if not nav_info['to_dest'] else "forward"
            self.status_update.emit(f"Navigating route {nav_info['route_name']} {direction_text}")
    
    def _start_signal_preparation(self, signal_type: str):
        """Start 5-second preparation phase after signal reception"""
        # Store signal info for countdown
        self._pending_signal_type = signal_type
        
        # Start a timer to update the countdown
        self._signal_countdown = 5
        self._signal_timer = QTimer()
        self._signal_timer.timeout.connect(self._update_signal_countdown)
        self._signal_timer.start(1000)  # Update every second
    
    def _update_signal_countdown(self):
        """Update the signal preparation countdown display"""
        if self._signal_countdown > 1:
            self._signal_countdown -= 1
            self.status_update.emit(f"OBSTACLE DETECTED - Signal '{self.waiting_for_signal_name}' received ({self._pending_signal_type}), start za {self._signal_countdown} sek")
        else:
            # Stop the countdown timer
            if self._signal_timer:
                self._signal_timer.stop()
                self._signal_timer = None
    
    def _complete_signal_action(self):
        """Complete the signal action after 5-second delay"""
        if self.is_executing and self.waiting_for_completion:
            # Send OK signal to stop buzzer and warning
            self.status_update.emit(f"OK - Signal '{self.waiting_for_signal_name}' processed, continuing plan")
            
            # Clear pending signal info
            self._pending_signal_type = None
            
            # Complete the action
            self.on_action_completed()
    
    def _start_plan_preparation(self, plan_name: str):
        """Start 5-second preparation phase before plan execution"""
        # Store plan info for countdown
        self._pending_plan_name = plan_name
        
        # Start a timer to update the countdown
        self._plan_countdown = 5
        self._plan_timer = QTimer()
        self._plan_timer.timeout.connect(self._update_plan_countdown)
        self._plan_timer.start(1000)  # Update every second
    
    def _update_plan_countdown(self):
        """Update the plan preparation countdown display"""
        if self._plan_countdown > 1:
            self._plan_countdown -= 1
            action_index = self.current_action_index + 1 if self.current_plan else 1
            self.status_update.emit(f"OBSTACLE DETECTED - Start planu '{self._pending_plan_name}' za {self._plan_countdown} sek (od akcji {action_index})")
        else:
            # Stop the countdown timer
            if self._plan_timer:
                self._plan_timer.stop()
                self._plan_timer = None
    
    def _execute_plan_after_delay(self):
        """Execute the plan after 5-second delay"""
        if self.is_executing and self.current_plan:
            # Send OK signal to stop buzzer and warning
            action_index = self.current_action_index + 1
            self.status_update.emit(f"OK - Starting execution of plan '{self._pending_plan_name}' from action {action_index}")
            
            # Clear pending plan info
            self._pending_plan_name = None
            
            # Now execute the first action
            self.execute_current_action()
    
    @pyqtSlot()
    def skip_to_wait_signal_action(self):
        """Skip current action and jump to next wait_for_signal action"""
        if not self.is_executing or not self.current_plan:
            return
            
        # Find the next wait_for_signal action
        current_index = self.current_plan.current_action_index
        wait_action_index = None
        
        # Look for wait_for_signal actions starting from current position + 1
        for i in range(len(self.current_plan.actions)):
            check_index = (current_index + 1 + i) % len(self.current_plan.actions)
            action = self.current_plan.actions[check_index]
            if hasattr(action, 'action_type') and action.action_type.value == "wait_for_signal":
                wait_action_index = check_index
                break
        
        if wait_action_index is not None:
            # Cancel current action
            self.waiting_for_completion = False
            self.current_action_type = None
            
            # Jump to the wait_for_signal action
            self.current_plan.set_current_action(wait_action_index)
            self.current_action_index = wait_action_index
            
            # Execute the wait_for_signal action directly
            wait_action = self.current_plan.actions[wait_action_index]
            self.status_update.emit(f"Skipped to wait_for_signal action: {wait_action.name}")
            self.perform_action(wait_action)

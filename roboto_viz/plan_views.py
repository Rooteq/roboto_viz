from PyQt5.QtWidgets import QWidget, QHBoxLayout
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt
from typing import Optional

from roboto_viz.map_view import MapView
from roboto_viz.plan_view_tools import PlanTools
from roboto_viz.plan_manager import PlanManager, ExecutionPlan
from roboto_viz.route_manager import RouteManager
from roboto_viz.dock_manager import DockManager
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
    
    # Navigation control
    stop_navigation = pyqtSignal()
    
    # Signal handling
    signal_button_pressed = pyqtSignal()
    skip_to_wait_signal = pyqtSignal()  # Skip directly to wait_for_signal action
    
    # Map signals
    map_load_requested = pyqtSignal(str)  # map_name

    def __init__(self, map_view: MapView, plan_manager: PlanManager, route_manager: RouteManager, dock_manager: DockManager):
        super().__init__()
        
        self.map_view = map_view
        self.plan_manager = plan_manager
        self.route_manager = route_manager
        self.dock_manager = dock_manager
        
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
        main_layout.setContentsMargins(2, 2, 2, 2)  # Reduced margins
        main_layout.setSpacing(2)  # Reduced spacing
        
        # Create stacked widget for left side (grid or map)
        from PyQt5.QtWidgets import QStackedWidget, QGridLayout, QVBoxLayout, QLabel
        self.left_stacked_widget = QStackedWidget()
        
        # Create grid widget with status cells (optimized for 1920x1080)
        self.grid_widget = QWidget()
        self.grid_widget.setStyleSheet("QWidget { background-color: #f8f9fa; }")
        self.grid_widget.setMinimumSize(600, 400)  # Much larger for big screens
        grid_layout = QGridLayout(self.grid_widget)
        grid_layout.setContentsMargins(20, 20, 20, 20)  # Larger margins
        grid_layout.setSpacing(20)  # More spacing between cells
        
        # Create status cells in a grid format
        # Robot Status Cell (full width top row, background color changes) - LARGE for 1920x1080
        self.robot_status_frame = QWidget()
        self.robot_status_frame.setMinimumSize(600, 200)  # Even larger to prevent clipping
        self.robot_status_frame.setStyleSheet("""
            QWidget {
                border: 4px solid #bdc3c7;
                border-radius: 12px;
                background-color: #f8f9fa;
                padding: 20px;
            }
        """)
        robot_layout = QVBoxLayout(self.robot_status_frame)
        robot_layout.setContentsMargins(20, 15, 20, 15)
        robot_layout.setSpacing(12)
        
        robot_title = QLabel("STATUS ROBOTA")
        robot_title.setAlignment(Qt.AlignCenter)
        robot_title.setStyleSheet("""
            QLabel {
                font-size: 48px;
                font-weight: bold;
                color: #2c3e50;
                background: none;
                border: none;
            }
        """)
        robot_layout.addWidget(robot_title)
        
        # Create a container for the status display - VERY LARGE AND VISIBLE
        self.robot_status_text = QLabel("Bezczynny")
        self.robot_status_text.setAlignment(Qt.AlignCenter)
        self.robot_status_text.setStyleSheet("""
            QLabel {
                font-size: 42px;
                color: #2c3e50;
                background: none;
                border: none;
                font-weight: bold;
            }
        """)
        robot_layout.addWidget(self.robot_status_text)

        # Plan Status Cell (bottom left, wider) - LARGE for 1920x1080
        plan_status_frame = QWidget()
        plan_status_frame.setMinimumSize(400, 180)  # Even larger to prevent clipping
        plan_status_frame.setStyleSheet("""
            QWidget {
                border: 4px solid #bdc3c7;
                border-radius: 12px;
                background-color: white;
                padding: 20px;
            }
        """)
        plan_layout = QVBoxLayout(plan_status_frame)
        plan_layout.setContentsMargins(20, 15, 20, 15)
        plan_layout.setSpacing(12)
        
        plan_title = QLabel("STATUS PLANU")
        plan_title.setAlignment(Qt.AlignCenter)
        plan_title.setStyleSheet("""
            QLabel {
                font-size: 40px;
                font-weight: bold;
                color: #2c3e50;
                background: none;
                border: none;
            }
        """)
        plan_layout.addWidget(plan_title)
        
        self.plan_status_display = QLabel("Brak aktywnego planu")
        self.plan_status_display.setAlignment(Qt.AlignCenter)
        self.plan_status_display.setStyleSheet("""
            QLabel {
                font-size: 30px;
                color: #7f8c8d;
                background: none;
                border: none;
                word-wrap: true;
                font-weight: bold;
            }
        """)
        plan_layout.addWidget(self.plan_status_display)

        # Battery Status Cell (bottom right, narrower) - LARGE for 1920x1080
        self.battery_status_frame = QWidget()
        self.battery_status_frame.setMinimumSize(240, 180)  # Even larger to prevent clipping
        self.battery_status_frame.setStyleSheet("""
            QWidget {
                border: 4px solid #bdc3c7;
                border-radius: 12px;
                background-color: white;
                padding: 20px;
            }
        """)
        battery_layout = QVBoxLayout(self.battery_status_frame)
        battery_layout.setContentsMargins(20, 15, 20, 15)
        battery_layout.setSpacing(12)
        
        battery_title = QLabel("BATERIA")
        battery_title.setAlignment(Qt.AlignCenter)
        battery_title.setStyleSheet("""
            QLabel {
                font-size: 40px;
                font-weight: bold;
                color: #2c3e50;
                background: none;
                border: none;
            }
        """)
        battery_layout.addWidget(battery_title)
        
        self.battery_status_display = QLabel("Nieznany")
        self.battery_status_display.setAlignment(Qt.AlignCenter)
        self.battery_status_display.setStyleSheet("""
            QLabel {
                font-size: 40px;
                color: #7f8c8d;
                background: none;
                border: none;
                font-weight: bold;
            }
        """)
        battery_layout.addWidget(self.battery_status_display)
        
        # Add cells to grid with new layout:
        # Row 0: Robot status (full width)
        # Row 1: Plan status (left, wider) and Battery status (right, narrower)
        grid_layout.addWidget(self.robot_status_frame, 0, 0, 1, 3)  # Span 3 columns
        grid_layout.addWidget(plan_status_frame, 1, 0, 1, 2)  # Span 2 columns (wider)
        grid_layout.addWidget(self.battery_status_frame, 1, 2, 1, 1)  # 1 column (narrower)
        
        # Add both grid and map to stacked widget
        self.left_stacked_widget.addWidget(self.grid_widget)  # Index 0
        self.left_stacked_widget.addWidget(self.map_view)     # Index 1
        
        # Start with grid view
        self.left_stacked_widget.setCurrentIndex(0)
        print(f"DEBUG: PlanActiveView - Grid widget created")
        print(f"DEBUG: PlanActiveView - Left stacked widget has {self.left_stacked_widget.count()} widgets")
        print(f"DEBUG: PlanActiveView - Current index: {self.left_stacked_widget.currentIndex()}")
        self.grid_widget.show()
        
        # Add stacked widget (takes 3/4 of space) and plan tools (takes 1/4 of space)
        main_layout.addWidget(self.left_stacked_widget, 3)
        main_layout.addWidget(self.plan_tools, 1)
    
    def setup_connections(self):
        # Plan control signals
        self.plan_tools.start_plan_execution.connect(self.handle_start_plan_execution)
        self.plan_tools.stop_plan_execution.connect(self.stop_plan_execution.emit)
        self.plan_tools.execute_action.connect(self.handle_execute_action)
        
        # Signal button connection
        self.plan_tools.signal_button_pressed.connect(self.signal_button_pressed.emit)
        self.plan_tools.skip_to_wait_signal.connect(self.skip_to_wait_signal.emit)
        
        # Navigation stop signal
        self.plan_tools.stop_navigation.connect(self.stop_navigation.emit)
        
        # Tab change signal - control left stacked widget
        self.plan_tools.tab_changed.connect(self.on_tab_changed)
        
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
        
        # Plan selection signal
        self.plan_tools.plan_selected.connect(self.on_plan_selected)
        
        
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
        # Note: stop_and_wait action type removed
        
        # Also emit the general execute action signal
        self.execute_plan_action.emit(plan_name, action_index)
    
    def handle_map_selected(self, map_name: str):
        """Handle map selection request"""
        self.map_load_requested.emit(map_name)
    
    def on_plan_selected(self, plan_name: str):
        """Handle plan selection and update status"""
        self.set_plan_status(f"Plan '{plan_name}' selected")
    
    
    def open_plan_editor(self):
        """Open the plan editor window"""
        if self.plan_editor is None:
            self.plan_editor = PlanEditor(self.plan_manager, self.route_manager, self.dock_manager)
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
    
    def set_current_status(self, status: str):
        """Update the robot status display and change cell background color based on status"""
        # Update the status text
        if hasattr(self, 'robot_status_text'):
            self.robot_status_text.setText(status)
        
        if hasattr(self, 'robot_status_frame'):
            # Define background colors based on status
            if status in ["At base", "At destination", "Docked", "Undocked", "Oczekiwanie na sygnał", 
                          "Zatrzymany", "Bezczynny"]:
                # Light green for completed/stable states
                bg_color = "#d5f4e6"
                text_color = "#27ae60"
            elif status in ["Nav to base", "Nav to dest", "Navigating", "Docking", "Undocking",
                            "Manual move"] or status.startswith("Wykonywanie"):
                # No color (white/light gray) for moving/active states
                bg_color = "#f8f9fa"
                text_color = "#2c3e50"
            elif status in ["Błąd", "Error", "Utracono połączenie", "Błąd Nawigacji", 
                           "Dokowanie Anulowane", "Oddokowanie Anulowane", "Dokowanie anulowane", "Oddokowanie anulowane"] or \
                 "failed" in status.lower() or "error" in status.lower() or "cancelled" in status.lower() or "błąd" in status.lower() or "anulowane" in status.lower():
                # Light red for errors and cancellations
                bg_color = "#fadbd8"
                text_color = "#e74c3c"
            elif status in ["Ostrzeżenie", "Słaba bateria", "Wykryto przeszkodę"]:
                # Light orange for warnings
                bg_color = "#fdebd0"
                text_color = "#f39c12"
            else:
                # Default light gray for unknown states
                bg_color = "#f8f9fa"
                text_color = "#2c3e50"
            
            # Update the frame style with new background color (large screen optimized)
            self.robot_status_frame.setStyleSheet(f"""
                QWidget {{
                    border: 4px solid #bdc3c7;
                    border-radius: 12px;
                    background-color: {bg_color};
                    padding: 20px;
                }}
            """)
            
            # Update text color with large fonts
            if hasattr(self, 'robot_status_text'):
                self.robot_status_text.setStyleSheet(f"""
                    QLabel {{
                        font-size: 42px;
                        color: {text_color};
                        background: none;
                        border: none;
                        font-weight: bold;
                    }}
                """)

    def set_battery_status(self, status: str):
        """Update the battery status display"""
        if hasattr(self, 'battery_status_display'):
            self.battery_status_display.setText(status)
            # Make the battery percentage font bigger
            self.battery_status_display.setStyleSheet("""
                QLabel {
                    font-size: 40px;
                    color: #2c3e50;
                    background: none;
                    border: none;
                    font-weight: bold;
                }
            """)
    
    def update_battery_status(self, percentage: int, status_string: str):
        """Update battery status display with color coding based on percentage."""
        # Update the text
        if hasattr(self, 'battery_status_display'):
            self.battery_status_display.setText(status_string)
        
        # Update the battery status frame background color based on percentage
        if hasattr(self, 'battery_status_frame'):
            if percentage <= 10:
                # Orange for warning (low battery)
                bg_color = '#fdebd0'
                border_color = '#f39c12'
            elif percentage <= 25:
                # Light yellow for caution
                bg_color = '#fef9e7'
                border_color = '#f1c40f'
            else:
                # Normal light gray background
                bg_color = '#f7f9fc'
                border_color = '#bdc3c7'
            
            # Update the frame style with new background color
            self.battery_status_frame.setStyleSheet(f'''
                QWidget {{
                    border: 4px solid {border_color};
                    border-radius: 12px;
                    background-color: {bg_color};
                    padding: 20px;
                }}
            ''')

    def set_plan_status(self, status: str):
        """Update the plan status display"""
        if hasattr(self, 'plan_status_display'):
            self.plan_status_display.setText(status)
    
    def switch_to_active(self):
        """Switch to active tab in plan tools"""
        self.plan_tools.switch_to_active()
        self.map_view.enable_drawing = False  # Disable arrow drawing in active mode
    
    def switch_to_configure(self):
        """Switch to configure tab in plan tools"""
        self.plan_tools.switch_to_configure()
        self.map_view.enable_drawing = True   # Enable arrow drawing in configure mode
    
    def on_action_completed(self):
        """Called when a plan action is completed"""
        self.plan_tools.on_action_completed()
    
    def on_plan_execution_stopped(self):
        """Called when plan execution is stopped"""
        self.plan_tools.on_plan_execution_stopped()
    
    def on_plan_execution_stopped_due_to_failure(self, plan_name: str, failure_reason: str):
        """Called when plan execution stops due to failure - don't override status"""
        # Only update the UI state, don't emit stop signals that would override the failure status
        self.plan_tools.on_plan_execution_stopped_due_to_failure()
    
    @pyqtSlot(str, int)
    def on_single_action_completed(self, plan_name: str, action_index: int):
        """Called when a single action execution is completed"""
        self.plan_tools.on_single_action_completed()
    
    @pyqtSlot(int)
    def on_tab_changed(self, tab_index: int):
        """Handle tab change to control enable_drawing and switch grid/map"""
        if tab_index == 0:  # Active tab
            self.map_view.enable_drawing = False
            # Show grid on left side
            if hasattr(self, 'left_stacked_widget'):
                self.left_stacked_widget.setCurrentIndex(0)  # Grid
                print("DEBUG: Active tab - showing grid")
        elif tab_index == 1:  # Configure tab
            self.map_view.enable_drawing = True
            self.map_view.editing_mode = False  # Ensure we're not in route editing mode
            # Show map on left side for pose setting
            if hasattr(self, 'left_stacked_widget'):
                self.left_stacked_widget.setCurrentIndex(1)  # Map
                print("DEBUG: Configure tab - showing map")
            print(f"DEBUG: Configure tab activated - enable_drawing={self.map_view.enable_drawing}, editing_mode={self.map_view.editing_mode}")
    
    def update_robot_status(self, status: str):
        """Update robot status display"""
        # Call the same logic as set_current_status to update grid cell
        self.set_current_status(status)
        
        # Forward navigation status to plan tools for signal button logic
        self.plan_tools.update_navigation_status(status)
    
    def close_plan_editor(self):
        """Close the plan editor if open"""
        if self.plan_editor:
            self.plan_editor.close()
            self.plan_editor = None
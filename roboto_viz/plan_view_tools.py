from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton,
                             QLabel, QListWidget, QListWidgetItem,
                             QTabWidget, QGridLayout, QGroupBox,
                             QMessageBox, QHBoxLayout)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QFont
from typing import Optional

from roboto_viz.plan_manager import PlanManager, ExecutionPlan, ActionType


class PlanTools(QWidget):
    on_disconnect = pyqtSignal()
    start_plan_execution = pyqtSignal(str)  # plan_name
    stop_plan_execution = pyqtSignal()
    execute_action = pyqtSignal(str, int)  # plan_name, action_index
    
    dock_robot = pyqtSignal()
    undock_robot = pyqtSignal()
    start_nav = pyqtSignal(str, bool)  # route_name, to_dest
    
    open_plan_editor = pyqtSignal()
    map_selected = pyqtSignal(str)
    
    start_keys_vel = pyqtSignal(str, float)
    stop_keys_vel = pyqtSignal()
    
    # Navigation control
    stop_navigation = pyqtSignal()
    
    # Signal handling
    signal_button_pressed = pyqtSignal()
    skip_to_wait_signal = pyqtSignal()  # Signal to skip directly to wait_for_signal action
    
    # Tab change signal
    tab_changed = pyqtSignal(int)  # tab_index
    
    # Plan selection signal
    plan_selected = pyqtSignal(str)  # plan_name

    def __init__(self, plan_manager: PlanManager):
        super().__init__()
        self.plan_manager = plan_manager
        self.current_plan: Optional[ExecutionPlan] = None
        self.is_executing = False
        self.is_navigating = False  # Track if robot is currently navigating
        self.next_action_is_wait_signal = False  # Track if next action is wait_for_signal
        
        # Styles
        self.button_style = """
            QPushButton {
                min-height: 45px;
                font-size: 16px;
                padding: 12px 20px;
                font-weight: bold;
                border: 2px solid #2c3e50;
                border-radius: 8px;
                background-color: #ecf0f1;
                color: #2c3e50;
            }
            QPushButton:hover {
                background-color: #d0d3d4;
                border-color: #34495e;
            }
            QPushButton:pressed {
                background-color: #bdc3c7;
                border-color: #2c3e50;
            }
            QPushButton:disabled {
                background-color: #f5f6f7;
                border-color: #bdc3c7;
                color: #95a5a6;
            }
        """
        
        # Note: Button styles are now inline in the UI creation
        
        self.setup_ui()
        self.setup_connections()
        self.refresh_plans()
    
    def setup_ui(self):
        # Main vertical layout
        main_layout = QVBoxLayout(self)
        
        # Title
        title_label = QLabel("Sterowanie Planem Robota")
        title_label.setFont(QFont("Arial", 12, QFont.Bold))  # Smaller font
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("QLabel { color: #2c3e50; padding: 5px; }")  # Reduced padding
        main_layout.addWidget(title_label)
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        self.tab_widget.currentChanged.connect(self.on_tab_changed)
        
        # Set stylesheet to make tabs wider
        self.tab_widget.setStyleSheet("""
            QTabWidget::tab-bar {
                alignment: center;
            }
            QTabWidget::pane {
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                background-color: white;
            }
            QTabBar::tab {
                background-color: #ecf0f1;
                border: 2px solid #bdc3c7;
                border-bottom: none;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                min-width: 120px;
                min-height: 35px;
                padding: 8px 25px;
                margin-right: 2px;
                font-size: 14px;
                font-weight: bold;
                color: #2c3e50;
            }
            QTabBar::tab:selected {
                background-color: white;
                border-color: #3498db;
                color: #2c3e50;
            }
            QTabBar::tab:hover {
                background-color: #d5dbdb;
                border-color: #85929e;
            }
        """)
        
        main_layout.addWidget(self.tab_widget)
        
        # Active tab
        self.active_tab = self.create_active_tab()
        self.tab_widget.addTab(self.active_tab, "Tryb automatyczny")
        
        # Configure tab
        self.configure_tab = self.create_configure_tab()
        self.tab_widget.addTab(self.configure_tab, "Tryb manualny")
    
    def create_active_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Plan Control Section
        control_group = QGroupBox("Sterowanie Planem")
        control_layout = QVBoxLayout(control_group)

        # Plan selection - using QListWidget for better visibility
        plan_label = QLabel("Plan:")
        plan_label.setStyleSheet("font-weight: bold; font-size: 13px; color: #2c3e50;")
        control_layout.addWidget(plan_label)

        # Plan list with navigation buttons
        plan_list_layout = QHBoxLayout()

        self.plan_list = QListWidget()
        self.plan_list.setMaximumHeight(140)  # Reduced height
        self.plan_list.setMinimumWidth(200)  # Ensure list has minimum width
        self.plan_list.setStyleSheet("""
            QListWidget {
                border: 2px solid #bdc3c7;
                border-radius: 6px;
                background-color: white;
                font-size: 14px;
                padding: 4px;
            }
            QListWidget::item {
                padding: 10px;
                min-height: 30px;
                border-bottom: 1px solid #ecf0f1;
            }
            QListWidget::item:selected {
                background-color: #3498db;
                color: white;
                border-radius: 5px;
            }
            QListWidget::item:hover {
                background-color: #d5dbdb;
            }
        """)
        plan_list_layout.addWidget(self.plan_list)

        # Navigation buttons for plan list
        self.plan_up_btn = QPushButton("▲")
        self.plan_up_btn.setFixedSize(50, 65)
        self.plan_up_btn.setStyleSheet("""
            QPushButton {
                font-size: 24px;
                font-weight: bold;
                border: 2px solid #2c3e50;
                border-radius: 8px;
                background-color: #ecf0f1;
                color: #2c3e50;
            }
            QPushButton:hover {
                background-color: #d0d3d4;
                border-color: #34495e;
            }
            QPushButton:pressed {
                background-color: #bdc3c7;
                border-color: #2c3e50;
            }
        """)
        self.plan_up_btn.clicked.connect(self.scroll_plan_up)
        plan_list_layout.addWidget(self.plan_up_btn)

        self.plan_down_btn = QPushButton("▼")
        self.plan_down_btn.setFixedSize(50, 65)
        self.plan_down_btn.setStyleSheet("""
            QPushButton {
                font-size: 24px;
                font-weight: bold;
                border: 2px solid #2c3e50;
                border-radius: 8px;
                background-color: #ecf0f1;
                color: #2c3e50;
            }
            QPushButton:hover {
                background-color: #d0d3d4;
                border-color: #34495e;
            }
            QPushButton:pressed {
                background-color: #bdc3c7;
                border-color: #2c3e50;
            }
        """)
        self.plan_down_btn.clicked.connect(self.scroll_plan_down)
        plan_list_layout.addWidget(self.plan_down_btn)

        control_layout.addLayout(plan_list_layout)

        self.choose_plan_btn = QPushButton("Wybierz Plan")
        self.choose_plan_btn.setStyleSheet("""
            QPushButton {
                min-height: 48px;
                font-size: 16px;
                padding: 12px 20px;
                font-weight: bold;
                border: 2px solid #2c3e50;
                border-radius: 8px;
                background-color: #ecf0f1;
                color: #2c3e50;
            }
            QPushButton:hover {
                background-color: #d0d3d4;
                border-color: #34495e;
            }
            QPushButton:pressed {
                background-color: #bdc3c7;
                border-color: #2c3e50;
            }
            QPushButton:disabled {
                background-color: #f5f6f7;
                border-color: #bdc3c7;
                color: #95a5a6;
            }
        """)
        control_layout.addWidget(self.choose_plan_btn)

        # Action selection - using QListWidget for better visibility
        action_label = QLabel("Przejdź do Akcji:")
        action_label.setStyleSheet("font-weight: bold; font-size: 13px; color: #2c3e50; margin-top: 8px;")
        control_layout.addWidget(action_label)

        # Action list with navigation buttons
        action_list_layout = QHBoxLayout()

        self.action_list = QListWidget()
        self.action_list.setMaximumHeight(160)  # Reduced height
        self.action_list.setMinimumWidth(200)  # Ensure list has minimum width
        self.action_list.setStyleSheet("""
            QListWidget {
                border: 2px solid #bdc3c7;
                border-radius: 6px;
                background-color: white;
                font-size: 14px;
                padding: 4px;
            }
            QListWidget::item {
                padding: 10px;
                min-height: 28px;
                border-bottom: 1px solid #ecf0f1;
            }
            QListWidget::item:selected {
                background-color: #27ae60;
                color: white;
                border-radius: 5px;
            }
            QListWidget::item:hover {
                background-color: #d5dbdb;
            }
        """)
        action_list_layout.addWidget(self.action_list)

        # Navigation buttons for action list
        self.action_up_btn = QPushButton("▲")
        self.action_up_btn.setFixedSize(50, 72)
        self.action_up_btn.setStyleSheet("""
            QPushButton {
                font-size: 24px;
                font-weight: bold;
                border: 2px solid #27ae60;
                border-radius: 8px;
                background-color: #d5f4e6;
                color: #27ae60;
            }
            QPushButton:hover {
                background-color: #a9dfbf;
                border-color: #229954;
            }
            QPushButton:pressed {
                background-color: #82c99a;
                border-color: #1e8449;
            }
        """)
        self.action_up_btn.clicked.connect(self.scroll_action_up)
        action_list_layout.addWidget(self.action_up_btn)

        self.action_down_btn = QPushButton("▼")
        self.action_down_btn.setFixedSize(50, 72)
        self.action_down_btn.setStyleSheet("""
            QPushButton {
                font-size: 24px;
                font-weight: bold;
                border: 2px solid #27ae60;
                border-radius: 8px;
                background-color: #d5f4e6;
                color: #27ae60;
            }
            QPushButton:hover {
                background-color: #a9dfbf;
                border-color: #229954;
            }
            QPushButton:pressed {
                background-color: #82c99a;
                border-color: #1e8449;
            }
        """)
        self.action_down_btn.clicked.connect(self.scroll_action_down)
        action_list_layout.addWidget(self.action_down_btn)

        control_layout.addLayout(action_list_layout)

        self.execute_action_btn = QPushButton("START")
        self.execute_action_btn.setStyleSheet("""
            QPushButton {
                min-height: 50px;
                font-size: 18px;
                font-weight: bold;
                padding: 12px 25px;
                border: 3px solid #27ae60;
                border-radius: 12px;
                background-color: #2ecc71;
                color: white;
            }
            QPushButton:hover {
                background-color: #27ae60;
                border-color: #229954;
            }
            QPushButton:pressed {
                background-color: #229954;
                border-color: #1e8449;
            }
            QPushButton:disabled {
                background-color: #a9dfbf;
                border-color: #82c99a;
                color: #566573;
            }
        """)
        control_layout.addWidget(self.execute_action_btn)


        layout.addWidget(control_group)
        
        # Main Control Buttons Section
        control_buttons_group = QGroupBox("Sterowanie")
        control_buttons_layout = QVBoxLayout(control_buttons_group)
        
        # Signal button (top - initially hidden) - LARGE for touch interaction
        self.signal_btn = QPushButton("SYGNAŁ")
        self.signal_btn.setStyleSheet("""
            QPushButton {
                min-height: 80px;
                font-size: 24px;
                padding: 20px 30px;
                font-weight: bold;
                border: 4px solid #3498db;
                border-radius: 15px;
                background-color: #5dade2;
                color: white;
            }
            QPushButton:hover {
                background-color: #3498db;
                border-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #2980b9;
                border-color: #1f618d;
            }
        """)
        self.signal_btn.setVisible(False)
        
        
        control_buttons_layout.addWidget(self.signal_btn)
        
        # Start button (middle) - VERY LARGE for touch interaction
        self.start_btn = QPushButton("WZNÓW")
        self.start_btn.setStyleSheet("""
            QPushButton {
                min-height: 90px;
                font-size: 28px;
                padding: 25px 40px;
                font-weight: bold;
                border: 4px solid #27ae60;
                border-radius: 15px;
                background-color: #2ecc71;
                color: white;
            }
            QPushButton:hover {
                background-color: #27ae60;
                border-color: #229954;
            }
            QPushButton:pressed {
                background-color: #229954;
                border-color: #1e8449;
            }
            QPushButton:disabled {
                background-color: #a9dfbf;
                border-color: #82c99a;
                color: #566573;
            }
        """)
        control_buttons_layout.addWidget(self.start_btn)

        # Stop button (bottom - always enabled) - VERY LARGE for touch interaction
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setStyleSheet("""
            QPushButton {
                min-height: 90px;
                font-size: 28px;
                padding: 25px 40px;
                font-weight: bold;
                border: 4px solid #c0392b;
                border-radius: 15px;
                background-color: #e74c3c;
                color: white;
            }
            QPushButton:hover {
                background-color: #c0392b;
                border-color: #962d22;
            }
            QPushButton:pressed {
                background-color: #962d22;
                border-color: #6d2018;
            }
        """)
        # Stop button is always enabled as precaution
        control_buttons_layout.addWidget(self.stop_btn)
        
        layout.addWidget(control_buttons_group)
        
        # Add stretch to push content to top
        layout.addStretch()
        
        return widget
    
    def create_configure_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Plan Management Section
        plan_group = QGroupBox("Zarządzanie Planami")
        plan_layout = QVBoxLayout(plan_group)

        self.edit_plans_btn = QPushButton("Edytuj Plany")
        self.edit_plans_btn.setStyleSheet("""
            QPushButton {
                min-height: 70px;
                font-size: 20px;
                padding: 20px 30px;
                font-weight: bold;
                border: 3px solid #2c3e50;
                border-radius: 12px;
                background-color: #ecf0f1;
                color: #2c3e50;
            }
            QPushButton:hover {
                background-color: #d0d3d4;
                border-color: #34495e;
            }
            QPushButton:pressed {
                background-color: #bdc3c7;
                border-color: #2c3e50;
            }
            QPushButton:disabled {
                background-color: #f5f6f7;
                border-color: #bdc3c7;
                color: #95a5a6;
            }
        """)
        plan_layout.addWidget(self.edit_plans_btn)

        layout.addWidget(plan_group)

        # Manual Control Section
        manual_group = QGroupBox("Sterowanie Ręczne")
        manual_layout = QGridLayout(manual_group)

        # Docking controls removed - not used

        # Movement controls - LARGE for touch
        arrow_button_style = """
            QPushButton {
                min-height: 80px;
                min-width: 80px;
                font-size: 32px;
                font-weight: bold;
                border: 3px solid #2c3e50;
                border-radius: 12px;
                background-color: #ecf0f1;
                color: #2c3e50;
            }
            QPushButton:hover {
                background-color: #d0d3d4;
                border-color: #34495e;
            }
            QPushButton:pressed {
                background-color: #bdc3c7;
                border-color: #2c3e50;
            }
            QPushButton:disabled {
                background-color: #f5f6f7;
                border-color: #bdc3c7;
                color: #95a5a6;
            }
        """

        self.up_btn = QPushButton("↑")
        self.down_btn = QPushButton("↓")
        self.left_btn = QPushButton("←")
        self.right_btn = QPushButton("→")

        for btn in [self.up_btn, self.down_btn, self.left_btn, self.right_btn]:
            btn.setStyleSheet(arrow_button_style)
        
        manual_layout.addWidget(self.up_btn, 0, 1)
        manual_layout.addWidget(self.left_btn, 1, 0)
        manual_layout.addWidget(self.right_btn, 1, 2)
        manual_layout.addWidget(self.down_btn, 2, 1)
        
        layout.addWidget(manual_group)
        
        layout.addStretch()
        
        return widget
    
    def setup_connections(self):
        # Plan control connections
        self.choose_plan_btn.clicked.connect(self.choose_plan)
        self.start_btn.clicked.connect(self.start_execution)
        self.stop_btn.clicked.connect(self.stop_execution)
        self.execute_action_btn.clicked.connect(self.execute_selected_action)
        self.signal_btn.clicked.connect(self.on_signal_button_clicked)
        
        # Configuration connections
        self.edit_plans_btn.clicked.connect(self.open_plan_editor.emit)
        
        # Manual control connections
        self.up_btn.pressed.connect(lambda: self.start_keys_vel.emit("f", 0.2))
        self.up_btn.released.connect(self.stop_keys_vel.emit)
        self.down_btn.pressed.connect(lambda: self.start_keys_vel.emit("b", 0.2))
        self.down_btn.released.connect(self.stop_keys_vel.emit)
        self.left_btn.pressed.connect(lambda: self.start_keys_vel.emit("l", 0.5))
        self.left_btn.released.connect(self.stop_keys_vel.emit)
        self.right_btn.pressed.connect(lambda: self.start_keys_vel.emit("r", 0.5))
        self.right_btn.released.connect(self.stop_keys_vel.emit)
        
        # Other connections - disconnect button removed
    
    def refresh_plans(self):
        """Refresh the plan list widget with available plans"""
        self.plan_list.clear()
        plan_names = self.plan_manager.get_plan_names()
        self.plan_list.addItems(plan_names)

        # Update action list if current plan exists
        if self.current_plan:
            self.refresh_action_list()

    def refresh_action_list(self):
        """Refresh the action list widget with current plan actions"""
        self.action_list.clear()
        if not self.current_plan:
            return

        for i, action in enumerate(self.current_plan.actions):
            action_name = action.name
            # Add (rev) for reversed route actions
            if action.action_type == ActionType.ROUTE and action.parameters.get('reverse', False):
                action_name = f"{action.name} (rev)"

            action_text = f"{i+1}. {action.action_type.value.replace('_', ' ').title()}: {action_name}"
            self.action_list.addItem(action_text)
    
    def choose_plan(self):
        """Choose the selected plan as current"""
        current_item = self.plan_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę najpierw wybrać plan!")
            return

        plan_name = current_item.text()
        success = self.plan_manager.set_current_plan(plan_name)
        if success:
            self.current_plan = self.plan_manager.get_current_plan()
            self.refresh_action_list()

            # Load the plan's map to robot if specified
            if self.current_plan.map_name:
                self.map_selected.emit(self.current_plan.map_name)

            # Emit plan selected signal for status updates
            self.plan_selected.emit(plan_name)

            QMessageBox.information(self, "Sukces", f"Plan '{plan_name}' został wybrany!")
        else:
            QMessageBox.warning(self, "Błąd", f"Nie udało się wybrać planu '{plan_name}'!")

    def start_execution(self):
        """Start plan execution"""
        if not self.current_plan:
            QMessageBox.warning(self, "Ostrzeżenie", "Nie wybrano planu!")
            return
        
        if len(self.current_plan.actions) == 0:
            QMessageBox.warning(self, "Ostrzeżenie", "Plan nie ma żadnych akcji!")
            return
        
        self.is_executing = True
        self.start_btn.setEnabled(False)
        # Stop button always stays enabled as precaution
        
        # Check if next action is wait_for_signal
        self._check_next_action_is_wait_signal()
        
        self.start_plan_execution.emit(self.current_plan.name)
    
    def stop_execution(self):
        """Stop plan execution"""
        self.is_executing = False
        self.start_btn.setEnabled(True)
        # Stop button always stays enabled as precaution
        self.signal_btn.setVisible(False)  # Hide signal button
        
        # Always stop both plan execution and navigation as precaution
        self.stop_plan_execution.emit()
        self.stop_navigation.emit()
    
    def execute_selected_action(self):
        """Execute the selected action immediately"""
        if not self.current_plan:
            QMessageBox.warning(self, "Ostrzeżenie", "Nie wybrano planu!")
            return

        action_index = self.action_list.currentRow()
        if action_index < 0:
            QMessageBox.warning(self, "Ostrzeżenie", "Nie wybrano akcji!")
            return

        # Set execution state like start button
        self.is_executing = True
        self.start_btn.setEnabled(False)
        # Stop button always stays enabled as precaution

        # Check if next action is wait_for_signal
        self._check_next_action_is_wait_signal()

        self.current_plan.set_current_action(action_index)
        self.execute_action.emit(self.current_plan.name, action_index)
    
    def on_action_completed(self):
        """Called when an action is completed during execution"""
        if not self.is_executing or not self.current_plan:
            return
        
        # Check if next action is wait_for_signal for continuous execution
        self._check_next_action_is_wait_signal()
        
        # This is called for continuous plan execution
        # Move to next action
        next_action = self.current_plan.next_action()
    
    def on_plan_execution_stopped(self):
        """Called when plan execution is stopped externally"""
        self.stop_execution()
    
    def on_plan_execution_stopped_due_to_failure(self):
        """Called when plan execution stops due to failure - don't call stop_navigation"""
        # Update UI state without calling stop signals that would override failure status
        self.is_executing = False
        self.start_btn.setEnabled(True)
        self.signal_btn.setVisible(False)
        # DON'T emit stop_plan_execution or stop_navigation signals
    
    def on_signal_button_clicked(self):
        """Called when signal button is clicked"""
        if self.is_navigating and self.is_executing and self._plan_has_wait_signal_action():
            # Cancel navigation and skip directly to wait_for_signal action
            self.stop_navigation.emit()  # Stop current navigation
            self.skip_to_wait_signal.emit()  # Signal to skip to wait_for_signal action
            # Don't hide button - it will be re-shown when we reach the wait_for_signal action
            # Don't emit signal_button_pressed when navigating - we want to wait at the wait_for_signal action
        else:
            # Regular signal button press for wait_for_signal actions
            self.signal_button_pressed.emit()  # Emit the signal for regular wait_for_signal handling
            # Hide button after regular signal press
            self.signal_btn.setVisible(False)
    
    def on_single_action_completed(self):
        """Called when a single action execution is completed"""
        self.is_executing = False
        self.start_btn.setEnabled(True)
        # Stop button always stays enabled as precaution
        self.signal_btn.setVisible(False)
    
    def update_robot_status(self, status: str):
        """Robot status now updated in grid on left side"""
        pass  # Status is now handled by the grid
    
    def switch_to_active(self):
        """Switch to active tab"""
        self.tab_widget.setCurrentIndex(0)
    
    def switch_to_configure(self):
        """Switch to configure tab"""
        self.tab_widget.setCurrentIndex(1)
    
    def show_signal_button(self, signal_name: str):
        """Show the signal button for wait-for-signal actions"""
        self.signal_btn.setText(f'SIGNAL ({signal_name})')
        self.signal_btn.setVisible(True)
    
    def hide_signal_button(self):
        """Hide the signal button"""
        # Don't hide if we're navigating and next action is wait_for_signal
        if self.is_navigating and self.next_action_is_wait_signal and self.is_executing:
            return
        
        # Don't hide if button was just shown for wait_for_signal action
        if self.signal_btn.isVisible() and self.signal_btn.text().startswith('SIGNAL ('):
            return
            
        self.signal_btn.setVisible(False)
    
    def on_tab_changed(self, index: int):
        """Handle tab change - emit signal for external handling"""
        self.tab_changed.emit(index)
        if index == 1:
            # Manual mode tab - stop navigation
            self.stop_navigation.emit()
    
    def on_tab_changed_external(self, index: int):
        """Handle tab change from external signal"""
        # This will be connected from plan_views to handle enable_drawing
        pass
    
    def update_navigation_status(self, status: str):
        """Update navigation status and show/hide signal button accordingly"""
        # Check if robot is currently navigating
        was_navigating = self.is_navigating
        # Check for both English and Polish navigation statuses
        self.is_navigating = status in ["Nav to dest", "Nav to base", "Navigating", 
                                       "Nawigacja do celu", "Nawigacja do bazy", "Nawiguje"]
        
        # Update signal button visibility based on navigation state and next action
        self._update_signal_button_visibility()
    
    def _update_signal_button_visibility(self):
        """Update signal button visibility based on current state"""
        # Show button if:
        # 1. Waiting for signal (already handled by show_signal_button), OR
        # 2. Currently navigating AND there's a wait_for_signal action somewhere in the plan
        if self.is_navigating and self.is_executing and self._plan_has_wait_signal_action():
            # Show button for navigation cancellation to proceed to wait signal
            self._show_navigation_signal_button()
    
    def _show_navigation_signal_button(self):
        """Show signal button during navigation when there's a wait_for_signal action in the plan"""
        if not self.current_plan or len(self.current_plan.actions) == 0:
            return
        
        # Find the next wait_for_signal action in the plan    
        wait_action = self._find_next_wait_signal_action()
        
        if wait_action:
            signal_name = wait_action.parameters.get('signal_name', 'default')
            self.signal_btn.setText(f'CONTINUE TO SIGNAL ({signal_name})')
            self.signal_btn.setVisible(True)
    
    def _plan_has_wait_signal_action(self):
        """Check if the current plan has any wait_for_signal actions"""
        if not self.current_plan or len(self.current_plan.actions) == 0:
            return False
            
        for action in self.current_plan.actions:
            if hasattr(action, 'action_type') and action.action_type.value == "wait_for_signal":
                return True
        return False
    
    def _find_next_wait_signal_action(self):
        """Find the next wait_for_signal action in the plan (may not be immediate next)"""
        if not self.current_plan or len(self.current_plan.actions) == 0:
            return None
            
        current_index = self.current_plan.current_action_index
        
        # Look for wait_for_signal actions starting from current position + 1
        for i in range(len(self.current_plan.actions)):
            check_index = (current_index + 1 + i) % len(self.current_plan.actions)
            action = self.current_plan.actions[check_index]
            if hasattr(action, 'action_type') and action.action_type.value == "wait_for_signal":
                return action
        return None
    
    def _check_next_action_is_wait_signal(self):
        """Check if the next action in the current plan is wait_for_signal"""
        if not self.current_plan or not self.is_executing:
            self.next_action_is_wait_signal = False
            return
            
        current_index = self.current_plan.current_action_index
        next_index = current_index + 1
        
        if next_index < len(self.current_plan.actions):
            next_action = self.current_plan.actions[next_index]
            self.next_action_is_wait_signal = (hasattr(next_action, 'action_type') and 
                                             next_action.action_type.value == "wait_for_signal")
        else:
            # Next action would be first action (plan loops)
            if len(self.current_plan.actions) > 0:
                first_action = self.current_plan.actions[0]
                self.next_action_is_wait_signal = (hasattr(first_action, 'action_type') and 
                                                 first_action.action_type.value == "wait_for_signal")
            else:
                self.next_action_is_wait_signal = False
        
        # Update button visibility after checking next action
        self._update_signal_button_visibility()

    def scroll_plan_up(self):
        """Scroll plan list up by one item"""
        current_row = self.plan_list.currentRow()
        if current_row > 0:
            self.plan_list.setCurrentRow(current_row - 1)

    def scroll_plan_down(self):
        """Scroll plan list down by one item"""
        current_row = self.plan_list.currentRow()
        if current_row < self.plan_list.count() - 1:
            self.plan_list.setCurrentRow(current_row + 1)

    def scroll_action_up(self):
        """Scroll action list up by one item"""
        current_row = self.action_list.currentRow()
        if current_row > 0:
            self.action_list.setCurrentRow(current_row - 1)

    def scroll_action_down(self):
        """Scroll action list down by one item"""
        current_row = self.action_list.currentRow()
        if current_row < self.action_list.count() - 1:
            self.action_list.setCurrentRow(current_row + 1)


from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QLabel, QListWidget, QListWidgetItem, QComboBox, 
                             QTabWidget, QLineEdit, QGridLayout, QGroupBox,
                             QMessageBox, QProgressBar)
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
    
    # Tab change signal
    tab_changed = pyqtSignal(int)  # tab_index
    
    # Plan selection signal
    plan_selected = pyqtSignal(str)  # plan_name

    def __init__(self, plan_manager: PlanManager):
        super().__init__()
        self.plan_manager = plan_manager
        self.current_plan: Optional[ExecutionPlan] = None
        self.is_executing = False
        
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
        main_layout.addWidget(self.tab_widget)
        
        # Active tab
        self.active_tab = self.create_active_tab()
        self.tab_widget.addTab(self.active_tab, "Aktywny")
        
        # Configure tab
        self.configure_tab = self.create_configure_tab()
        self.tab_widget.addTab(self.configure_tab, "Konfiguracja")
    
    def create_active_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Plan Control Section
        control_group = QGroupBox("Sterowanie Planem")
        control_layout = QVBoxLayout(control_group)
        
        # Plan selection
        plan_select_layout = QHBoxLayout()
        plan_select_layout.addWidget(QLabel("Plan:"))
        self.plan_combo = QComboBox()
        plan_select_layout.addWidget(self.plan_combo)
        self.choose_plan_btn = QPushButton("Wybierz Plan")
        self.choose_plan_btn.setStyleSheet(self.button_style)
        plan_select_layout.addWidget(self.choose_plan_btn)
        control_layout.addLayout(plan_select_layout)
        
        # Note: Main control buttons moved to below robot status
        
        # Action selection
        action_layout = QHBoxLayout()
        action_layout.addWidget(QLabel("Przejdź do Akcji:"))
        self.action_combo = QComboBox()
        action_layout.addWidget(self.action_combo)
        self.execute_action_btn = QPushButton("Wykonaj")
        self.execute_action_btn.setStyleSheet(self.button_style)
        action_layout.addWidget(self.execute_action_btn)
        control_layout.addLayout(action_layout)
        
        
        layout.addWidget(control_group)
        
        # Main Control Buttons Section
        control_buttons_group = QGroupBox("Sterowanie")
        control_buttons_layout = QVBoxLayout(control_buttons_group)
        
        # Signal button (top - initially hidden) - made taller
        self.signal_btn = QPushButton("SYGNAŁ")
        self.signal_btn.setStyleSheet("""
            QPushButton {
                min-height: 50px;
                font-size: 18px;
                padding: 12px 25px;
                font-weight: bold;
                border: 2px solid #3498db;
                border-radius: 10px;
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
        
        # Start button (middle) - made taller
        self.start_btn = QPushButton("START")
        self.start_btn.setStyleSheet("""
            QPushButton {
                min-height: 65px;
                font-size: 22px;
                padding: 18px 35px;
                font-weight: bold;
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
        control_buttons_layout.addWidget(self.start_btn)
        
        # Stop button (bottom - always enabled) - made taller
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setStyleSheet("""
            QPushButton {
                min-height: 65px;
                font-size: 22px;
                padding: 18px 35px;
                font-weight: bold;
                border: 3px solid #c0392b;
                border-radius: 12px;
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
        self.edit_plans_btn.setStyleSheet(self.button_style)
        plan_layout.addWidget(self.edit_plans_btn)
        
        layout.addWidget(plan_group)
        
        # Manual Control Section
        manual_group = QGroupBox("Sterowanie Ręczne")
        manual_layout = QGridLayout(manual_group)
        
        # Docking controls removed - not used
        
        # Movement controls
        self.up_btn = QPushButton("↑")
        self.down_btn = QPushButton("↓")
        self.left_btn = QPushButton("←")
        self.right_btn = QPushButton("→")
        
        for btn in [self.up_btn, self.down_btn, self.left_btn, self.right_btn]:
            btn.setStyleSheet(self.button_style)
            btn.setMinimumSize(40, 40)  # Smaller buttons
        
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
        """Refresh the plan combo box with available plans"""
        self.plan_combo.clear()
        plan_names = self.plan_manager.get_plan_names()
        self.plan_combo.addItems(plan_names)
        
        # Update action combo if current plan exists
        if self.current_plan:
            self.refresh_action_combo()
    
    def refresh_action_combo(self):
        """Refresh the action combo box with current plan actions"""
        self.action_combo.clear()
        if not self.current_plan:
            return
        
        for i, action in enumerate(self.current_plan.actions):
            action_name = action.name
            # Add (rev) for reversed route actions
            if action.action_type == ActionType.ROUTE and action.parameters.get('reverse', False):
                action_name = f"{action.name} (rev)"
            
            action_text = f"{i+1}. {action.action_type.value.replace('_', ' ').title()}: {action_name}"
            self.action_combo.addItem(action_text)
    
    def choose_plan(self):
        """Choose the selected plan as current"""
        plan_name = self.plan_combo.currentText()
        if not plan_name:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę najpierw wybrać plan!")
            return
        
        success = self.plan_manager.set_current_plan(plan_name)
        if success:
            self.current_plan = self.plan_manager.get_current_plan()
            self.refresh_action_combo()
            
            # Load the plan's map if specified - this will trigger position loading
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
        
        action_index = self.action_combo.currentIndex()
        if action_index < 0:
            QMessageBox.warning(self, "Ostrzeżenie", "Nie wybrano akcji!")
            return
        
        # Set execution state like start button
        self.is_executing = True
        self.start_btn.setEnabled(False)
        # Stop button always stays enabled as precaution
        
        self.current_plan.set_current_action(action_index)
        self.execute_action.emit(self.current_plan.name, action_index)
    
    def on_action_completed(self):
        """Called when an action is completed during execution"""
        if not self.is_executing or not self.current_plan:
            return
        
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
        self.signal_btn.setVisible(False)  # Hide immediately after clicking
        self.signal_button_pressed.emit()  # Emit the signal
    
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
        self.signal_btn.setVisible(False)
    
    def on_tab_changed(self, index: int):
        """Handle tab change - emit signal for external handling"""
        self.tab_changed.emit(index)
    
    def on_tab_changed_external(self, index: int):
        """Handle tab change from external signal"""
        # This will be connected from plan_views to handle enable_drawing
        pass
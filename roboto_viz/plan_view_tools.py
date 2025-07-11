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

    def __init__(self, plan_manager: PlanManager):
        super().__init__()
        self.plan_manager = plan_manager
        self.current_plan: Optional[ExecutionPlan] = None
        self.is_executing = False
        
        # Styles
        self.button_style = """
            QPushButton {
                min-height: 20px;
                font-size: 14px;
                padding: 5px 10px;
                font-weight: bold;
                border: 2px solid #2c3e50;
                border-radius: 5px;
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
        
        self.start_button_style = """
            QPushButton {
                min-height: 25px;
                font-size: 16px;
                padding: 8px 15px;
                font-weight: bold;
                border: 2px solid #27ae60;
                border-radius: 5px;
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
        """
        
        self.stop_button_style = """
            QPushButton {
                min-height: 25px;
                font-size: 16px;
                padding: 8px 15px;
                font-weight: bold;
                border: 2px solid #c0392b;
                border-radius: 5px;
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
        """
        
        self.setup_ui()
        self.setup_connections()
        self.refresh_plans()
    
    def setup_ui(self):
        # Main vertical layout
        main_layout = QVBoxLayout(self)
        
        # Title
        title_label = QLabel("Robot Plan Control")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("QLabel { color: #2c3e50; padding: 10px; }")
        main_layout.addWidget(title_label)
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)
        
        # Active tab
        self.active_tab = self.create_active_tab()
        self.tab_widget.addTab(self.active_tab, "Active")
        
        # Configure tab
        self.configure_tab = self.create_configure_tab()
        self.tab_widget.addTab(self.configure_tab, "Configure")
    
    def create_active_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Plan Status Section
        status_group = QGroupBox("Plan Status")
        status_layout = QVBoxLayout(status_group)
        
        # Current plan display
        self.current_plan_label = QLabel("No plan selected")
        self.current_plan_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.current_plan_label.setStyleSheet("QLabel { color: #2c3e50; padding: 5px; }")
        status_layout.addWidget(self.current_plan_label)
        
        # Current action display
        self.current_action_label = QLabel("No action")
        self.current_action_label.setStyleSheet("QLabel { color: #7f8c8d; padding: 5px; }")
        status_layout.addWidget(self.current_action_label)
        
        # Execution status
        self.execution_status_label = QLabel("")
        self.execution_status_label.setStyleSheet("QLabel { color: #27ae60; padding: 5px; font-weight: bold; }")
        self.execution_status_label.setVisible(False)
        status_layout.addWidget(self.execution_status_label)
        
        layout.addWidget(status_group)
        
        # Plan Control Section
        control_group = QGroupBox("Plan Control")
        control_layout = QVBoxLayout(control_group)
        
        # Plan selection
        plan_select_layout = QHBoxLayout()
        plan_select_layout.addWidget(QLabel("Plan:"))
        self.plan_combo = QComboBox()
        plan_select_layout.addWidget(self.plan_combo)
        self.choose_plan_btn = QPushButton("Choose Plan")
        self.choose_plan_btn.setStyleSheet(self.button_style)
        plan_select_layout.addWidget(self.choose_plan_btn)
        control_layout.addLayout(plan_select_layout)
        
        # Main control buttons
        main_buttons_layout = QHBoxLayout()
        self.start_btn = QPushButton("START")
        self.start_btn.setStyleSheet(self.start_button_style)
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setStyleSheet(self.stop_button_style)
        self.stop_btn.setEnabled(False)
        
        main_buttons_layout.addWidget(self.start_btn)
        main_buttons_layout.addWidget(self.stop_btn)
        control_layout.addLayout(main_buttons_layout)
        
        # Action selection
        action_layout = QHBoxLayout()
        action_layout.addWidget(QLabel("Jump to Action:"))
        self.action_combo = QComboBox()
        action_layout.addWidget(self.action_combo)
        self.execute_action_btn = QPushButton("Execute")
        self.execute_action_btn.setStyleSheet(self.button_style)
        action_layout.addWidget(self.execute_action_btn)
        control_layout.addLayout(action_layout)
        
        layout.addWidget(control_group)
        
        # Robot Status Section
        robot_group = QGroupBox("Robot Status")
        robot_layout = QVBoxLayout(robot_group)
        
        self.robot_status_label = QLabel("Status: Unknown")
        self.robot_status_label.setStyleSheet("QLabel { color: #7f8c8d; padding: 5px; }")
        robot_layout.addWidget(self.robot_status_label)
        
        layout.addWidget(robot_group)
        
        # Disconnect button
        layout.addStretch()
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setStyleSheet(self.button_style)
        layout.addWidget(self.disconnect_btn)
        
        return widget
    
    def create_configure_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Plan Management Section
        plan_group = QGroupBox("Plan Management")
        plan_layout = QVBoxLayout(plan_group)
        
        self.edit_plans_btn = QPushButton("Edit Plans")
        self.edit_plans_btn.setStyleSheet(self.button_style)
        plan_layout.addWidget(self.edit_plans_btn)
        
        layout.addWidget(plan_group)
        
        # Manual Control Section
        manual_group = QGroupBox("Manual Control")
        manual_layout = QGridLayout(manual_group)
        
        # Docking controls
        self.dock_btn = QPushButton("Dock")
        self.dock_btn.setStyleSheet(self.button_style)
        self.undock_btn = QPushButton("Undock")
        self.undock_btn.setStyleSheet(self.button_style)
        
        manual_layout.addWidget(self.dock_btn, 0, 0)
        manual_layout.addWidget(self.undock_btn, 0, 1)
        
        # Movement controls
        self.up_btn = QPushButton("↑")
        self.down_btn = QPushButton("↓")
        self.left_btn = QPushButton("←")
        self.right_btn = QPushButton("→")
        
        for btn in [self.up_btn, self.down_btn, self.left_btn, self.right_btn]:
            btn.setStyleSheet(self.button_style)
            btn.setMinimumSize(50, 50)
        
        manual_layout.addWidget(self.up_btn, 1, 1)
        manual_layout.addWidget(self.left_btn, 2, 0)
        manual_layout.addWidget(self.right_btn, 2, 2)
        manual_layout.addWidget(self.down_btn, 3, 1)
        
        layout.addWidget(manual_group)
        
        layout.addStretch()
        
        return widget
    
    def setup_connections(self):
        # Plan control connections
        self.choose_plan_btn.clicked.connect(self.choose_plan)
        self.start_btn.clicked.connect(self.start_execution)
        self.stop_btn.clicked.connect(self.stop_execution)
        self.execute_action_btn.clicked.connect(self.execute_selected_action)
        
        # Configuration connections
        self.edit_plans_btn.clicked.connect(self.open_plan_editor.emit)
        self.dock_btn.clicked.connect(self.dock_robot.emit)
        self.undock_btn.clicked.connect(self.undock_robot.emit)
        
        # Manual control connections
        self.up_btn.pressed.connect(lambda: self.start_keys_vel.emit("up", 0.2))
        self.up_btn.released.connect(self.stop_keys_vel.emit)
        self.down_btn.pressed.connect(lambda: self.start_keys_vel.emit("down", 0.2))
        self.down_btn.released.connect(self.stop_keys_vel.emit)
        self.left_btn.pressed.connect(lambda: self.start_keys_vel.emit("left", 0.5))
        self.left_btn.released.connect(self.stop_keys_vel.emit)
        self.right_btn.pressed.connect(lambda: self.start_keys_vel.emit("right", 0.5))
        self.right_btn.released.connect(self.stop_keys_vel.emit)
        
        # Other connections
        self.disconnect_btn.clicked.connect(self.on_disconnect.emit)
    
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
            action_text = f"{i+1}. {action.action_type.value.replace('_', ' ').title()}: {action.name}"
            self.action_combo.addItem(action_text)
    
    def choose_plan(self):
        """Choose the selected plan as current"""
        plan_name = self.plan_combo.currentText()
        if not plan_name:
            QMessageBox.warning(self, "Warning", "Please select a plan first!")
            return
        
        success = self.plan_manager.set_current_plan(plan_name)
        if success:
            self.current_plan = self.plan_manager.get_current_plan()
            self.update_plan_status()
            self.refresh_action_combo()
            
            # Load the plan's map if specified
            if self.current_plan.map_name:
                self.map_selected.emit(self.current_plan.map_name)
            
            QMessageBox.information(self, "Success", f"Plan '{plan_name}' selected!")
        else:
            QMessageBox.warning(self, "Error", f"Failed to select plan '{plan_name}'!")
    
    def update_plan_status(self):
        """Update the plan status display"""
        if not self.current_plan:
            self.current_plan_label.setText("No plan selected")
            self.current_action_label.setText("No action")
            return
        
        self.current_plan_label.setText(f"Plan: {self.current_plan.name}")
        
        current_action = self.current_plan.get_current_action()
        if current_action:
            action_text = f"Action {self.current_plan.current_action_index + 1}: {current_action.name}"
            self.current_action_label.setText(action_text)
        else:
            self.current_action_label.setText("No actions in plan")
    
    def start_execution(self):
        """Start plan execution"""
        if not self.current_plan:
            QMessageBox.warning(self, "Warning", "No plan selected!")
            return
        
        if len(self.current_plan.actions) == 0:
            QMessageBox.warning(self, "Warning", "Plan has no actions!")
            return
        
        self.is_executing = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.execution_status_label.setText("In Progress")
        self.execution_status_label.setVisible(True)
        
        self.start_plan_execution.emit(self.current_plan.name)
        self.update_plan_status()
    
    def stop_execution(self):
        """Stop plan execution"""
        self.is_executing = False
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.execution_status_label.setVisible(False)
        
        self.stop_plan_execution.emit()
    
    def execute_selected_action(self):
        """Execute the selected action immediately"""
        if not self.current_plan:
            QMessageBox.warning(self, "Warning", "No plan selected!")
            return
        
        action_index = self.action_combo.currentIndex()
        if action_index < 0:
            QMessageBox.warning(self, "Warning", "No action selected!")
            return
        
        self.current_plan.set_current_action(action_index)
        self.execute_action.emit(self.current_plan.name, action_index)
        self.update_plan_status()
    
    def on_action_completed(self):
        """Called when an action is completed during execution"""
        if not self.is_executing or not self.current_plan:
            return
        
        # Move to next action
        next_action = self.current_plan.next_action()
        self.update_plan_status()
    
    def on_plan_execution_stopped(self):
        """Called when plan execution is stopped externally"""
        self.stop_execution()
    
    def update_robot_status(self, status: str):
        """Update robot status display"""
        self.robot_status_label.setText(f"Status: {status}")
    
    def switch_to_active(self):
        """Switch to active tab"""
        self.tab_widget.setCurrentIndex(0)
    
    def switch_to_configure(self):
        """Switch to configure tab"""
        self.tab_widget.setCurrentIndex(1)
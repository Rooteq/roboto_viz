from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QListWidget, QListWidgetItem, QLabel, 
                             QLineEdit, QComboBox, QTextEdit, QSplitter, QGroupBox,
                             QMessageBox, QInputDialog, QGridLayout, QFrame, QCheckBox, QDialog)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from typing import Optional

from roboto_viz.plan_manager import PlanManager, ExecutionPlan, PlanAction, ActionType
from roboto_viz.route_manager import RouteManager, BezierRoute
from roboto_viz.dock_manager import DockManager, Dock
from roboto_viz.map_view import MapView
from roboto_viz.speed_zone_editor import SpeedZoneEditor


class WaitActionDialog(QDialog):
    """Custom dialog for configuring wait actions"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Dodaj Akcję Oczekiwania")
        self.setModal(True)
        self.resize(400, 200)  # Smaller since we removed options
        
        self.signal_name = 'default'
        # CAN signal activation (no parameters needed - any CAN message on ID 0x69 triggers)
        
        layout = QVBoxLayout()
        
        # Signal name input
        signal_label = QLabel("Nazwa sygnału:")
        self.signal_input = QLineEdit("default")
        layout.addWidget(signal_label)
        layout.addWidget(self.signal_input)
        
        # Info label explaining the behavior
        info_label = QLabel("Akcja oczekiwania zostanie aktywowana przez naciśnięcie niebieskiego przycisku lub sygnał CAN (ID 0x69).")
        info_label.setStyleSheet("color: #666; font-style: italic; margin: 10px 0;")
        layout.addWidget(info_label)
        
        # Buttons
        buttons_layout = QHBoxLayout()
        self.ok_button = QPushButton("OK")
        self.cancel_button = QPushButton("Anuluj")
        
        self.ok_button.clicked.connect(self.accept)
        self.cancel_button.clicked.connect(self.reject)
        
        buttons_layout.addWidget(self.ok_button)
        buttons_layout.addWidget(self.cancel_button)
        layout.addLayout(buttons_layout)
        
        self.setLayout(layout)
    
    def accept(self):
        self.signal_name = self.signal_input.text() or 'default'
        super().accept()


class RouteSelectionDialog(QDialog):
    """Custom dialog for selecting a route with optional reverse checkbox"""
    
    def __init__(self, route_names, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Dodaj Akcję Trasy")
        self.setModal(True)
        self.resize(400, 180)  # Larger for 1920x1080 screens
        
        self.route_name = None
        self.reverse = False
        
        layout = QVBoxLayout()
        
        # Route selection
        route_label = QLabel("Wybierz trasę:")
        layout.addWidget(route_label)
        
        self.route_combo = QComboBox()
        self.route_combo.addItems(route_names)
        layout.addWidget(self.route_combo)
        
        # Reverse checkbox
        self.reverse_checkbox = QCheckBox("Uruchom trasę w odwrotnym kierunku")
        layout.addWidget(self.reverse_checkbox)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        ok_button = QPushButton("OK")
        ok_button.clicked.connect(self.accept)
        button_layout.addWidget(ok_button)
        
        cancel_button = QPushButton("Anuluj")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def get_selection(self):
        """Get the selected route name and reverse state"""
        return self.route_combo.currentText(), self.reverse_checkbox.isChecked()


class DockSelectionDialog(QDialog):
    """Custom dialog for selecting a dock for dock actions"""
    
    def __init__(self, dock_names, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Dodaj Akcję Dokowania")
        self.setModal(True)
        self.resize(400, 150)  # Larger for 1920x1080 screens
        
        layout = QVBoxLayout()
        
        # Dock selection
        dock_label = QLabel("Wybierz dokowanie:")
        layout.addWidget(dock_label)
        
        self.dock_combo = QComboBox()
        self.dock_combo.addItems(dock_names)
        layout.addWidget(self.dock_combo)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        ok_button = QPushButton("OK")
        ok_button.clicked.connect(self.accept)
        button_layout.addWidget(ok_button)
        
        cancel_button = QPushButton("Anuluj")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def get_selection(self):
        """Get the selected dock name"""
        return self.dock_combo.currentText()


class PlanEditor(QMainWindow):
    plan_selected = pyqtSignal(str)  # plan_name
    plan_updated = pyqtSignal()
    
    def __init__(self, plan_manager: PlanManager, route_manager: RouteManager, dock_manager: DockManager):
        super().__init__()
        self.plan_manager = plan_manager
        self.route_manager = route_manager
        self.dock_manager = dock_manager
        self.current_plan: Optional[ExecutionPlan] = None
        self.editing_route_name: Optional[str] = None  # Track if we're editing an existing route
        self.editing_dock_name: Optional[str] = None   # Track if we're editing an existing dock
        
        # Create a new map view for the editor
        self.map_view = MapView()
        
        # Create speed zone editor (will be shown as popup)
        self.speed_zone_editor = None
        self.speed_zone_window = None
        
        self.setWindowTitle("Edytor Planów")
        self.setGeometry(100, 100, 1600, 900)  # Much larger for 1920x1080 screens
        
        # Apply global font scaling for large screens (1920x1080)
        font = self.font()
        font.setPointSize(14)  # Larger base font size for visibility
        self.setFont(font)
        
        # Apply global scaling optimized for large screens (1920x1080)
        self.setStyleSheet("""
            QWidget {
                font-size: 14px;
            }
            QLabel {
                font-size: 14px;
            }
            QPushButton {
                font-size: 16px;
                min-height: 40px;
                padding: 8px 16px;
                font-weight: bold;
            }
            QComboBox {
                font-size: 14px;
                min-height: 35px;
                padding: 5px;
            }
            QListWidget {
                font-size: 14px;
            }
            QLineEdit {
                font-size: 14px;
                min-height: 35px;
                padding: 5px;
            }
            QTextEdit {
                font-size: 14px;
            }
            QGroupBox {
                font-size: 16px;
                font-weight: bold;
                padding-top: 25px;
                margin-top: 10px;
            }
            QTabWidget {
                font-size: 14px;
            }
            QTabBar::tab {
                font-size: 14px;
                min-height: 35px;
                padding: 8px 16px;
                font-weight: bold;
            }
            QDialog {
                font-size: 14px;
            }
            QCheckBox {
                font-size: 14px;
                min-height: 25px;
            }
            QMessageBox {
                font-size: 14px;
            }
        """)
        
        self.setup_ui()
        self.setup_connections()
        self.refresh_plan_list()
        self.refresh_routes_list()
        
        # Always start in fullscreen
        self.showFullScreen()
    
    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main horizontal layout
        main_layout = QHBoxLayout(central_widget)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left panel - Plan Management
        left_panel = self.create_left_panel()
        splitter.addWidget(left_panel)
        
        # Right panel - Map and Route Editor
        right_panel = self.create_right_panel()
        splitter.addWidget(right_panel)
        
        # Set initial splitter proportions - adjusted for large screen (1600px width)
        splitter.setSizes([400, 1200])  # Larger left panel for 1600px total width
    
    def create_left_panel(self) -> QWidget:
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # Plan List Section
        plan_group = QGroupBox("Plany")
        plan_layout = QVBoxLayout(plan_group)
        
        # Plan list
        self.plan_list = QListWidget()
        plan_layout.addWidget(self.plan_list)
        
        # Plan control buttons
        plan_buttons_layout = QHBoxLayout()
        self.new_plan_btn = QPushButton("Nowy Plan")
        self.delete_plan_btn = QPushButton("Usuń Plan")
        self.duplicate_plan_btn = QPushButton("Duplikuj")
        
        plan_buttons_layout.addWidget(self.new_plan_btn)
        plan_buttons_layout.addWidget(self.delete_plan_btn)
        plan_buttons_layout.addWidget(self.duplicate_plan_btn)
        plan_layout.addLayout(plan_buttons_layout)
        
        left_layout.addWidget(plan_group)
        
        # Plan Details Section
        details_group = QGroupBox("Szczegóły Planu")
        details_layout = QVBoxLayout(details_group)
        
        # Plan name
        details_layout.addWidget(QLabel("Nazwa Planu:"))
        self.plan_name_edit = QLineEdit()
        details_layout.addWidget(self.plan_name_edit)
        
        
        # Map selection
        details_layout.addWidget(QLabel("Mapa:"))
        self.map_combo = QComboBox()
        details_layout.addWidget(self.map_combo)
        
        # Load map button
        self.load_map_btn = QPushButton("Załaduj Mapę")
        details_layout.addWidget(self.load_map_btn)
        
        left_layout.addWidget(details_group)
        
        # Actions Section
        actions_group = QGroupBox("Akcje Planu")
        actions_layout = QVBoxLayout(actions_group)
        
        # Actions list
        self.actions_list = QListWidget()
        actions_layout.addWidget(self.actions_list)
        
        # Action control buttons
        action_buttons_layout = QGridLayout()
        
        self.add_route_btn = QPushButton("Dodaj Trasę")
        self.add_dock_btn = QPushButton("Dodaj Dokowanie")
        self.add_undock_btn = QPushButton("Dodaj Oddokowanie")
        self.add_wait_btn = QPushButton("Dodaj Oczekiwanie")
        # Note: Add Stop button removed
        self.remove_action_btn = QPushButton("Usuń Akcję")
        self.move_up_btn = QPushButton("Przenieś Wyżej")
        self.move_down_btn = QPushButton("Przenieś Niżej")
        
        action_buttons_layout.addWidget(self.add_route_btn, 0, 0)
        action_buttons_layout.addWidget(self.add_dock_btn, 0, 1)
        action_buttons_layout.addWidget(self.add_undock_btn, 1, 0)
        action_buttons_layout.addWidget(self.add_wait_btn, 1, 1)
        # Note: add_stop_btn removed
        action_buttons_layout.addWidget(self.remove_action_btn, 2, 0)
        action_buttons_layout.addWidget(self.move_up_btn, 2, 1)
        action_buttons_layout.addWidget(self.move_down_btn, 3, 0)
        
        actions_layout.addLayout(action_buttons_layout)
        
        left_layout.addWidget(actions_group)
        
        # Add stretch to push everything to top
        left_layout.addStretch()
        
        return left_widget
    
    def create_right_panel(self) -> QWidget:
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Map controls
        map_controls_layout = QHBoxLayout()
        
        map_label = QLabel("Widok Mapy i Edytor Tras")
        map_label.setFont(QFont("Arial", 12, QFont.Bold))
        map_controls_layout.addWidget(map_label)
        
        map_controls_layout.addStretch()
        
        # Route editing controls (initially hidden)
        self.route_editing_widget = QWidget()
        route_editing_layout = QHBoxLayout(self.route_editing_widget)
        route_editing_layout.setContentsMargins(0, 0, 0, 0)
        
        self.clear_route_btn = QPushButton("Wyczyść Trasę")
        self.save_route_btn = QPushButton("Zapisz Trasę")
        self.cancel_route_btn = QPushButton("Anuluj")
        
        route_editing_layout.addWidget(self.clear_route_btn)
        route_editing_layout.addWidget(self.save_route_btn)
        route_editing_layout.addWidget(self.cancel_route_btn)
        
        map_controls_layout.addWidget(self.route_editing_widget)
        self.route_editing_widget.setVisible(False)  # Initially hidden
        
        # Dock editing controls (initially hidden)
        self.dock_editing_widget = QWidget()
        dock_editing_layout = QHBoxLayout(self.dock_editing_widget)
        dock_editing_layout.setContentsMargins(0, 0, 0, 0)
        
        self.save_dock_btn = QPushButton("Zapisz Dokowanie")
        self.cancel_dock_btn = QPushButton("Anuluj")
        
        dock_editing_layout.addWidget(self.save_dock_btn)
        dock_editing_layout.addWidget(self.cancel_dock_btn)
        
        map_controls_layout.addWidget(self.dock_editing_widget)
        self.dock_editing_widget.setVisible(False)  # Initially hidden
        
        
        right_layout.addLayout(map_controls_layout)
        
        # Create horizontal layout for map and side panels
        content_layout = QHBoxLayout()
        
        # Map section (left side)
        map_section = QWidget()
        map_section_layout = QVBoxLayout(map_section)
        map_section_layout.setContentsMargins(2, 2, 2, 2)  # Reduced margins
        map_section_layout.addWidget(self.map_view)
        content_layout.addWidget(map_section, 2)  # Give map more space (weight 2)
        
        # Routes and Docks section (right side)
        routes_docks_section = self.create_routes_docks_section()
        content_layout.addWidget(routes_docks_section, 1)  # Less space than map (weight 1)
        
        right_layout.addLayout(content_layout)
        
        return right_widget
    
    def create_routes_docks_section(self) -> QWidget:
        """Create the Routes and Docks section for the right side"""
        section_widget = QWidget()
        section_layout = QVBoxLayout(section_widget)
        
        # Routes Section
        routes_group = QGroupBox("Trasy")
        routes_layout = QVBoxLayout(routes_group)
        
        # Routes list
        self.routes_list = QListWidget()
        self.routes_list.setMaximumHeight(120)  # Reduced height to save space
        routes_layout.addWidget(self.routes_list)
        
        # Route control buttons
        route_buttons_layout = QHBoxLayout()
        self.add_route_editor_btn = QPushButton("Dodaj Trasę")
        self.edit_route_btn = QPushButton("Edytuj Trasę")
        self.remove_route_btn = QPushButton("Usuń Trasę")
        
        route_buttons_layout.addWidget(self.add_route_editor_btn)
        route_buttons_layout.addWidget(self.edit_route_btn)
        route_buttons_layout.addWidget(self.remove_route_btn)
        routes_layout.addLayout(route_buttons_layout)
        
        section_layout.addWidget(routes_group)
        
        # Docks Section
        docks_group = QGroupBox("Dokowania")
        docks_layout = QVBoxLayout(docks_group)
        
        # Docks list
        self.docks_list = QListWidget()
        self.docks_list.setMaximumHeight(120)  # Reduced height to save space
        docks_layout.addWidget(self.docks_list)
        
        # Dock control buttons
        dock_buttons_layout = QHBoxLayout()
        self.add_dock_editor_btn = QPushButton("Dodaj Dokowanie")
        self.edit_dock_editor_btn = QPushButton("Edytuj Dokowanie")
        self.remove_dock_editor_btn = QPushButton("Usuń Dokowanie")
        
        dock_buttons_layout.addWidget(self.add_dock_editor_btn)
        dock_buttons_layout.addWidget(self.edit_dock_editor_btn)
        dock_buttons_layout.addWidget(self.remove_dock_editor_btn)
        docks_layout.addLayout(dock_buttons_layout)
        
        section_layout.addWidget(docks_group)
        
        # Edit Speed Zones button
        self.edit_speed_zones_btn = QPushButton("Edytuj Strefy Prędkości")
        self.edit_speed_zones_btn.setStyleSheet("""
            QPushButton {
                font-weight: bold;
                font-size: 10px;
                padding: 6px 12px;
                background-color: #3498db;
                color: white;
                border: 1px solid #2980b9;
                border-radius: 4px;
                min-height: 20px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #21618c;
            }
        """)
        section_layout.addWidget(self.edit_speed_zones_btn)
        
        # Save plan and exit button
        self.save_plan_btn = QPushButton("Zapisz plan i wyjść")
        self.save_plan_btn.setStyleSheet("""
            QPushButton {
                font-weight: bold;
                font-size: 12px;
                padding: 8px 16px;
                background-color: #27ae60;
                color: white;
                border: 1px solid #2ecc71;
                border-radius: 4px;
                min-height: 24px;
            }
            QPushButton:hover {
                background-color: #2ecc71;
            }
            QPushButton:pressed {
                background-color: #229954;
            }
        """)
        section_layout.addWidget(self.save_plan_btn)
        
        # Add stretch to push everything to top except save button
        section_layout.addStretch()
        
        return section_widget
    
    def setup_connections(self):
        # Plan management connections
        self.plan_list.itemClicked.connect(self.on_plan_selected)
        self.new_plan_btn.clicked.connect(self.create_new_plan)
        self.delete_plan_btn.clicked.connect(self.delete_current_plan)
        self.duplicate_plan_btn.clicked.connect(self.duplicate_current_plan)
        self.save_plan_btn.clicked.connect(self.save_current_plan)
        
        # Map connections
        self.load_map_btn.clicked.connect(self.load_selected_map)
        
        # Action management connections
        self.add_route_btn.clicked.connect(self.add_route_action)
        self.add_dock_btn.clicked.connect(self.add_dock_action)
        self.add_undock_btn.clicked.connect(self.add_undock_action)
        self.add_wait_btn.clicked.connect(self.add_wait_action)
        # Note: add_stop_btn connection removed
        self.remove_action_btn.clicked.connect(self.remove_selected_action)
        self.move_up_btn.clicked.connect(self.move_action_up)
        self.move_down_btn.clicked.connect(self.move_action_down)
        
        # Route editor connections
        self.clear_route_btn.clicked.connect(self.clear_current_route)
        self.save_route_btn.clicked.connect(self.save_current_route)
        self.cancel_route_btn.clicked.connect(self.cancel_route_creation)
        
        # Dock editor connections
        self.save_dock_btn.clicked.connect(self.save_current_dock)
        self.cancel_dock_btn.clicked.connect(self.cancel_dock_creation)
        
        # Route management connections
        self.add_route_editor_btn.clicked.connect(self.start_route_creation)
        self.edit_route_btn.clicked.connect(self.edit_selected_route)
        self.remove_route_btn.clicked.connect(self.remove_selected_route)
        self.routes_list.itemClicked.connect(self.on_route_selected)
        
        # Dock management connections
        self.add_dock_editor_btn.clicked.connect(self.start_dock_creation)
        self.edit_dock_editor_btn.clicked.connect(self.edit_selected_dock)
        self.remove_dock_editor_btn.clicked.connect(self.remove_selected_dock)
        self.docks_list.itemClicked.connect(self.on_dock_selected)
        
        # Detail change connections
        self.plan_name_edit.textChanged.connect(self.on_plan_details_changed)
        self.map_combo.currentTextChanged.connect(self.on_map_changed)
        
        # Map view connections
        self.map_view.dock_placed.connect(self.on_dock_placed)
        
        # Speed zone editor connections
        self.edit_speed_zones_btn.clicked.connect(self.start_speed_zone_editing)
    
    def refresh_plan_list(self):
        self.plan_list.clear()
        for plan_name in self.plan_manager.get_plan_names():
            self.plan_list.addItem(plan_name)
    
    def refresh_map_list(self):
        self.map_combo.clear()
        maps = self.route_manager.get_map_names()
        self.map_combo.addItems(maps)
    
    def refresh_routes_list(self):
        """Refresh the routes list for the current map"""
        self.routes_list.clear()
        if self.route_manager.current_map:
            routes = self.route_manager.load_routes()
            for route_name in routes.keys():
                self.routes_list.addItem(route_name)
    
    def on_plan_selected(self, item: QListWidgetItem):
        plan_name = item.text()
        print(f"DEBUG: Plan selected: {plan_name}")
        self.current_plan = self.plan_manager.get_plan(plan_name)
        if self.current_plan:
            print(f"DEBUG: Current plan map_name: '{self.current_plan.map_name}'")
            self.load_plan_details()
            self.refresh_actions_list()
            self.plan_selected.emit(plan_name)
        else:
            print(f"DEBUG: Failed to get plan: {plan_name}")
    
    def load_plan_details(self):
        if not self.current_plan:
            print(f"DEBUG: load_plan_details - no current plan")
            return
        
        print(f"DEBUG: load_plan_details for plan '{self.current_plan.name}' with map '{self.current_plan.map_name}'")
        
        self.plan_name_edit.setText(self.current_plan.name)
        
        # Refresh maps and select current
        self.refresh_map_list()
        if self.current_plan.map_name:
            print(f"DEBUG: Plan has assigned map: '{self.current_plan.map_name}'")
            index = self.map_combo.findText(self.current_plan.map_name)
            print(f"DEBUG: Map combo index for '{self.current_plan.map_name}': {index}")
            if index >= 0:
                # Temporarily disconnect signal to avoid triggering on_map_changed
                try:
                    self.map_combo.currentTextChanged.disconnect(self.on_map_changed)
                except TypeError:
                    # Signal might not be connected yet
                    pass
                self.map_combo.setCurrentIndex(index)
                # Reconnect signal
                self.map_combo.currentTextChanged.connect(self.on_map_changed)
                
                # Load routes and docks for this map
                self.route_manager.set_current_map(self.current_plan.map_name)
                self.dock_manager.set_current_map(self.current_plan.map_name)
                self.refresh_routes_list()
                self.refresh_docks_list()
                # Auto-load the map assigned to this plan
                self.load_selected_map(show_success_dialog=False)
                print(f"DEBUG: Auto-loaded map '{self.current_plan.map_name}' for plan '{self.current_plan.name}'")
            else:
                print(f"DEBUG: Map '{self.current_plan.map_name}' not found in combo box")
        else:
            print(f"DEBUG: Plan '{self.current_plan.name}' has no assigned map")
    
    def refresh_actions_list(self):
        self.actions_list.clear()
        if not self.current_plan:
            return
        
        for i, action in enumerate(self.current_plan.actions):
            action_name = action.name
            # Add (rev) for reversed route actions
            if action.action_type == ActionType.ROUTE and action.parameters.get('reverse', False):
                action_name = f"{action.name} (rev)"
            
            item_text = f"{i+1}. {action.action_type.value.replace('_', ' ').title()}: {action_name}"
            self.actions_list.addItem(item_text)
    
    def create_new_plan(self):
        name, ok = QInputDialog.getText(self, 'New Plan', 'Enter plan name:')
        if ok and name:
            if name in self.plan_manager.plans:
                QMessageBox.warning(self, "Błąd", f"Plan '{name}' już istnieje!")
                return
            
            new_plan = ExecutionPlan(name=name)
            if self.plan_manager.add_plan(new_plan):
                self.refresh_plan_list()
                # Select the new plan
                items = self.plan_list.findItems(name, Qt.MatchExactly)
                if items:
                    self.plan_list.setCurrentItem(items[0])
                    self.on_plan_selected(items[0])
    
    def delete_current_plan(self):
        if not self.current_plan:
            return
        
        reply = QMessageBox.question(self, 'Usuń Plan', 
                                   f'Czy na pewno chcesz usunąć plan "{self.current_plan.name}"?',
                                   QMessageBox.Yes | QMessageBox.No, 
                                   QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            if self.plan_manager.remove_plan(self.current_plan.name):
                self.current_plan = None
                self.refresh_plan_list()
                self.clear_plan_details()
    
    def duplicate_current_plan(self):
        if not self.current_plan:
            return
        
        name, ok = QInputDialog.getText(self, 'Duplicate Plan', 
                                      f'Enter name for copy of "{self.current_plan.name}":',
                                      text=f"{self.current_plan.name}_copy")
        if ok and name:
            if name in self.plan_manager.plans:
                QMessageBox.warning(self, "Błąd", f"Plan '{name}' już istnieje!")
                return
            
            # Create a copy
            new_plan = ExecutionPlan(
                name=name,
                map_name=self.current_plan.map_name,
                actions=self.current_plan.actions.copy()
            )
            
            if self.plan_manager.add_plan(new_plan):
                self.refresh_plan_list()
                # Select the new plan
                items = self.plan_list.findItems(name, Qt.MatchExactly)
                if items:
                    self.plan_list.setCurrentItem(items[0])
                    self.on_plan_selected(items[0])
    
    def save_current_plan(self):
        if not self.current_plan:
            return
        
        # Update plan with current details
        self.current_plan.name = self.plan_name_edit.text()
        self.current_plan.map_name = self.map_combo.currentText()
        
        if self.plan_manager.update_plan(self.current_plan):
            self.refresh_plan_list()
            self.plan_updated.emit()
            QMessageBox.information(self, "Sukces", "Plan został pomyślnie zapisany!")
            self.close()  # Close the plan editor window after saving
    
    def clear_plan_details(self):
        self.plan_name_edit.clear()
        self.map_combo.setCurrentIndex(-1)
        self.actions_list.clear()
    
    def on_plan_details_changed(self):
        # Enable save button when details change
        self.save_plan_btn.setEnabled(True)
    
    def on_map_changed(self):
        if self.current_plan:
            self.current_plan.map_name = self.map_combo.currentText()
            # Update route manager and dock manager, refresh lists
            if self.current_plan.map_name:
                self.route_manager.set_current_map(self.current_plan.map_name)
                self.dock_manager.set_current_map(self.current_plan.map_name)
                self.refresh_routes_list()
                self.refresh_docks_list()
            self.on_plan_details_changed()
    
    def load_selected_map(self, show_success_dialog=True):
        map_name = self.map_combo.currentText()
        print(f"DEBUG: load_selected_map called with map_name: '{map_name}', show_success_dialog: {show_success_dialog}")
        if not map_name:
            if show_success_dialog:
                QMessageBox.warning(self, "Ostrzeżenie", "Proszę najpierw wybrać mapę!")
            return
        
        # Set the current map in route and dock managers
        self.route_manager.set_current_map(map_name)
        self.dock_manager.set_current_map(map_name)
        
        # Load the map in the editor's map view
        from pathlib import Path
        import yaml
        import shutil
        
        maps_dir = Path.home() / ".robotroutes" / "maps"
        map_path = maps_dir / f"{map_name}.pgm"
        yaml_path = maps_dir / f"{map_name}.yaml"
        
        try:
            if not map_path.exists() or not yaml_path.exists():
                if show_success_dialog:
                    QMessageBox.warning(self, "Błąd", f"Pliki mapy '{map_name}' nie zostały znalezione!")
                return
            
            # Create speed_ prefixed copy if it doesn't exist
            speed_map_path = maps_dir / f"speed_{map_name}.pgm"
            speed_yaml_path = maps_dir / f"speed_{map_name}.yaml"
            
            if not speed_map_path.exists():
                shutil.copy2(map_path, speed_map_path)
                print(f"DEBUG: Created speed map copy: {speed_map_path}")
            
            if not speed_yaml_path.exists():
                shutil.copy2(yaml_path, speed_yaml_path)
                print(f"DEBUG: Created speed yaml copy: {speed_yaml_path}")
            
            with open(yaml_path, 'r') as file:
                yaml_data = yaml.safe_load(file)
            
            # Always load original map in plan view (not speed_ version)
            self.map_view.load_image(str(map_path), yaml_data['origin'], yaml_data.get('resolution'))
            # Initialize dock graphics after map is loaded
            self.map_view.init_dock_graphics(self.dock_manager)
            # Refresh routes and docks lists after map is loaded
            self.refresh_routes_list()
            self.refresh_docks_list()
            
            # Assign this map to the current plan
            if self.current_plan:
                if self.current_plan.map_name != map_name:
                    old_map = self.current_plan.map_name
                    self.current_plan.map_name = map_name
                    print(f"DEBUG: Assigned map '{map_name}' to plan '{self.current_plan.name}' (was: '{old_map}')")
                    # Save the plan to persist the map assignment
                    success = self.plan_manager.update_plan(self.current_plan)
                    print(f"DEBUG: Plan update success: {success}")
                else:
                    print(f"DEBUG: Map '{map_name}' already assigned to plan '{self.current_plan.name}'")
            
            if show_success_dialog:
                QMessageBox.information(self, "Sukces", f"Mapa '{map_name}' została załadowana pomyślnie!")
            
        except Exception as e:
            if show_success_dialog:
                QMessageBox.critical(self, "Błąd", f"Nie udało się załadować mapy: {str(e)}")
    
    # Action management methods
    def add_route_action(self):
        if not self.current_plan:
            return
        
        routes = self.route_manager.load_routes()
        if not routes:
            QMessageBox.warning(self, "Ostrzeżenie", "Brak dostępnych tras dla bieżącej mapy!")
            return
        
        route_names = list(routes.keys())
        dialog = RouteSelectionDialog(route_names, self)
        if dialog.exec_() == QDialog.Accepted:
            route_name, reverse = dialog.get_selection()
            action = self.plan_manager.create_route_action(route_name, reverse)
            self.current_plan.add_action(action)
            self.refresh_actions_list()
            self.on_plan_details_changed()
    
    def add_dock_action(self):
        if not self.current_plan:
            return
        
        # Get available docks
        docks = self.dock_manager.load_docks()
        if not docks:
            QMessageBox.warning(self, "Ostrzeżenie", "Brak dostępnych dokowań dla bieżącej mapy!")
            return
        
        dock_names = list(docks.keys())
        dialog = DockSelectionDialog(dock_names, self)
        if dialog.exec_() == QDialog.Accepted:
            dock_name = dialog.get_selection()
            action = self.plan_manager.create_dock_action(dock_name)
            self.current_plan.add_action(action)
            self.refresh_actions_list()
            self.on_plan_details_changed()
    
    def add_undock_action(self):
        if not self.current_plan:
            return
        
        action = self.plan_manager.create_undock_action()
        self.current_plan.add_action(action)
        self.refresh_actions_list()
        self.on_plan_details_changed()
    
    def add_wait_action(self):
        if not self.current_plan:
            return
        
        dialog = WaitActionDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            action = self.plan_manager.create_wait_signal_action(dialog.signal_name)
            
            # No additional parameters needed for CAN signal (any message on ID 0x69 triggers)
            
            self.current_plan.add_action(action)
            self.refresh_actions_list()
            self.on_plan_details_changed()
    
    # Note: add_stop_action method removed
    
    def remove_selected_action(self):
        if not self.current_plan:
            return
        
        current_row = self.actions_list.currentRow()
        if current_row >= 0:
            self.current_plan.remove_action(current_row)
            self.refresh_actions_list()
            self.on_plan_details_changed()
    
    def move_action_up(self):
        if not self.current_plan:
            return
        
        current_row = self.actions_list.currentRow()
        if current_row > 0:
            # Swap actions
            actions = self.current_plan.actions
            actions[current_row], actions[current_row-1] = actions[current_row-1], actions[current_row]
            self.refresh_actions_list()
            self.actions_list.setCurrentRow(current_row-1)
            self.on_plan_details_changed()
    
    def move_action_down(self):
        if not self.current_plan:
            return
        
        current_row = self.actions_list.currentRow()
        if current_row >= 0 and current_row < len(self.current_plan.actions) - 1:
            # Swap actions
            actions = self.current_plan.actions
            actions[current_row], actions[current_row+1] = actions[current_row+1], actions[current_row]
            self.refresh_actions_list()
            self.actions_list.setCurrentRow(current_row+1)
            self.on_plan_details_changed()
    
    # Route editor methods
    def clear_current_route(self):
        self.map_view.clear_route()
    
    def save_current_route(self):
        current_route = self.map_view.get_current_route()
        if not current_route or len(current_route.nodes) == 0:
            QMessageBox.warning(self, "Ostrzeżenie", "Brak trasy do zapisania!")
            return
        
        # Check if we're editing an existing route
        if self.editing_route_name:
            # Editing existing route - ask for confirmation
            reply = QMessageBox.question(self, 'Aktualizuj Trasę', 
                                       f"Aktualizować istniejącą trasę '{self.editing_route_name}'?",
                                       QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                name = self.editing_route_name
                if self.route_manager.update_route(name, current_route):
                    self.refresh_routes_list()
                    # Exit route editing mode
                    self.map_view.stop_route_editing()
                    self.route_editing_widget.setVisible(False)
                    self.editing_route_name = None
                    QMessageBox.information(self, "Sukces", f"Trasa '{name}' została pomyślnie zaktualizowana!")
                else:
                    QMessageBox.warning(self, "Błąd", f"Nie udało się zaktualizować trasy '{name}'!")
        else:
            # Creating new route - ask for name
            name, ok = QInputDialog.getText(self, 'Zapisz Trasę', 'Wprowadź nazwę trasy:')
            if ok and name:
                if self.route_manager.add_route(name, current_route):
                    self.refresh_routes_list()  # Refresh the routes list
                    # Exit route editing mode
                    self.map_view.stop_route_editing()
                    self.route_editing_widget.setVisible(False)
                    QMessageBox.information(self, "Sukces", f"Trasa '{name}' została pomyślnie zapisana!")
                else:
                    QMessageBox.warning(self, "Błąd", f"Nie udało się zapisać trasy '{name}'!")
    
    # Route management methods
    def start_route_creation(self):
        """Start creating a new route"""
        if not self.route_manager.current_map:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę najpierw wybrać i załadować mapę!")
            return
        
        # Clear editing state (we're creating a new route)
        self.editing_route_name = None
        
        # Clear any existing route and enable route editing
        self.map_view.clear_route()
        self.map_view.start_route_editing()
        
        # Show route editing controls
        self.route_editing_widget.setVisible(True)
    
    def edit_selected_route(self):
        """Start editing the selected route"""
        current_item = self.routes_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę wybrać trasę do edycji!")
            return
        
        if not self.route_manager.current_map:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę najpierw wybrać i załadować mapę!")
            return
        
        route_name = current_item.text()
        routes = self.route_manager.load_routes()
        
        if route_name not in routes:
            QMessageBox.warning(self, "Ostrzeżenie", "Wybrana trasa nie została znaleziona!")
            return
        
        # Store the route we're editing
        self.editing_route_name = route_name
        
        # Load the route for editing
        route = routes[route_name]
        self.map_view.clear_route()
        self.map_view.start_route_editing(route)
        
        # Show route editing controls
        self.route_editing_widget.setVisible(True)
    
    def cancel_route_creation(self):
        """Cancel current route creation"""
        self.map_view.stop_route_editing()
        self.map_view.clear_route()
        
        # Clear editing state
        self.editing_route_name = None
        
        # Hide route editing controls
        self.route_editing_widget.setVisible(False)
    
    def remove_selected_route(self):
        """Remove the selected route from the routes list"""
        current_item = self.routes_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę wybrać trasę do usunięcia!")
            return
        
        route_name = current_item.text()
        reply = QMessageBox.question(self, 'Remove Route', 
                                   f'Are you sure you want to remove route "{route_name}"?',
                                   QMessageBox.Yes | QMessageBox.No, 
                                   QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # Load current routes, remove the selected one, and save
            routes = self.route_manager.load_routes()
            if route_name in routes:
                del routes[route_name]
                if self.route_manager.save_routes(routes):
                    self.refresh_routes_list()
                    self.map_view.clear_route()  # Clear route display
                    QMessageBox.information(self, "Sukces", f"Trasa '{route_name}' została pomyślnie usunięta!")
                else:
                    QMessageBox.warning(self, "Błąd", f"Nie udało się usunąć trasy '{route_name}'!")
    
    def on_route_selected(self, item: QListWidgetItem):
        """Handle route selection from list"""
        # Display the selected route on the map
        route_name = item.text()
        routes = self.route_manager.load_routes()
        
        if route_name in routes:
            route = routes[route_name]
            # Exit editing mode temporarily to view the route
            was_editing = hasattr(self.map_view, 'editing_mode') and self.map_view.editing_mode
            if was_editing:
                self.map_view.stop_route_editing()
                self.route_editing_widget.setVisible(False)
                self.editing_route_name = None
            
            # Hide all docks when showing a route
            self.map_view.hide_all_docks()
            
            # Display the route
            self.map_view.display_bezier_route(route, force_update=True)
    
    # Dock management methods
    def refresh_docks_list(self):
        """Refresh the docks list display"""
        self.docks_list.clear()
        if not self.dock_manager.current_map:
            return
        
        docks = self.dock_manager.load_docks()
        for dock_name in sorted(docks.keys()):
            self.docks_list.addItem(dock_name)
    
    def start_dock_creation(self):
        """Start creating a new dock by clicking on the map"""
        if not self.dock_manager.current_map:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę najpierw wybrać i załadować mapę!")
            return
        
        # Clear editing state (we're creating a new dock)
        self.editing_dock_name = None
        
        # Get dock name from user
        name, ok = QInputDialog.getText(self, 'Create Dock', 'Enter dock name:')
        if not ok or not name:
            return
        
        # Check if dock name already exists
        docks = self.dock_manager.load_docks()
        if name in docks:
            QMessageBox.warning(self, "Ostrzeżenie", f"Dokowanie '{name}' już istnieje!")
            return
        
        # Set up for new dock creation
        self.editing_dock_name = name
        
        # Hide route editing controls if visible
        self.route_editing_widget.setVisible(False)
        
        # Enable dock placement mode
        self.map_view.start_dock_placement(name)
        
        # Show dock editing controls for new dock
        self.dock_editing_widget.setVisible(True)
        
        QMessageBox.information(self, "Umieść Dokowanie", 
                               f"Kliknij na mapę, aby umieścić dokowanie '{name}', następnie kliknij Zapisz lub Anuluj")
    
    def edit_selected_dock(self):
        """Start editing the selected dock"""
        current_item = self.docks_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę wybrać dokowanie do edycji!")
            return
        
        dock_name = current_item.text()
        docks = self.dock_manager.load_docks()
        
        if dock_name not in docks:
            QMessageBox.warning(self, "Ostrzeżenie", "Wybrane dokowanie nie zostało znalezione!")
            return
        
        # Store the dock we're editing
        self.editing_dock_name = dock_name
        
        # Hide route editing controls if visible
        self.route_editing_widget.setVisible(False)
        
        # Start dock editing mode in map view
        self.map_view.start_dock_editing(dock_name)
        
        # Show dock editing controls
        self.dock_editing_widget.setVisible(True)
        
        QMessageBox.information(self, "Edytuj Dokowanie", 
                               f"Przeciągnij dokowanie '{dock_name}', aby je przesunąć, następnie kliknij Zapisz lub Anuluj")
    
    def remove_selected_dock(self):
        """Remove the selected dock from the docks list"""
        current_item = self.docks_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę wybrać dokowanie do usunięcia!")
            return
        
        dock_name = current_item.text()
        
        # Confirm deletion
        reply = QMessageBox.question(self, 'Remove Dock', 
                                   f"Are you sure you want to remove dock '{dock_name}'?",
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            if self.dock_manager.remove_dock(dock_name):
                self.refresh_docks_list()
                # Force remove any stuck dock graphics
                if self.map_view.dock_graphics_manager:
                    self.map_view.dock_graphics_manager.force_remove_dock_graphics(dock_name)
                # Update dock graphics display
                self.map_view.update_dock_graphics()
                QMessageBox.information(self, "Sukces", f"Dokowanie '{dock_name}' zostało pomyślnie usunięte!")
            else:
                QMessageBox.warning(self, "Błąd", f"Nie udało się usunąć dokowania '{dock_name}'!")
    
    def on_dock_selected(self, item: QListWidgetItem):
        """Handle dock selection from list"""
        # Display the selected dock on the map
        dock_name = item.text()
        docks = self.dock_manager.load_docks()
        
        if dock_name in docks:
            dock = docks[dock_name]
            
            # Hide routes when showing a dock
            self.map_view.clear_route()
            
            # Hide all docks first, then show only the selected one
            self.map_view.hide_all_docks()
            self.map_view.show_dock(dock_name)
            
            print(f"DEBUG: Selected dock '{dock_name}' at ({dock.x}, {dock.y})")
    
    def on_dock_placed(self, dock_name: str):
        """Handle when a dock is successfully placed on the map"""
        # Note: For new editing system, we don't refresh the list until saved
        # The dock is now temporary and needs to be saved
        print(f"DEBUG: Dock '{dock_name}' placed temporarily, waiting for save/cancel")
    
    def save_current_dock(self):
        """Save the current dock position"""
        if not self.editing_dock_name:
            QMessageBox.warning(self, "Ostrzeżenie", "Nie edytuje się żadnego dokowania!")
            return
        
        if self.map_view.save_current_dock():
            # Exit dock editing mode
            self.map_view.stop_dock_editing()
            self.dock_editing_widget.setVisible(False)
            self.editing_dock_name = None
            
            # Refresh the dock list
            self.refresh_docks_list()
            QMessageBox.information(self, "Sukces", "Pozycja dokowania została pomyślnie zapisana!")
        else:
            QMessageBox.warning(self, "Błąd", "Nie udało się zapisać pozycji dokowania!")
    
    def cancel_dock_creation(self):
        """Cancel current dock editing"""
        if self.editing_dock_name:
            # Cancel dock editing and revert position
            self.map_view.cancel_dock_editing()
            self.dock_editing_widget.setVisible(False)
            self.editing_dock_name = None
            QMessageBox.information(self, "Anulowano", "Edycja dokowania została anulowana.")
    
    # Speed zone editing methods
    def start_speed_zone_editing(self):
        """Start speed zone editing mode with popup window"""
        print(f"DEBUG: start_speed_zone_editing called")
        map_name = self.map_combo.currentText()
        print(f"DEBUG: Current map name: {map_name}")
        if not map_name:
            QMessageBox.warning(self, "Ostrzeżenie", "Proszę najpierw wybrać i załadować mapę!")
            return
            
        # Create speed zone editor if it doesn't exist
        if not self.speed_zone_editor:
            self.speed_zone_editor = SpeedZoneEditor()
            # Connect signals
            self.speed_zone_editor.speed_zone_updated.connect(self.on_speed_zone_updated)
            self.speed_zone_editor.editing_finished.connect(self.stop_speed_zone_editing)
            print(f"DEBUG: Connected speed zone editor signals")
            
        # Ensure speed_ prefixed maps exist before loading in editor
        from pathlib import Path
        import shutil
        maps_dir = Path.home() / ".robotroutes" / "maps"
        map_path = maps_dir / f"{map_name}.pgm"
        yaml_path = maps_dir / f"{map_name}.yaml"
        speed_map_path = maps_dir / f"speed_{map_name}.pgm"
        speed_yaml_path = maps_dir / f"speed_{map_name}.yaml"
        
        # Create speed_ prefixed copies if they don't exist
        if not speed_map_path.exists() and map_path.exists():
            shutil.copy2(map_path, speed_map_path)
            print(f"DEBUG: Created speed map copy: {speed_map_path}")
        
        if not speed_yaml_path.exists() and yaml_path.exists():
            shutil.copy2(yaml_path, speed_yaml_path)
            print(f"DEBUG: Created speed yaml copy: {speed_yaml_path}")
        
        # Load the map in the speed zone editor
        print(f"DEBUG: Loading map in speed zone editor")
        if self.speed_zone_editor.load_map(map_name):
            print(f"DEBUG: Map loaded successfully, creating popup window")
            # Create popup window for speed zone editor
            if not self.speed_zone_window:
                print(f"DEBUG: Creating new speed zone window")
                from PyQt5.QtWidgets import QDialog, QVBoxLayout
                self.speed_zone_window = QDialog(self)
                self.speed_zone_window.setWindowTitle("Edytor Stref Prędkości")
                self.speed_zone_window.setModal(False)  # Non-modal so user can interact with map
                self.speed_zone_window.resize(300, 400)
                
                layout = QVBoxLayout(self.speed_zone_window)
                layout.addWidget(self.speed_zone_editor)
                
                # Handle window close event
                self.speed_zone_window.closeEvent = self.on_speed_zone_window_close
                
            # Load speed_ prefixed map in map view for editing
            import yaml
            speed_map_path = maps_dir / f"speed_{map_name}.pgm"
            speed_yaml_path = maps_dir / f"speed_{map_name}.yaml"
            
            try:
                with open(speed_yaml_path, 'r') as file:
                    yaml_data = yaml.safe_load(file)
                self.map_view.load_image(str(speed_map_path), yaml_data['origin'], yaml_data.get('resolution'))
                print(f"DEBUG: Loaded speed_ prefixed map for editing: {speed_map_path}")
            except Exception as e:
                print(f"Error loading speed map: {e}")
            
            # Start speed zone editing mode in map view
            self.map_view.start_speed_zone_editing(self.speed_zone_editor)
            
            # Hide other editing controls
            self.route_editing_widget.setVisible(False)
            self.dock_editing_widget.setVisible(False)
            
            # Update button state
            self.edit_speed_zones_btn.setEnabled(False)
            
            # Show the speed zone editor window
            print(f"DEBUG: Showing speed zone window")
            self.speed_zone_window.show()
            print(f"DEBUG: Speed zone window shown")
            
        else:
            print(f"DEBUG: Failed to load map in speed zone editor")
            QMessageBox.warning(self, "Błąd", "Nie udało się załadować mapy dla edycji stref prędkości!")
    
    def stop_speed_zone_editing(self):
        """Stop speed zone editing mode"""
        # Stop speed zone editing mode in map view first
        self.map_view.stop_speed_zone_editing()
        
        # Clean up speed zone editor state (removes working files)
        if self.speed_zone_editor:
            self.speed_zone_editor.cleanup_editing()
        
        # Hide the speed zone editor window
        if self.speed_zone_window:
            self.speed_zone_window.hide()
            
        # Update button state
        self.edit_speed_zones_btn.setEnabled(True)
        
        # Reload original map to show clean version without speed zones
        self.load_selected_map(show_success_dialog=False)
    
    def on_speed_zone_window_close(self, event):
        """Handle speed zone window close event"""
        self.stop_speed_zone_editing()
        event.accept()
    
    def on_speed_zone_updated(self):
        """Handle speed zone updates during editing"""
        print(f"DEBUG: on_speed_zone_updated called")
        # Reload speed_ prefixed map to show updated speed zones
        if self.speed_zone_editor and self.speed_zone_editor.map_name:
            from pathlib import Path
            map_name = self.speed_zone_editor.map_name
            maps_dir = Path.home() / ".robotroutes" / "maps"
            speed_map_path = maps_dir / f"speed_{map_name}.pgm"
            speed_yaml_path = maps_dir / f"speed_{map_name}.yaml"
            
            print(f"DEBUG: Reloading speed map: {speed_map_path}")
            try:
                import yaml
                
                with open(speed_yaml_path, 'r') as file:
                    yaml_data = yaml.safe_load(file)
                self.map_view.load_image(str(speed_map_path), yaml_data['origin'], yaml_data.get('resolution'))
                print(f"DEBUG: Successfully reloaded speed map")
            except Exception as e:
                print(f"Error reloading speed map: {e}")
        else:
            print(f"DEBUG: No map name available for reload")
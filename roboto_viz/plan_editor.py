from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QListWidget, QListWidgetItem, QLabel, 
                             QLineEdit, QComboBox, QTextEdit, QSplitter, QGroupBox,
                             QMessageBox, QInputDialog, QGridLayout, QFrame, QCheckBox, QDialog)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from typing import Optional

from roboto_viz.plan_manager import PlanManager, ExecutionPlan, PlanAction, ActionType
from roboto_viz.route_manager import RouteManager, BezierRoute
from roboto_viz.map_view import MapView


class RouteSelectionDialog(QDialog):
    """Custom dialog for selecting a route with optional reverse checkbox"""
    
    def __init__(self, route_names, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add Route Action")
        self.setModal(True)
        self.resize(300, 150)
        
        self.route_name = None
        self.reverse = False
        
        layout = QVBoxLayout()
        
        # Route selection
        route_label = QLabel("Select route:")
        layout.addWidget(route_label)
        
        self.route_combo = QComboBox()
        self.route_combo.addItems(route_names)
        layout.addWidget(self.route_combo)
        
        # Reverse checkbox
        self.reverse_checkbox = QCheckBox("Run route in reverse")
        layout.addWidget(self.reverse_checkbox)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        ok_button = QPushButton("OK")
        ok_button.clicked.connect(self.accept)
        button_layout.addWidget(ok_button)
        
        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def get_selection(self):
        """Get the selected route name and reverse state"""
        return self.route_combo.currentText(), self.reverse_checkbox.isChecked()


class PlanEditor(QMainWindow):
    plan_selected = pyqtSignal(str)  # plan_name
    plan_updated = pyqtSignal()
    
    def __init__(self, plan_manager: PlanManager, route_manager: RouteManager):
        super().__init__()
        self.plan_manager = plan_manager
        self.route_manager = route_manager
        self.current_plan: Optional[ExecutionPlan] = None
        self.editing_route_name: Optional[str] = None  # Track if we're editing an existing route
        
        # Create a new map view for the editor
        self.map_view = MapView()
        
        self.setWindowTitle("Plan Editor")
        self.setGeometry(100, 100, 1200, 800)
        
        self.setup_ui()
        self.setup_connections()
        self.refresh_plan_list()
        self.refresh_routes_list()
    
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
        
        # Set initial splitter proportions
        splitter.setSizes([300, 900])
    
    def create_left_panel(self) -> QWidget:
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # Plan List Section
        plan_group = QGroupBox("Plans")
        plan_layout = QVBoxLayout(plan_group)
        
        # Plan list
        self.plan_list = QListWidget()
        plan_layout.addWidget(self.plan_list)
        
        # Plan control buttons
        plan_buttons_layout = QHBoxLayout()
        self.new_plan_btn = QPushButton("New Plan")
        self.delete_plan_btn = QPushButton("Delete Plan")
        self.duplicate_plan_btn = QPushButton("Duplicate")
        
        plan_buttons_layout.addWidget(self.new_plan_btn)
        plan_buttons_layout.addWidget(self.delete_plan_btn)
        plan_buttons_layout.addWidget(self.duplicate_plan_btn)
        plan_layout.addLayout(plan_buttons_layout)
        
        left_layout.addWidget(plan_group)
        
        # Plan Details Section
        details_group = QGroupBox("Plan Details")
        details_layout = QVBoxLayout(details_group)
        
        # Plan name
        details_layout.addWidget(QLabel("Plan Name:"))
        self.plan_name_edit = QLineEdit()
        details_layout.addWidget(self.plan_name_edit)
        
        # Plan description
        details_layout.addWidget(QLabel("Description:"))
        self.plan_description_edit = QTextEdit()
        self.plan_description_edit.setMaximumHeight(80)
        details_layout.addWidget(self.plan_description_edit)
        
        # Map selection
        details_layout.addWidget(QLabel("Map:"))
        self.map_combo = QComboBox()
        details_layout.addWidget(self.map_combo)
        
        # Load map button
        self.load_map_btn = QPushButton("Load Map")
        details_layout.addWidget(self.load_map_btn)
        
        # Save plan button
        self.save_plan_btn = QPushButton("Save Plan")
        details_layout.addWidget(self.save_plan_btn)
        
        left_layout.addWidget(details_group)
        
        # Actions Section
        actions_group = QGroupBox("Plan Actions")
        actions_layout = QVBoxLayout(actions_group)
        
        # Actions list
        self.actions_list = QListWidget()
        actions_layout.addWidget(self.actions_list)
        
        # Action control buttons
        action_buttons_layout = QGridLayout()
        
        self.add_route_btn = QPushButton("Add Route")
        self.add_dock_btn = QPushButton("Add Dock")
        self.add_undock_btn = QPushButton("Add Undock")
        self.add_wait_btn = QPushButton("Add Wait")
        # Note: Add Stop button removed
        self.remove_action_btn = QPushButton("Remove Action")
        self.move_up_btn = QPushButton("Move Up")
        self.move_down_btn = QPushButton("Move Down")
        
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
        
        # Routes Section
        routes_group = QGroupBox("Routes")
        routes_layout = QVBoxLayout(routes_group)
        
        # Routes list
        self.routes_list = QListWidget()
        routes_layout.addWidget(self.routes_list)
        
        # Route control buttons
        route_buttons_layout = QHBoxLayout()
        self.add_route_editor_btn = QPushButton("Add Route")
        self.edit_route_btn = QPushButton("Edit Route")
        self.remove_route_btn = QPushButton("Remove Route")
        
        route_buttons_layout.addWidget(self.add_route_editor_btn)
        route_buttons_layout.addWidget(self.edit_route_btn)
        route_buttons_layout.addWidget(self.remove_route_btn)
        routes_layout.addLayout(route_buttons_layout)
        
        left_layout.addWidget(routes_group)
        
        # Add stretch to push everything to top
        left_layout.addStretch()
        
        return left_widget
    
    def create_right_panel(self) -> QWidget:
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Map controls
        map_controls_layout = QHBoxLayout()
        
        map_label = QLabel("Map View & Route Editor")
        map_label.setFont(QFont("Arial", 12, QFont.Bold))
        map_controls_layout.addWidget(map_label)
        
        map_controls_layout.addStretch()
        
        # Route editing controls (initially hidden)
        self.route_editing_widget = QWidget()
        route_editing_layout = QHBoxLayout(self.route_editing_widget)
        route_editing_layout.setContentsMargins(0, 0, 0, 0)
        
        self.clear_route_btn = QPushButton("Clear Route")
        self.save_route_btn = QPushButton("Save Route")
        self.cancel_route_btn = QPushButton("Cancel")
        
        route_editing_layout.addWidget(self.clear_route_btn)
        route_editing_layout.addWidget(self.save_route_btn)
        route_editing_layout.addWidget(self.cancel_route_btn)
        
        map_controls_layout.addWidget(self.route_editing_widget)
        self.route_editing_widget.setVisible(False)  # Initially hidden
        
        right_layout.addLayout(map_controls_layout)
        
        # Add map view
        right_layout.addWidget(self.map_view)
        
        return right_widget
    
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
        
        # Route management connections
        self.add_route_editor_btn.clicked.connect(self.start_route_creation)
        self.edit_route_btn.clicked.connect(self.edit_selected_route)
        self.remove_route_btn.clicked.connect(self.remove_selected_route)
        self.routes_list.itemClicked.connect(self.on_route_selected)
        
        # Detail change connections
        self.plan_name_edit.textChanged.connect(self.on_plan_details_changed)
        self.plan_description_edit.textChanged.connect(self.on_plan_details_changed)
        self.map_combo.currentTextChanged.connect(self.on_map_changed)
    
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
        self.current_plan = self.plan_manager.get_plan(plan_name)
        if self.current_plan:
            self.load_plan_details()
            self.refresh_actions_list()
            self.plan_selected.emit(plan_name)
    
    def load_plan_details(self):
        if not self.current_plan:
            return
        
        self.plan_name_edit.setText(self.current_plan.name)
        self.plan_description_edit.setPlainText(self.current_plan.description)
        
        # Refresh maps and select current
        self.refresh_map_list()
        if self.current_plan.map_name:
            index = self.map_combo.findText(self.current_plan.map_name)
            if index >= 0:
                self.map_combo.setCurrentIndex(index)
                # Load routes for this map
                self.route_manager.set_current_map(self.current_plan.map_name)
                self.refresh_routes_list()
                # Auto-load the map
                self.load_selected_map(show_success_dialog=False)
    
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
                QMessageBox.warning(self, "Error", f"Plan '{name}' already exists!")
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
        
        reply = QMessageBox.question(self, 'Delete Plan', 
                                   f'Are you sure you want to delete plan "{self.current_plan.name}"?',
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
                QMessageBox.warning(self, "Error", f"Plan '{name}' already exists!")
                return
            
            # Create a copy
            new_plan = ExecutionPlan(
                name=name,
                description=self.current_plan.description,
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
        self.current_plan.description = self.plan_description_edit.toPlainText()
        self.current_plan.map_name = self.map_combo.currentText()
        
        if self.plan_manager.update_plan(self.current_plan):
            self.refresh_plan_list()
            self.plan_updated.emit()
            QMessageBox.information(self, "Success", "Plan saved successfully!")
    
    def clear_plan_details(self):
        self.plan_name_edit.clear()
        self.plan_description_edit.clear()
        self.map_combo.setCurrentIndex(-1)
        self.actions_list.clear()
    
    def on_plan_details_changed(self):
        # Enable save button when details change
        self.save_plan_btn.setEnabled(True)
    
    def on_map_changed(self):
        if self.current_plan:
            self.current_plan.map_name = self.map_combo.currentText()
            # Update route manager and refresh routes list
            if self.current_plan.map_name:
                self.route_manager.set_current_map(self.current_plan.map_name)
                self.refresh_routes_list()
            self.on_plan_details_changed()
    
    def load_selected_map(self, show_success_dialog=True):
        map_name = self.map_combo.currentText()
        if not map_name:
            if show_success_dialog:
                QMessageBox.warning(self, "Warning", "Please select a map first!")
            return
        
        # Set the current map in route manager
        self.route_manager.set_current_map(map_name)
        
        # Load the map in the editor's map view
        from pathlib import Path
        import yaml
        
        maps_dir = Path.home() / ".robotroutes" / "maps"
        map_path = maps_dir / f"{map_name}.pgm"
        yaml_path = maps_dir / f"{map_name}.yaml"
        
        try:
            if not map_path.exists() or not yaml_path.exists():
                if show_success_dialog:
                    QMessageBox.warning(self, "Error", f"Map files not found for '{map_name}'!")
                return
            
            with open(yaml_path, 'r') as file:
                yaml_data = yaml.safe_load(file)
            
            self.map_view.load_image(str(map_path), yaml_data['origin'])
            # Refresh routes list after map is loaded
            self.refresh_routes_list()
            if show_success_dialog:
                QMessageBox.information(self, "Success", f"Map '{map_name}' loaded successfully!")
            
        except Exception as e:
            if show_success_dialog:
                QMessageBox.critical(self, "Error", f"Failed to load map: {str(e)}")
    
    # Action management methods
    def add_route_action(self):
        if not self.current_plan:
            return
        
        routes = self.route_manager.load_routes()
        if not routes:
            QMessageBox.warning(self, "Warning", "No routes available for current map!")
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
        
        action = self.plan_manager.create_dock_action()
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
        
        signal_name, ok = QInputDialog.getText(self, 'Add Wait Action', 
                                             'Enter signal name:', text='default')
        if ok:
            action = self.plan_manager.create_wait_signal_action(signal_name)
            self.current_plan.add_action(action)
            self.refresh_actions_list()
            self.on_plan_details_changed()
    
    # Note: add_stop_action method removed
            self.on_plan_details_changed()
    
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
            QMessageBox.warning(self, "Warning", "No route to save!")
            return
        
        # Check if we're editing an existing route
        if self.editing_route_name:
            # Editing existing route - ask for confirmation
            reply = QMessageBox.question(self, 'Update Route', 
                                       f"Update existing route '{self.editing_route_name}'?",
                                       QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                name = self.editing_route_name
                if self.route_manager.update_route(name, current_route):
                    self.refresh_routes_list()
                    # Exit route editing mode
                    self.map_view.stop_route_editing()
                    self.route_editing_widget.setVisible(False)
                    self.editing_route_name = None
                    QMessageBox.information(self, "Success", f"Route '{name}' updated successfully!")
                else:
                    QMessageBox.warning(self, "Error", f"Failed to update route '{name}'!")
        else:
            # Creating new route - ask for name
            name, ok = QInputDialog.getText(self, 'Save Route', 'Enter route name:')
            if ok and name:
                if self.route_manager.add_route(name, current_route):
                    self.refresh_routes_list()  # Refresh the routes list
                    # Exit route editing mode
                    self.map_view.stop_route_editing()
                    self.route_editing_widget.setVisible(False)
                    QMessageBox.information(self, "Success", f"Route '{name}' saved successfully!")
                else:
                    QMessageBox.warning(self, "Error", f"Failed to save route '{name}'!")
    
    # Route management methods
    def start_route_creation(self):
        """Start creating a new route"""
        if not self.route_manager.current_map:
            QMessageBox.warning(self, "Warning", "Please select and load a map first!")
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
            QMessageBox.warning(self, "Warning", "Please select a route to edit!")
            return
        
        if not self.route_manager.current_map:
            QMessageBox.warning(self, "Warning", "Please select and load a map first!")
            return
        
        route_name = current_item.text()
        routes = self.route_manager.load_routes()
        
        if route_name not in routes:
            QMessageBox.warning(self, "Warning", "Selected route not found!")
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
            QMessageBox.warning(self, "Warning", "Please select a route to remove!")
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
                    QMessageBox.information(self, "Success", f"Route '{route_name}' removed successfully!")
                else:
                    QMessageBox.warning(self, "Error", f"Failed to remove route '{route_name}'!")
    
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
            
            # Display the route
            self.map_view.display_bezier_route(route, force_update=True)
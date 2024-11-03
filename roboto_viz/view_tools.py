
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QPushButton, QTabWidget, QLineEdit, QLabel, QHBoxLayout, QListWidget, QListWidgetItem, QComboBox, QGridLayout
from PyQt5.QtCore import Qt, pyqtSignal, QObject, pyqtSlot

class ActiveTools(QWidget):
    on_disconnect = pyqtSignal()
    start_planning = pyqtSignal()

    save_current_routes = pyqtSignal(dict)

    draw_points = pyqtSignal(list)
    stop_drawing_points = pyqtSignal()

    start_nav = pyqtSignal(str, bool)
    stop_nav = pyqtSignal()

    map_selected = pyqtSignal(str)

    switch_to_active = pyqtSignal()
    switch_to_configure = pyqtSignal()

    start_keys_vel = pyqtSignal(str, float)
    stop_keys_vel = pyqtSignal()

    def __init__(self, routes: dict, maps: list):
        super().__init__()

        self.routes = routes
        self.maps = maps
        
        self.active_route_name = None  # Store the name of active route

        # TABS
        main_layout = QVBoxLayout()
        self.tab_widget = QTabWidget()
        
        # Create first tab (Operation tab)
        operation_tab = QWidget()
        first_layout = QVBoxLayout(operation_tab)
        self.route_list = QListWidget()
        self.route_list.setFixedHeight(200)
        first_layout.addWidget(self.route_list)
        
        # Create button layout for first row
        button_layout_1 = QHBoxLayout()
        self.button_add = QPushButton("Add Route")
        self.button_remove = QPushButton("Remove Route")
        button_layout_1.addWidget(self.button_add)
        button_layout_1.addWidget(self.button_remove)
        first_layout.addLayout(button_layout_1)
        
        # Create button layout for second row
        button_layout_2 = QHBoxLayout()
        self.button_set_active = QPushButton("Set Active")
        button_layout_2.addWidget(self.button_set_active)
        first_layout.addLayout(button_layout_2)
        
        first_layout.insertSpacing(10,20)
        first_layout.addStretch()

        # Add the navigation buttons
        self.button_go_to_base = QPushButton("Go to base")
        self.button_go_to_dest = QPushButton("Go to dest")
        self.button_stop = QPushButton("Stop")
        first_layout.addWidget(self.button_go_to_base)
        first_layout.addWidget(self.button_go_to_dest)
        first_layout.addWidget(self.button_stop)
        
        self.tab_widget.addTab(operation_tab, "Operation")
        
        # Create second tab (Configuration tab)
        config_tab = QWidget()
        config_layout = QVBoxLayout(config_tab)
        
        # Add map selection combo box
        map_layout = QVBoxLayout()
        map_label = QLabel("Select Map:")
        self.map_combo = QComboBox()
        self.map_combo.addItems(self.maps)
        self.map_combo.currentTextChanged.connect(self.on_map_selected)
        
        map_layout.addWidget(map_label)
        map_layout.addWidget(self.map_combo)
        config_layout.addLayout(map_layout)
        config_layout.addStretch()

        self.upButton =QPushButton("↑")
        self.downButton =QPushButton("↓")
        self.leftButton =QPushButton("←")
        self.rightButton =QPushButton("→")

        self.upButton.setFixedSize(50, 50)
        self.leftButton.setFixedSize(50, 50)
        self.downButton.setFixedSize(50, 50)
        self.rightButton.setFixedSize(50, 50)

        keys_grid = QGridLayout()
        keys_grid.setSpacing(10)
        keys_grid.addWidget(self.upButton, 0,1)
        keys_grid.addWidget(self.leftButton, 1,0)
        keys_grid.addWidget(self.downButton, 1,1)
        keys_grid.addWidget(self.rightButton, 1,2)

        spaced_keys_layout = QHBoxLayout()
        spaced_keys_layout.addStretch()
        spaced_keys_layout.addLayout(keys_grid)
        spaced_keys_layout.addStretch()

        config_layout.addLayout(spaced_keys_layout)
        
        self.tab_widget.addTab(config_tab, "Configuration")
        
        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

        # Connecting keys signals 

        self.upButton.pressed.connect(lambda: self.start_keys_vel.emit('f', 0.5))
        self.leftButton.pressed.connect(lambda: self.start_keys_vel.emit('l', 0.5))
        self.downButton.pressed.connect(lambda: self.start_keys_vel.emit('b', 0.5))
        self.rightButton.pressed.connect(lambda: self.start_keys_vel.emit('r', 0.5))
        
        self.upButton.released.connect(lambda: self.stop_keys_vel.emit())
        self.leftButton.released.connect(lambda: self.stop_keys_vel.emit())
        self.downButton.released.connect(lambda: self.stop_keys_vel.emit())
        self.rightButton.released.connect(lambda: self.stop_keys_vel.emit())

        # Connecting button signals
        self.button_add.clicked.connect(lambda: self.start_planning.emit())
        self.button_remove.clicked.connect(self.remove_route)
        self.button_set_active.clicked.connect(self.set_active_route)
        self.button_go_to_dest.clicked.connect(self.handle_navigate_to_dest)
        self.button_go_to_base.clicked.connect(self.handle_navigate_to_base)
        self.button_stop.clicked.connect(lambda: self.stop_nav.emit())

        self.tab_widget.currentChanged.connect(self.emit_based_on_tab)

    def emit_based_on_tab(self, index: int):
        if index == 0:
            self.switch_to_active.emit()
        if index == 1:
            self.switch_to_configure.emit()

    def update_maps(self, maps: list):
        """Update the map combo box with new maps"""
        current_map = self.map_combo.currentText()
        self.map_combo.clear()
        self.map_combo.addItems(maps)
        
        # Try to restore the previously selected map
        index = self.map_combo.findText(current_map)
        if index >= 0:
            self.map_combo.setCurrentIndex(index)

    def on_map_selected(self, map_name: str):
        """Handle map selection"""
        if map_name:
            print(f"selected: {map_name}")
            self.map_selected.emit(map_name)


    def handle_navigate_to_dest(self):
        if self.active_route_name:
            self.start_nav.emit(self.active_route_name, True)

    def handle_navigate_to_base(self):
        if self.active_route_name:
            self.start_nav.emit(self.active_route_name, False)

    def update_routes(self):
        """Load routes from received list while preserving active route"""
        current_selection = self.route_list.currentItem()
        currently_selected_name = current_selection.text() if current_selection else None
        if currently_selected_name and currently_selected_name.startswith("✓ "):
            currently_selected_name = currently_selected_name[2:]
        
        # Clear existing routes
        self.route_list.clear()
        
        # Add new routes, maintaining active route if it still exists
        for route_name in list(self.routes.keys()):
            item = QListWidgetItem()
            display_name = route_name
            
            # If this was the active route and still exists, mark it active
            if route_name == self.active_route_name:
                display_name = "✓ " + route_name
                item.setForeground(Qt.green)
                self.route_list.insertItem(0, item)
            else:
                item.setForeground(Qt.black)
                self.route_list.addItem(item)
                
            item.setText(display_name)
            
            # Restore selection if possible
            if route_name == currently_selected_name:
                self.route_list.setCurrentItem(item)
        
        # If active route was removed, clear the active route state
        if self.active_route_name and self.active_route_name not in self.routes:
            self.active_route_name = None
            self.stop_drawing_points.emit()
        elif self.active_route_name:
            # Redraw points for active route
            self.draw_points.emit(self.routes[self.active_route_name])


    def remove_route(self):
        """Remove the currently selected route"""
        current_item = self.route_list.currentItem()
        if current_item:
            route_name = current_item.text()
            if route_name.startswith("✓ "):
                route_name = route_name[2:]
            
            # Clear active route if removing it
            if route_name == self.active_route_name:
                self.active_route_name = None
            
            self.routes.pop(route_name)
            self.update_routes()
            self.save_current_routes.emit(self.routes)
            self.stop_drawing_points.emit()

    def set_active_route(self):
        """Set the selected route as active"""
        current_item = self.route_list.currentItem()
        if current_item:
            route_name = current_item.text()
            if route_name.startswith("✓ "):
                route_name = route_name[2:]
            
            # Update active route name
            self.active_route_name = route_name
            
            # Update display
            self.update_routes()
            
            # Draw points for the active route
            if route_name in self.routes:
                self.draw_points.emit(self.routes[route_name])
            

class PlanningTools(QWidget):
    finish_planning = pyqtSignal()
    save_current_routes = pyqtSignal(dict)

    draw_points = pyqtSignal(list)
    stop_drawing_points = pyqtSignal()

    def __init__(self, routes: dict):
        super().__init__()

        self.routes: dict = routes

        self.new_route: list = list()

        self.setup_ui()
        
    def setup_ui(self):
        self.route_name = QLineEdit(self)
        self.main_layout = QVBoxLayout()
         
        self.done_button = QPushButton("Done")
        self.cancel_button = QPushButton("Cancel")

        self._last_point = None  # Store last point to prevent duplicates

        self.done_button.clicked.connect(self.exit_done)
        self.cancel_button.clicked.connect(self.exit_cancel)

        # layout.addWidget(self.map_view, 3)
        # layout.addWidget(self.finish_planning_button, 1)
        self.main_layout.addWidget(self.route_name)
        self.main_layout.addStretch()
        self.main_layout.addWidget(self.done_button)
        self.main_layout.addWidget(self.cancel_button)
        self.setLayout(self.main_layout)

    pyqtSlot(float,float,float)
    def setPoint(self, x, y, theta):
        # Check if this is a duplicate point
        new_point = [x, y, 0.0, theta]
        if self._last_point != new_point:  # Only append if different
            self.new_route.append(new_point)
            self._last_point = new_point
            self.draw_points.emit(self.new_route)

    def exit_cancel(self):
        self.new_route.clear()
        self.stop_drawing_points.emit()
        self.finish_planning.emit()
    
    def exit_done(self):
        if self.route_name.text().strip():  # Only save if name is not empty
            self.routes[self.route_name.text()] = self.new_route.copy()  # Make a copy
            self.save_current_routes.emit(self.routes)
            self.new_route.clear()
            self._last_point = None
            self.stop_drawing_points.emit()
            self.finish_planning.emit()
        

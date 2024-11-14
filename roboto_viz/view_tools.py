
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QPushButton, QTabWidget, QLineEdit, QLabel, QHBoxLayout, QListWidget, QListWidgetItem, QComboBox, QGridLayout
from PyQt5.QtGui import QPainter, QColor, QPen, QFont
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer

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

        self.active_route_name = None 

        button_style = """
            QPushButton {
                min-height: 40px;
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
        """
        

        stop_button_style = """
            QPushButton {
                min-height: 40px;
                font-size: 14px;
                padding: 5px 10px;
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

        tab_style = """
            QTabWidget::pane {
                border: 2px solid #2c3e50;
                border-radius: 5px;
                background: white;
            }

            QTabBar::tab {
                background: #ecf0f1;
                border: 2px solid #2c3e50;
                border-bottom: none;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
                min-width: 120px;
                min-height: 35px;
                padding: 5px;
                margin-right: 2px;
                font-size: 14px;
                font-weight: bold;
                color: #2c3e50;
            }

            QTabBar::tab:selected {
                background: white;
                margin-bottom: -2px;
                padding-bottom: 7px;
            }

            QTabBar::tab:hover:!selected {
                background: #d0d3d4;
            }
        """
        
        # TABS
        main_layout = QVBoxLayout()
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet(tab_style)

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

        statusLabel = QLabel("Status:")
        self.status_display = StatusDisplay()
        first_layout.addWidget(statusLabel)
        first_layout.addWidget(self.status_display)

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

        self.button_add.setStyleSheet(button_style)
        self.button_remove.setStyleSheet(button_style)
        self.button_set_active.setStyleSheet(button_style)
        self.button_go_to_base.setStyleSheet(button_style)
        self.button_go_to_dest.setStyleSheet(button_style)
        
        # Apply special style to stop button
        self.button_stop.setStyleSheet(stop_button_style)

        direction_button_style = """
            QPushButton {
                min-height: 60px;
                min-width: 60px;
                font-size: 20px;
                font-weight: bold;
                border: 2px solid #2980b9;
                border-radius: 5px;
                background-color: #3498db;
                color: white;
            }
            QPushButton:hover {
                background-color: #2980b9;
                border-color: #2472a4;
            }
            QPushButton:pressed {
                background-color: #2472a4;
                border-color: #1a5276;
            }
        """
        
        self.upButton.setStyleSheet(direction_button_style)
        self.leftButton.setStyleSheet(direction_button_style)
        self.downButton.setStyleSheet(direction_button_style)
        self.rightButton.setStyleSheet(direction_button_style)
        
        # Update fixed sizes for direction buttons
        self.upButton.setFixedSize(60, 60)
        self.leftButton.setFixedSize(60, 60)
        self.downButton.setFixedSize(60, 60)
        self.rightButton.setFixedSize(60, 60)

        # Make the route list font bigger
        route_list_font = QFont()
        route_list_font.setPointSize(12)
        self.route_list.setFont(route_list_font)

        # Make the map combo box font bigger
        map_combo_font = QFont()
        map_combo_font.setPointSize(12)
        self.map_combo.setFont(map_combo_font)

        # Make labels bigger
        label_font = QFont()
        label_font.setPointSize(12)
        statusLabel.setFont(label_font)
        map_label.setFont(label_font)

        # Update the list style to include active state
        list_style = """
            QListWidget {
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                padding: 5px;
                background-color: white;
            }
            QListWidget::item {
                height: 30px;
                border-radius: 3px;
                padding: 5px;
            }
            QListWidget::item:selected {
                background-color: #3498db;
                color: white;
            }
            QListWidget::item:hover:!selected {
                background-color: #ecf0f1;
            }
        """
        self.route_list.setStyleSheet(list_style)

        # Add some vertical spacing between major sections
        first_layout.setSpacing(10)
        config_layout.setSpacing(10)

        # Style the status label
        status_label_style = """
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: #2c3e50;
                padding: 5px;
            }
        """
        statusLabel.setStyleSheet(status_label_style)
        map_label.setStyleSheet(status_label_style)

    pyqtSlot(str)
    def set_current_status(self, status: str):
        """Update the status display"""
        self.status_display.set_status(status)

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
            # self.set_current_status("Nav to dest")

    def handle_navigate_to_base(self):
        if self.active_route_name:
            self.start_nav.emit(self.active_route_name, False)
            # self.set_current_status("Nav to base")

    def update_routes(self):
        """Load routes from received list while preserving active route"""
        current_selection = self.route_list.currentItem()
        currently_selected_name = current_selection.text() if current_selection else None
        
        # Clear existing routes
        self.route_list.clear()
        
        # Add new routes, maintaining active route if it still exists
        for route_name in list(self.routes.keys()):
            item = QListWidgetItem()
            item.setText(route_name)  # Just the name, no check mark
            
            # If this is the active route, give it a distinct background
            if route_name == self.active_route_name:
                font = QFont()
                # font.setPixelSize(20)
                font.setBold(True)
                item.setBackground(Qt.white)  # Green background for active route
                item.setForeground(Qt.black)  # White text for better contrast
                item.setFont(font)
                self.route_list.insertItem(0, item)
            else:
                item.setBackground(QColor("white"))  # Normal background
                item.setForeground(QColor("#2c3e50"))  # Dark text for normal routes
                self.route_list.addItem(item)
            
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

    def set_active_route(self):
        """Set the selected route as active"""
        current_item = self.route_list.currentItem()
        if current_item:
            route_name = current_item.text()  # No need to strip check mark anymore
            
            # Update active route name
            self.active_route_name = route_name
            
            # Update display
            self.update_routes()
            
            # Draw points for the active route
            if route_name in self.routes:
                self.draw_points.emit(self.routes[route_name])

    def remove_route(self):
        """Remove the currently selected route"""
        current_item = self.route_list.currentItem()
        if current_item:
            route_name = current_item.text()  # No need to strip check mark anymore
            
            # Clear active route if removing it
            if route_name == self.active_route_name:
                self.active_route_name = None
            
            self.routes.pop(route_name)
            self.update_routes()
            self.save_current_routes.emit(self.routes)
            self.stop_drawing_points.emit()
            

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

        self.route_name_label = QLabel("Route Name:")
        label_font = QFont()
        label_font.setPointSize(12)
        self.route_name_label.setFont(label_font)


        self.done_button = QPushButton("Done")
        self.cancel_button = QPushButton("Cancel")

        self._last_point = None  # Store last point to prevent duplicates

        self.done_button.clicked.connect(self.exit_done)
        self.cancel_button.clicked.connect(self.exit_cancel)

        # layout.addWidget(self.map_view, 3)
        # layout.addWidget(self.finish_planning_button, 1)
        self.main_layout.addWidget(self.route_name_label)
        self.main_layout.addWidget(self.route_name)
        self.main_layout.addStretch()
        self.main_layout.addWidget(self.done_button)
        self.main_layout.addWidget(self.cancel_button)
        self.setLayout(self.main_layout)


        button_style = """
            QPushButton {
                min-height: 40px;
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
        """
        
        self.done_button.setStyleSheet(button_style)
        self.cancel_button.setStyleSheet(button_style)
        
        # Make the route name input bigger
        route_name_style = """
            QLineEdit {
                min-height: 40px;
                font-size: 14px;
                padding: 5px 10px;
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                background-color: white;
            }
        """
        self.route_name.setStyleSheet(route_name_style)

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
        
class LEDIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(32, 32)
        self._color = QColor(Qt.green)
        self._flashing = False
        self._flash_state = True
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._toggle_flash)
        self.timer.setInterval(300)
        
    def _toggle_flash(self):
        self._flash_state = not self._flash_state
        self.update()
        
    def setFlashing(self, enable, color=None):
        self._flashing = enable
        if color:
            self._color = QColor(color)
        if enable:
            self.timer.start()
        else:
            self.timer.stop()
            self._flash_state = True
        self.update()
        
    def setColor(self, color):
        self._color = QColor(color)
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw LED border
        pen = QPen(Qt.black)
        pen.setWidth(1)
        painter.setPen(pen)
        
        if self._flashing and not self._flash_state:
            painter.setBrush(Qt.black)
        else:
            painter.setBrush(self._color)
            
        painter.drawEllipse(4, 4, 24, 24)

class StatusDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.led = LEDIndicator()
        self.status_label = QLabel("Idle")
        font = QFont()
        font.setPointSize(15)
        font.setBold(True)
        self.status_label.setFont(font)
        # self.status_label.setStyleSheet("font-weight: bold;")
        
        layout.addWidget(self.led)
        layout.addWidget(self.status_label)
        layout.addStretch()
        
        self.setLayout(layout)
        
    def set_status(self, status: str):
        self.status_label.setText(status)
        
        if status in ["Idle", "At base", "At destination"]:
            self.led.setFlashing(False)
            self.led.setColor(Qt.green)
        elif status in ["Nav to base", "Nav to dest", "Manual move"]:
            self.led.setFlashing(True, QColor(255, 165, 0))  # Orange color
        elif status == "Failed":
            self.led.setFlashing(True)
            self.led.setColor(Qt.red)

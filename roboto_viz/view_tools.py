
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QPushButton, QTabWidget, QLineEdit, QLabel, QHBoxLayout, QListWidget, QListWidgetItem, QComboBox, QGridLayout
from PyQt5.QtGui import QPainter, QColor, QPen, QFont
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer
from roboto_viz.route_manager import BezierRoute, RouteNode

class ActiveTools(QWidget):
    on_disconnect = pyqtSignal()
    start_planning = pyqtSignal()

    save_current_routes = pyqtSignal(dict)

    draw_route = pyqtSignal(object)  # BezierRoute object
    stop_drawing_points = pyqtSignal()

    start_nav = pyqtSignal(str, bool)
    stop_nav = pyqtSignal()
    
    dock_robot = pyqtSignal()
    undock_robot = pyqtSignal()

    map_selected = pyqtSignal(str)

    switch_to_active = pyqtSignal()
    switch_to_configure = pyqtSignal()

    start_keys_vel = pyqtSignal(str, float)
    stop_keys_vel = pyqtSignal()

    def __init__(self, routes: dict, maps: list, map_view=None):
        super().__init__()

        self.routes = routes
        self.maps = maps
        self.map_view = map_view

        self.active_route_name = None 

        button_style = """
            QPushButton {
                min-height: 40px;
                font-size: 16px;
                padding: 8px 16px;
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
        """
        

        stop_button_style = """
            QPushButton {
                min-height: 40px;
                font-size: 16px;
                padding: 8px 16px;
                font-weight: bold;
                border: 2px solid #c0392b;
                border-radius: 8px;
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
                border: 1px solid #2c3e50;
                border-radius: 3px;
                background: white;
            }

            QTabBar::tab {
                background: #ecf0f1;
                border: 2px solid #2c3e50;
                border-bottom: none;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                min-width: 120px;
                min-height: 35px;
                padding: 8px 16px;
                margin-right: 2px;
                font-size: 16px;
                font-weight: bold;
                color: #2c3e50;
            }

            QTabBar::tab:selected {
                background: white;
                margin-bottom: -1px;
                padding-bottom: 3px;
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
        self.route_list.setFixedHeight(200)  # Larger height for big screens
        first_layout.addWidget(self.route_list)
        
        # Create button layout for first row
        button_layout_1 = QHBoxLayout()
        self.button_add = QPushButton("Dodaj Trasę")
        self.button_remove = QPushButton("Usuń Trasę")
        button_layout_1.addWidget(self.button_add)
        button_layout_1.addWidget(self.button_remove)
        first_layout.addLayout(button_layout_1)
        
        # Create button layout for second row
        button_layout_2 = QHBoxLayout()
        self.button_set_active = QPushButton("Ustaw zaznaczenie jako aktywne")
        button_layout_2.addWidget(self.button_set_active)
        first_layout.addLayout(button_layout_2)
        
        first_layout.insertSpacing(10,20)
        first_layout.addStretch()

        # Add the navigation buttons
        self.button_go_to_base = QPushButton("Idź do bazy")
        self.button_go_to_dest = QPushButton("Idź do celu")
        
        # Add dock and undock buttons in a horizontal layout
        dock_layout = QHBoxLayout()
        self.button_dock = QPushButton("Dokuj")
        self.button_undock = QPushButton("Oddokuj")
        dock_layout.addWidget(self.button_dock)
        dock_layout.addWidget(self.button_undock)
        
        self.button_stop = QPushButton("Zatrzymaj")
        first_layout.addWidget(self.button_go_to_base)
        first_layout.addWidget(self.button_go_to_dest)
        first_layout.addLayout(dock_layout)
        first_layout.addWidget(self.button_stop)
        
        self.tab_widget.addTab(operation_tab, "Operacje")
        
        # Create second tab (Configuration tab)
        config_tab = QWidget()
        config_layout = QVBoxLayout(config_tab)
        
        # Add map selection combo box
        map_layout = QVBoxLayout()
        map_label = QLabel("Wybierz Mapę:")
        self.map_combo = QComboBox()
        self.map_combo.addItems(self.maps)
        self.map_combo.currentTextChanged.connect(self.on_map_selected)
        
        map_layout.addWidget(map_label)
        map_layout.addWidget(self.map_combo)
        config_layout.addLayout(map_layout)
        
        # Map is now on the left side, not in configure tab
        
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
        keys_grid.setSpacing(20)  # More spacing for larger buttons
        keys_grid.addWidget(self.upButton, 0,1)
        keys_grid.addWidget(self.leftButton, 1,0)
        keys_grid.addWidget(self.downButton, 1,1)
        keys_grid.addWidget(self.rightButton, 1,2)

        spaced_keys_layout = QHBoxLayout()
        spaced_keys_layout.addStretch()
        spaced_keys_layout.addLayout(keys_grid)
        spaced_keys_layout.addStretch()

        config_layout.addLayout(spaced_keys_layout)
        
        self.tab_widget.addTab(config_tab, "Konfiguracja")
        
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
        self.button_dock.clicked.connect(lambda: self.dock_robot.emit())
        self.button_undock.clicked.connect(lambda: self.undock_robot.emit())
        self.button_stop.clicked.connect(lambda: self.stop_nav.emit())

        self.tab_widget.currentChanged.connect(self.emit_based_on_tab)

        self.button_add.setStyleSheet(button_style)
        self.button_remove.setStyleSheet(button_style)
        self.button_set_active.setStyleSheet(button_style)
        self.button_go_to_base.setStyleSheet(button_style)
        self.button_go_to_dest.setStyleSheet(button_style)
        self.button_dock.setStyleSheet(button_style)
        self.button_undock.setStyleSheet(button_style)
        
        # Apply special style to stop button
        self.button_stop.setStyleSheet(stop_button_style)

        direction_button_style = """
            QPushButton {
                min-height: 50px;
                min-width: 50px;
                font-size: 20px;
                font-weight: bold;
                border: 2px solid #2980b9;
                border-radius: 8px;
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
        
        # Update fixed sizes for direction buttons (large screen)
        self.upButton.setFixedSize(50, 50)
        self.leftButton.setFixedSize(50, 50)
        self.downButton.setFixedSize(50, 50)
        self.rightButton.setFixedSize(50, 50)

        # Make the route list font larger for big screens
        route_list_font = QFont()
        route_list_font.setPointSize(14)
        self.route_list.setFont(route_list_font)

        # Make the map combo box font larger for big screens
        map_combo_font = QFont()
        map_combo_font.setPointSize(14)
        self.map_combo.setFont(map_combo_font)

        # Make labels larger for big screens
        label_font = QFont()
        label_font.setPointSize(16)
        map_label.setFont(label_font)

        # Update the list style to include active state
        list_style = """
            QListWidget {
                border: 2px solid #bdc3c7;
                border-radius: 4px;
                padding: 3px;
                background-color: white;
            }
            QListWidget::item {
                height: 40px;
                border-radius: 8px;
                padding: 8px;
                font-size: 14px;
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

        # Add some vertical spacing between major sections (larger for big screens)
        first_layout.setSpacing(15)
        config_layout.setSpacing(15)

        # Style the status label
        status_label_style = """
            QLabel {
                font-size: 16px;
                font-weight: bold;
                color: #2c3e50;
                padding: 8px;
            }
        """
        map_label.setStyleSheet(status_label_style)


    def emit_based_on_tab(self, index: int):
        if index == 0:
            # Operation tab - show grid
            self.switch_to_active.emit()
        if index == 1:
            # Configuration tab - show map
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
            print(f"DEBUG: Map selected: {map_name}")
            
            # Clear current routes when map changes
            self.routes.clear()
            self.active_route_name = None
            self.update_routes()
            
            # Stop any current drawing/route display
            self.stop_drawing_points.emit()
            
            # Emit the map selection signal - this will trigger map loading
            self.map_selected.emit(map_name)
            
            print(f"DEBUG: Map selection completed for: {map_name}")

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
            # Redraw route for active route
            self.draw_route.emit(self.routes[self.active_route_name])

    def set_active_route(self):
        """Set the selected route as active"""
        current_item = self.route_list.currentItem()
        if current_item:
            route_name = current_item.text()  # No need to strip check mark anymore
            
            # Clear previous route graphics first
            self.stop_drawing_points.emit()
            
            # Update active route name
            self.active_route_name = route_name
            
            # Update display
            self.update_routes()
            
            # Draw route for the active route
            if route_name in self.routes:
                self.draw_route.emit(self.routes[route_name])

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

    start_route_editing = pyqtSignal()
    stop_route_editing = pyqtSignal()
    get_current_route = pyqtSignal()

    def __init__(self, routes: dict):
        super().__init__()

        self.routes: dict = routes
        self.current_bezier_route = BezierRoute()

        self.setup_ui()
        
    def setup_ui(self):
        self.route_name = QLineEdit(self)
        self.main_layout = QVBoxLayout()

        self.route_name_label = QLabel("Nazwa Trasy:")
        label_font = QFont()
        label_font.setPointSize(16)  # Larger font size for big screens
        self.route_name_label.setFont(label_font)

        # Instructions
        self.instructions_label = QLabel("Kliknij na mapę, aby dodać węzły.\nPrzeciągnij węzły, aby je przesunąć.\nKliknij dwukrotnie lub prawym przyciskiem, aby usunąć.")
        self.instructions_label.setWordWrap(True)
        self.instructions_label.setStyleSheet("color: #555; font-size: 14px; padding: 12px;")

        self.done_button = QPushButton("Gotowe")
        self.cancel_button = QPushButton("Anuluj")

        self.done_button.clicked.connect(self.exit_done)
        self.cancel_button.clicked.connect(self.exit_cancel)

        self.main_layout.addWidget(self.route_name_label)
        self.main_layout.addWidget(self.route_name)
        self.main_layout.addWidget(self.instructions_label)
        self.main_layout.addStretch()
        self.main_layout.addWidget(self.done_button)
        self.main_layout.addWidget(self.cancel_button)
        self.setLayout(self.main_layout)

        button_style = """
            QPushButton {
                min-height: 40px;
                font-size: 16px;
                padding: 8px 16px;
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
        """
        
        self.done_button.setStyleSheet(button_style)
        self.cancel_button.setStyleSheet(button_style)
        
        # Make the route name input smaller
        route_name_style = """
            QLineEdit {
                min-height: 40px;
                font-size: 16px;
                padding: 8px 16px;
                border: 2px solid #bdc3c7;
                border-radius: 8px;
                background-color: white;
            }
        """
        self.route_name.setStyleSheet(route_name_style)

    def start_planning_mode(self):
        """Start route planning mode"""
        self.current_bezier_route = BezierRoute()
        self.route_name.clear()
        self.start_route_editing.emit()

    def exit_cancel(self):
        self.current_bezier_route = BezierRoute()
        self.stop_route_editing.emit()
        self.finish_planning.emit()
    
    def exit_done(self):
        if self.route_name.text().strip():  # Only save if name is not empty
            # Get the current route from the map view
            self.get_current_route.emit()  # This should trigger a callback
            
    def save_route(self, bezier_route: BezierRoute):
        """Called when route should be saved"""
        if bezier_route and self.route_name.text().strip():
            self.routes[self.route_name.text()] = bezier_route
            self.save_current_routes.emit(self.routes)
            self.current_bezier_route = BezierRoute()
            self.stop_route_editing.emit()
            self.finish_planning.emit()
        
class LEDIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(40, 40)  # Larger LED size for big screens
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
            
        painter.drawEllipse(5, 5, 30, 30)  # Adjusted for larger size

class StatusDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.led = LEDIndicator()
        self.status_label = QLabel("Bezczynny")
        font = QFont()
        font.setPointSize(18)  # Larger font size for big screens
        font.setBold(True)
        self.status_label.setFont(font)
        # self.status_label.setStyleSheet("font-weight: bold;")
        
        layout.addWidget(self.led)
        layout.addWidget(self.status_label)
        layout.addStretch()
        
        self.setLayout(layout)
        
    def set_status(self, status: str):
        self.status_label.setText(status)
        
        if status in ["Bezczynny", "W bazie", "Na miejscu docelowym"]:
            self.led.setFlashing(False)
            self.led.setColor(Qt.green)
        elif status in ["Nawigacja do bazy", "Nawigacja do celu", "Ruch ręczny"]:
            self.led.setFlashing(True, QColor(255, 165, 0))  # Orange color
        elif status == "Błąd":
            self.led.setFlashing(True)
            self.led.setColor(Qt.red)

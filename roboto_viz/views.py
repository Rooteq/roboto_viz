from PyQt5.QtWidgets import QStackedWidget, QVBoxLayout, QWidget, QPushButton, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from roboto_viz.view_tools import ActiveTools, PlanningTools
from PyQt5.QtGui import QFont
from roboto_viz.route_manager import BezierRoute

from roboto_viz.map_view import MapView

class ActiveView(QWidget):
    finish_planning = pyqtSignal()
    on_disconnection = pyqtSignal(str)
    start_planning = pyqtSignal()
    start_nav = pyqtSignal(str, bool, float, float)


    def __init__(self, map_view : MapView):
        super().__init__()

        self.map_view = map_view

        self.routes: dict = dict()
        self.maps: list = list()


        self.curr_x: float = 0
        self.curr_y: float = 0

        self.active_tools = ActiveTools(self.routes, self.maps, self.map_view)
        self.planning_tools = PlanningTools(self.routes)

        self.stacked_widget = QStackedWidget()
        # self.layout.addWidget(self.stacked_widget)

        self.stacked_widget.addWidget(self.active_tools)
        self.stacked_widget.addWidget(self.planning_tools)

        self.main_layout = QHBoxLayout()
        self.main_layout.setContentsMargins(2, 2, 2, 2)  # Reduced margins
        self.main_layout.setSpacing(2)  # Reduced spacing
        
        # Create stacked widget for left side (grid or map)
        self.left_stacked_widget = QStackedWidget()
        
        # Create proper grid widget with status cells
        self.grid_widget = QWidget()
        self.grid_widget.setStyleSheet("QWidget { background-color: #e8f4fd; border: 3px solid red; }")
        self.grid_widget.setMinimumSize(300, 200)
        from PyQt5.QtWidgets import QGridLayout
        grid_layout = QGridLayout(self.grid_widget)
        grid_layout.setContentsMargins(10, 10, 10, 10)
        grid_layout.setSpacing(10)
        
        # Create status cells in a grid format
        # Robot Status Cell
        self.robot_status_frame = QWidget()
        self.robot_status_frame.setMinimumSize(120, 80)
        self.robot_status_frame.setStyleSheet("""
            QWidget {
                border: 2px solid #3498db;
                border-radius: 5px;
                background-color: white;
                padding: 5px;
            }
        """)
        robot_layout = QVBoxLayout(self.robot_status_frame)
        robot_layout.setContentsMargins(5, 3, 5, 3)
        robot_layout.setSpacing(2)
        
        robot_title = QLabel("Status Robota")
        robot_title.setStyleSheet("""
            QLabel {
                font-size: 18px;
                font-weight: bold;
                color: #2c3e50;
                background: none;
                border: none;
            }
        """)
        robot_layout.addWidget(robot_title)
        
        from roboto_viz.view_tools import StatusDisplay
        self.status_display = StatusDisplay()
        robot_layout.addWidget(self.status_display)

        # Plan Status Cell
        plan_status_frame = QWidget()
        plan_status_frame.setMinimumSize(120, 80)
        plan_status_frame.setStyleSheet("""
            QWidget {
                border: 2px solid #e74c3c;
                border-radius: 5px;
                background-color: white;
                padding: 5px;
            }
        """)
        plan_layout = QVBoxLayout(plan_status_frame)
        plan_layout.setContentsMargins(5, 3, 5, 3)
        plan_layout.setSpacing(2)
        
        plan_title = QLabel("Status Planu")
        plan_title.setStyleSheet("""
            QLabel {
                font-size: 18px;
                font-weight: bold;
                color: #2c3e50;
                background: none;
                border: none;
            }
        """)
        plan_layout.addWidget(plan_title)
        
        self.plan_status_display = QLabel("Brak aktywnego planu")
        self.plan_status_display.setStyleSheet("""
            QLabel {
                font-size: 16px;
                color: #7f8c8d;
                background: none;
                border: none;
            }
        """)
        plan_layout.addWidget(self.plan_status_display)

        # Navigation Status Cell
        nav_status_frame = QWidget()
        nav_status_frame.setMinimumSize(120, 80)
        nav_status_frame.setStyleSheet("""
            QWidget {
                border: 2px solid #2ecc71;
                border-radius: 5px;
                background-color: white;
                padding: 5px;
            }
        """)
        nav_layout = QVBoxLayout(nav_status_frame)
        nav_layout.setContentsMargins(5, 3, 5, 3)
        nav_layout.setSpacing(2)
        
        nav_title = QLabel("Nawigacja")
        nav_title.setStyleSheet("""
            QLabel {
                font-size: 18px;
                font-weight: bold;
                color: #2c3e50;
                background: none;
                border: none;
            }
        """)
        nav_layout.addWidget(nav_title)
        
        self.nav_status_display = QLabel("Bezczynny")
        self.nav_status_display.setStyleSheet("""
            QLabel {
                font-size: 16px;
                color: #7f8c8d;
                background: none;
                border: none;
            }
        """)
        nav_layout.addWidget(self.nav_status_display)
        
        # Battery Status Cell
        self.battery_status_frame = QWidget()
        self.battery_status_frame.setMinimumSize(120, 80)
        self.battery_status_frame.setStyleSheet("""
            QWidget {
                border: 2px solid #f39c12;
                border-radius: 5px;
                background-color: white;
                padding: 5px;
            }
        """)
        battery_layout = QVBoxLayout(self.battery_status_frame)
        battery_layout.setContentsMargins(5, 3, 5, 3)
        battery_layout.setSpacing(2)
        
        battery_title = QLabel("Bateria")
        battery_title.setStyleSheet("""
            QLabel {
                font-size: 18px;
                font-weight: bold;
                color: #2c3e50;
                background: none;
                border: none;
            }
        """)
        battery_layout.addWidget(battery_title)
        
        self.battery_status_display = QLabel("Nieznany")
        self.battery_status_display.setStyleSheet("""
            QLabel {
                font-size: 16px;
                color: #7f8c8d;
                background: none;
                border: none;
            }
        """)
        battery_layout.addWidget(self.battery_status_display)

        # Add cells to grid (2x2 layout)
        grid_layout.addWidget(self.robot_status_frame, 0, 0)
        grid_layout.addWidget(plan_status_frame, 0, 1)
        grid_layout.addWidget(nav_status_frame, 1, 0)
        grid_layout.addWidget(self.battery_status_frame, 1, 1)

        # Add both grid and map to stacked widget
        self.left_stacked_widget.addWidget(self.grid_widget)  # Index 0
        self.left_stacked_widget.addWidget(self.map_view)     # Index 1

        # Start with grid view
        self.left_stacked_widget.setCurrentIndex(0)
        print(f"DEBUG: Grid widget created with {len(grid_layout.children())} children")
        print(f"DEBUG: Left stacked widget has {self.left_stacked_widget.count()} widgets")
        print(f"DEBUG: Current index: {self.left_stacked_widget.currentIndex()}")
        self.grid_widget.show()

        self.main_layout.addWidget(self.left_stacked_widget, 3)
        self.main_layout.addWidget(self.stacked_widget, 1)
        self.setLayout(self.main_layout)

        self.active_tools.start_nav.connect(self.handle_set_route)

        # Connect route drawing signals
        self.active_tools.draw_route.connect(self.map_view.display_bezier_route)
        self.active_tools.stop_drawing_points.connect(self.map_view.clear_route)

        # Connect planning tools signals
        self.planning_tools.start_route_editing.connect(lambda: self.map_view.start_route_editing())
        self.planning_tools.stop_route_editing.connect(self.map_view.stop_route_editing)
        self.planning_tools.get_current_route.connect(self.handle_get_current_route)

        # Connect tab switching signals to control left stacked widget
        self.active_tools.switch_to_active.connect(
            lambda: self.left_stacked_widget.setCurrentIndex(0))  # Grid
        self.active_tools.switch_to_configure.connect(
            lambda: self.left_stacked_widget.setCurrentIndex(1))  # Map

    @pyqtSlot(str, bool)
    def handle_set_route(self, route_name, to_dest):
        self.start_nav.emit(route_name, to_dest, self.curr_x, self.curr_y)

    @pyqtSlot(float, float, float)
    def update_robot_pose(self, x, y, theta):
        self.map_view.update_robot_pose(x, y, theta)
        self.curr_x = x
        self.curr_y = y

    @pyqtSlot(dict)
    def load__routes(self, routes):
        self.routes.clear()
        self.routes.update(routes)
        self.active_tools.update_routes()

    @pyqtSlot(list)
    def load_maps(self, maps: list):
        self.maps = maps.copy()  # Make a copy of the list
        self.active_tools.update_maps(self.maps)  # Update maps in ActiveTools

    def switch_to_active_tools(self):
        self.stacked_widget.setCurrentWidget(self.active_tools)

        self.active_tools.on_disconnect.connect(lambda: self.on_disconnection.emit())
        self.active_tools.start_planning.connect(lambda: self.start_planning.emit())

    def switch_to_configuring_tab(self):
        self.active_tools.tab_widget.setCurrentIndex(1)
        # Show map on left side for pose setting
        self.left_stacked_widget.setCurrentIndex(1)  # Map

    def switch_to_active(self):  # OPTIMIZE
        self.active_tools.tab_widget.setCurrentIndex(0)
        # Show grid on left side for status display
        self.left_stacked_widget.setCurrentIndex(0)  # Grid

    def switch_to_planning(self):
        self.stacked_widget.setCurrentWidget(self.planning_tools)
        
        # Start planning mode
        self.planning_tools.start_planning_mode()

        # Connect signals
        self.planning_tools.finish_planning.connect(lambda: self.finish_planning.emit())
        self.planning_tools.get_current_route.connect(self.handle_get_current_route)
        
    def handle_get_current_route(self):
        """Get current route from map view and save it."""
        current_route = self.map_view.get_current_route()
        if current_route:
            self.planning_tools.save_route(current_route)
    
    @pyqtSlot(str)
    def set_current_status(self, status: str):
        """Update the status display."""
        self.status_display.set_status(status)

    def set_nav_status(self, status: str):
        """Update the navigation status display."""
        self.nav_status_display.setText(status)

    def set_battery_status(self, status: str):
        """Update the battery status display."""
        self.battery_status_display.setText(status)
    
    def update_battery_status(self, percentage: int, status_string: str):
        """Update battery status display with color coding based on percentage."""
        # Update the text
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
                # Normal white background
                bg_color = 'white'
                border_color = '#f39c12'
            
            # Update the frame style with new background color
            self.battery_status_frame.setStyleSheet(f'''
                QWidget {{
                    border: 2px solid {border_color};
                    border-radius: 5px;
                    background-color: {bg_color};
                    padding: 5px;
                }}
            ''')

    def set_plan_status(self, status: str):
        """Update the plan status display."""
        self.plan_status_display.setText(status)

    def update_robot_status(self, status: str):
        """Update robot status display with color coding for the grid cell."""
        # Update the status display (LED and text)
        self.status_display.set_status(status)

        # Update the robot status frame background color
        if hasattr(self, 'robot_status_frame'):
            # Define background colors based on status
            if status in ['Bezczynny', 'W bazie', 'Na miejscu docelowym']:
                # Light green for normal operational states
                bg_color = '#d5f4e6'
                border_color = '#27ae60'
            elif status in ['Nav to base', 'Nav to dest', 'Navigating', 'Manual move',
                            'Docking', 'Undocking', 'Oczekiwanie na sygnał'] or status.startswith('Wykonywanie'):
                # Light blue for active operations
                bg_color = '#d6eaf8'
                border_color = '#3498db'
            elif status in ['Failed', 'Error', 'Connection lost', 'Navigation Error']:
                # Light red for errors
                bg_color = '#fadbd8'
                border_color = '#e74c3c'
            elif status in ['Warning', 'Low battery', 'Obstacle detected']:
                # Light orange for warnings
                bg_color = '#fdebd0'
                border_color = '#f39c12'
            else:
                # Default light gray for unknown states
                bg_color = 'white'
                border_color = '#3498db'

            # Update the frame style with new background color
            self.robot_status_frame.setStyleSheet(f'''
                QWidget {{
                    border: 2px solid {border_color};
                    border-radius: 5px;
                    background-color: {bg_color};
                    padding: 5px;
                }}
            ''')


class DisconnectedView(QWidget):
    on_connection = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.wait_for_connection: bool = False
        self.setup_ui()

    def setup_ui(self):
        # Define styles matching the other widgets
        button_style = """
            QPushButton {
                min-height: 50px;
                font-size: 18px;
                padding: 15px 25px;
                font-weight: bold;
                border: 3px solid #2c3e50;
                border-radius: 10px;
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

        label_style = """
            QLabel {
                font-size: 20px;
                font-weight: bold;
                color: #2c3e50;
                padding: 10px;
                min-height: 50px;
            }
        """

        # Create layout with proper spacing for large screens
        layout = QVBoxLayout(self)
        layout.setSpacing(20)  # Larger spacing for big screens
        layout.setContentsMargins(40, 40, 40, 40)  # Larger margins

        # Create and style the service state label
        self.service_state = QLabel('Usługa: Niedostępna')
        self.service_state.setStyleSheet(label_style)
        self.service_state.setAlignment(Qt.AlignCenter)
        
        # Create and style the connect button
        self.connect_button = QPushButton('Połącz')
        self.connect_button.setStyleSheet(button_style)
        self.connect_button.clicked.connect(self.on_connect)
        self.connect_button.setEnabled(False)

        # Set fonts for large screens
        font = QFont()
        font.setPointSize(18)  # Larger font size for 1920x1080
        self.service_state.setFont(font)
        self.connect_button.setFont(font)

        # Add widgets to layout with some spacing
        layout.addStretch()
        layout.addWidget(self.service_state)
        layout.addWidget(self.connect_button)
        layout.addStretch()

    def on_connect(self):
        self.on_connection.emit('connect on Widget')

    def set_availability(self, available):
        if not self.wait_for_connection:
            status = 'Dostępna' if available else 'Niedostępna'
            self.service_state.setText(f'Service: {status}')
            self.connect_button.setEnabled(available)

    def waiting_for_connection(self, state: bool):
        if state:
            self.connect_button.setEnabled(False)
            self.service_state.setText('Usługa: łączenie')
            self.wait_for_connection = state

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

        self.active_tools = ActiveTools(self.routes, self.maps)
        self.planning_tools = PlanningTools(self.routes)

        self.stacked_widget = QStackedWidget()
        # self.layout.addWidget(self.stacked_widget)

        self.stacked_widget.addWidget(self.active_tools)
        self.stacked_widget.addWidget(self.planning_tools)

        self.main_layout = QHBoxLayout()
        self.main_layout.addWidget(self.map_view, 3)
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

    pyqtSlot(dict,bool)
    def handle_set_route(self, routes, to_dest):
        self.start_nav.emit(routes, to_dest, self.curr_x, self.curr_y)

    pyqtSlot()
    def update_robot_pose(self, x, y, theta):
        self.map_view.update_robot_pose(x,y,theta)
        self.curr_x = x
        self.curr_y = y

    pyqtSlot(dict)
    def load__routes(self, routes):
        self.routes.clear()
        self.routes.update(routes) 
        self.active_tools.update_routes()

    pyqtSlot(list)
    def load_maps(self, maps: list):
        self.maps = maps.copy()  # Make a copy of the list
        self.active_tools.update_maps(self.maps)  # Update maps in ActiveTools

    def switch_to_active_tools(self):
        self.stacked_widget.setCurrentWidget(self.active_tools)
        
        self.active_tools.on_disconnect.connect(lambda: self.on_disconnection.emit())
        self.active_tools.start_planning.connect(lambda: self.start_planning.emit())

    def switch_to_configuring_tab(self):
        self.active_tools.tab_widget.setCurrentIndex(1)

    def switch_to_active(self): #OPTIMIZE
        self.active_tools.tab_widget.setCurrentIndex(0)

    def switch_to_planning(self):
        self.stacked_widget.setCurrentWidget(self.planning_tools)
        
        # Start planning mode
        self.planning_tools.start_planning_mode()

        # Connect signals
        self.planning_tools.finish_planning.connect(lambda: self.finish_planning.emit())
        self.planning_tools.get_current_route.connect(self.handle_get_current_route)
        
    def handle_get_current_route(self):
        """Get current route from map view and save it"""
        current_route = self.map_view.get_current_route()
        if current_route:
            self.planning_tools.save_route(current_route)



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
            QPushButton:disabled {
                background-color: #f5f6f7;
                border-color: #bdc3c7;
                color: #95a5a6;
            }
        """

        label_style = """
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: #2c3e50;
                padding: 5px;
                min-height: 30px;
            }
        """

        # Create layout with proper spacing
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(20, 20, 20, 20)

        # Create and style the service state label
        self.service_state = QLabel("Service: Not Available")
        self.service_state.setStyleSheet(label_style)
        self.service_state.setAlignment(Qt.AlignCenter)
        
        # Create and style the connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.setStyleSheet(button_style)
        self.connect_button.clicked.connect(self.on_connect)
        self.connect_button.setEnabled(False)

        # Set fonts
        font = QFont()
        font.setPointSize(12)
        self.service_state.setFont(font)
        self.connect_button.setFont(font)

        # Add widgets to layout with some spacing
        layout.addStretch()
        layout.addWidget(self.service_state)
        layout.addWidget(self.connect_button)
        layout.addStretch()

    def on_connect(self):
        self.on_connection.emit("connect on Widget")

    def set_availability(self, available):
        if not self.wait_for_connection:
            status = "Available" if available else "Not Available"
            self.service_state.setText(f"Service: {status}")
            self.connect_button.setEnabled(available)

    def waiting_for_connection(self, state: bool):
        if state:
            self.connect_button.setEnabled(False)
            self.service_state.setText("Service: connecting")
            self.wait_for_connection = state
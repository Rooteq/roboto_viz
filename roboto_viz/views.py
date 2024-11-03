from PyQt5.QtWidgets import QStackedWidget, QVBoxLayout, QWidget, QPushButton, QTabWidget, QLineEdit, QLabel, QHBoxLayout, QListWidget, QListWidgetItem, QComboBox, QGridLayout
from PyQt5.QtCore import Qt, pyqtSignal, QObject, pyqtSlot
from roboto_viz.view_tools import ActiveTools, PlanningTools

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

        # self.stacked_widget.setCurrentWidget(self.planning_tools)
        self.planning_tools.finish_planning.connect(lambda: self.finish_planning.emit())


class DisconnectedView(QWidget):
    on_connection = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        # self.main_view = main_view
        self.setup_ui()
        self.wait_for_connection: bool = False

    def setup_ui(self):
        layout = QVBoxLayout(self)

        self.service_state = QLabel("Service: Not Available")
        
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.on_connect)
        self.connect_button.setEnabled(False)

        layout.addWidget(self.service_state)
        layout.addWidget(self.connect_button)

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


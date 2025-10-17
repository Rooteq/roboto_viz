from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QPainter
from roboto_viz.robot_item import RobotItem
from roboto_viz.bezier_graphics import BezierRouteGraphics
import math


class MiniMapView(QGraphicsView):
    """
    A mini map view that shows the current map with robot position and route.
    Always centered on the robot position.
    """

    def __init__(self):
        super().__init__()

        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Disable scrollbars
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Enable antialiasing for smoother rendering
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)

        # Disable interaction - this is view-only
        self.setInteractive(False)

        # Map properties
        self.map_pixmap_item = None
        self.pixmap = None
        self.origin = [0, 0, 0]
        self.resolution = 0.05

        # Robot item
        self.robot_item = RobotItem()
        self.scene.addItem(self.robot_item)
        self.robot_item.setVisible(False)

        # Current robot position
        self.robot_x = 0
        self.robot_y = 0

        # Bezier route graphics
        self.current_route_graphics = None

        # Styling
        self.setStyleSheet("""
            QGraphicsView {
                border: none;
                background-color: #ecf0f1;
            }
        """)

    def load_image(self, image_path: str, origin: list, resolution: float = 0.05):
        """Load a map image into the mini map view"""
        self.origin = origin
        self.resolution = resolution

        # Clear existing map
        if self.map_pixmap_item:
            self.scene.removeItem(self.map_pixmap_item)

        # Load and add new map
        self.pixmap = QPixmap(image_path)
        self.map_pixmap_item = self.scene.addPixmap(self.pixmap)
        self.map_pixmap_item.setZValue(-1)  # Ensure map is behind robot

        # Position pixmap at (0, 0) same as map_view - BezierRouteGraphics assumes this
        self.map_pixmap_item.setPos(0, 0)

        # Update scene rectangle to match image
        self.scene.setSceneRect(self.pixmap.rect())

        # Center on robot if available, otherwise center on map origin
        if self.robot_item.isVisible():
            self.center_on_robot()
        else:
            # Center on map origin initially
            self.centerOn(0, 0)
            self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)

    def update_robot_pose(self, x: float, y: float, theta: float):
        """Update robot position and center view on it
        Args:
            x, y: position in meters (world coordinates)
            theta: orientation in radians
        """
        self.robot_x = x
        self.robot_y = y

        # Convert world coordinates to scene coordinates (same as BezierRouteGraphics)
        map_x = (x - self.origin[0]) * 20
        map_y = self.pixmap.height() - ((y - self.origin[1]) * 20)

        # Update robot item position and rotation
        # Note: theta is in radians, need to convert to degrees for Qt
        self.robot_item.setPos(map_x, map_y)
        self.robot_item.setRotation(math.degrees(theta))  # Convert radians to degrees
        self.robot_item.setVisible(True)

        # Always center on robot
        self.center_on_robot()

    def center_on_robot(self):
        """Center the view on the robot position"""
        # Convert world coordinates to scene coordinates (same as BezierRouteGraphics uses)
        map_x = (self.robot_x - self.origin[0]) * 20
        map_y = self.pixmap.height() - ((self.robot_y - self.origin[1]) * 20)

        self.centerOn(map_x, map_y)

        # Adjust zoom to show reasonable area around robot
        # Fit approximately 6 meters around the robot (2x zoomed out from before)
        view_range = 6.0  # meters (zoomed out 2x from previous 3.0)
        scene_range = view_range * 20  # Convert to scene units (20 pixels per meter)

        # Calculate scale to fit the desired range
        view_width = self.viewport().width()
        view_height = self.viewport().height()

        if view_width > 0 and view_height > 0:
            scale_x = view_width / scene_range
            scale_y = view_height / scene_range
            scale = min(scale_x, scale_y) * 0.7  # 0.7 for some padding

            self.resetTransform()
            self.scale(scale, scale)

    def display_bezier_route(self, route):
        """Display a bezier route on the mini map"""
        print(f"DEBUG: MiniMapView.display_bezier_route called with route: {route}")
        # Clear existing route
        self.clear_route()

        if route is None:
            print("DEBUG: Route is None, skipping display")
            return

        # Check if we have a map loaded
        if self.pixmap and self.origin:
            print(f"DEBUG: Creating BezierRouteGraphics for mini map with origin: {self.origin}")
            print(f"DEBUG: Pixmap height: {self.pixmap.rect().height()}")
            # Create graphics for the route - same parameters as map_view
            self.current_route_graphics = BezierRouteGraphics(
                route, self.origin, self.pixmap.rect().height(), self.scene
            )
            print(f"DEBUG: Route graphics created: {self.current_route_graphics}")
        else:
            print(f"DEBUG: Cannot display route - pixmap: {self.pixmap is not None}, origin: {self.origin}")

    def clear_route(self):
        """Clear the displayed route"""
        if self.current_route_graphics:
            self.current_route_graphics.clear_graphics()  # Use correct method name
            self.current_route_graphics = None

    def resizeEvent(self, event):
        """Handle resize events by re-centering on robot"""
        super().resizeEvent(event)
        if self.robot_item.isVisible():
            self.center_on_robot()

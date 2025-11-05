from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PyQt5.QtCore import Qt, QRectF, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QPainter, QColor, QImage, qRgba
from roboto_viz.robot_item import RobotItem
from roboto_viz.bezier_graphics import BezierRouteGraphics
import math
from pathlib import Path
import time


class CollisionZoneFilterWorker(QThread):
    """Background thread worker to filter collision zones without blocking UI"""
    finished = pyqtSignal(QPixmap)  # Emits the filtered pixmap when done
    
    def __init__(self, collision_image, route_zone_colors):
        super().__init__()
        self.collision_image = collision_image
        self.route_zone_colors = route_zone_colors
    
    def run(self):
        """Filter collision zones in background thread"""
        start_time = time.time()
        
        # Create filtered image showing only zones for this route
        filtered_image = QImage(self.collision_image.size(), QImage.Format_ARGB32)
        filtered_image.fill(Qt.transparent)

        # Copy only pixels that match route zone colors
        for y in range(self.collision_image.height()):
            for x in range(self.collision_image.width()):
                pixel_color = QColor(self.collision_image.pixel(x, y))
                pixel_rgb = (pixel_color.red(), pixel_color.green(), pixel_color.blue())

                # If pixel color matches a zone for this route, copy it
                if pixel_rgb in self.route_zone_colors:
                    filtered_image.setPixel(x, y, pixel_color.rgba())

        # Convert to pixmap
        filtered_pixmap = QPixmap.fromImage(filtered_image)
        
        elapsed = time.time() - start_time
        print(f"PERF: Filtered collision zones in {elapsed:.3f}s (background thread)")
        
        # Emit result
        self.finished.emit(filtered_pixmap)


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

        # Collision zones overlay
        self.collision_zones_item = None
        
        # Background worker for collision zone filtering
        self.filter_worker = None

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
        self.scene.setSceneRect(QRectF(self.pixmap.rect()))

        # Don't auto-load collision zones - they will be loaded when navigation starts
        # Clear any existing collision zones
        self.clear_collision_zones()

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
        # Check if origin and pixmap are properly initialized
        if not self.origin or len(self.origin) < 2:
            return

        if not self.pixmap:
            return

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
        view_range = 12.0  # meters (zoomed out 2x from previous 3.0)
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

    def clear_collision_zones(self):
        """Clear the collision zones overlay"""
        if self.collision_zones_item:
            self.scene.removeItem(self.collision_zones_item)
            self.collision_zones_item = None

    def show_collision_zones_for_route(self, route_name: str, color_cache: dict = None):
        """Load and display collision zones for a specific route as a transparent overlay

        Uses the SAME simple method as collision zone editor: loads the collision_mapname.png
        directly and filters it to show only zones for this route.

        Args:
            route_name: Name of the route to display zones for
            color_cache: Unused (kept for signal compatibility), we load the image directly
        """
        # Note: color_cache parameter kept for PyQt signal compatibility but not used
        # We simply load the collision map image directly instead
        print(f"INFO: MiniMapView.show_collision_zones_for_route called for route '{route_name}'")

        # Remove existing collision zones overlay
        self.clear_collision_zones()

        if not self.pixmap:
            print(f"WARNING: Cannot show collision zones - no map loaded")
            return

        maps_dir = Path.home() / ".robotroutes" / "maps"

        # Find which map is currently loaded by matching dimensions
        collision_map_files = list(maps_dir.glob("collision_*.png"))
        if not collision_map_files:
            collision_map_files = list(maps_dir.glob("collision_*.pgm"))

        collision_map_path = None
        map_name = None
        for file in collision_map_files:
            filename = file.stem  # collision_<mapname>
            if not filename.endswith("_zones"):
                # Extract map name
                map_name_candidate = filename.replace("collision_", "")
                # Check if this matches our current pixmap dimensions
                test_pixmap = QPixmap(str(file))
                if test_pixmap.size() == self.pixmap.size():
                    collision_map_path = file
                    map_name = map_name_candidate
                    break

        if not collision_map_path or not map_name:
            print(f"INFO: Could not find collision map matching current map dimensions")
            return

        # Check if zones exist for this map
        collision_zones_json = maps_dir / f"collision_{map_name}_zones.json"
        if not collision_zones_json.exists():
            print(f"INFO: No collision zones file found for map '{map_name}'")
            return

        # Load zone definitions to get colors for this route
        try:
            import json
            with open(collision_zones_json, 'r') as f:
                zones_data = json.load(f)
        except Exception as e:
            print(f"ERROR: Failed to load zones JSON: {e}")
            return

        if not zones_data:
            print(f"INFO: No zones defined for map '{map_name}'")
            return

        # Extract colors for zones belonging to this route
        route_zone_colors = set()
        for zone_id_str, zone_data in zones_data.items():
            zone_routes = zone_data.get('route_names', [])
            if route_name not in zone_routes:
                continue

            color = zone_data.get('color', [])
            if len(color) == 3:
                route_zone_colors.add(tuple(color))  # (R, G, B)

        if not route_zone_colors:
            print(f"INFO: No zones found for route '{route_name}' on map '{map_name}'")
            return

        # Load the collision map directly (SAME AS COLLISION ZONE EDITOR)
        collision_pixmap = QPixmap(str(collision_map_path))
        collision_image = collision_pixmap.toImage()

        print(f"INFO: Starting collision zone filtering in background thread (won't block UI)...")
        
        # Cancel any existing filter worker
        if self.filter_worker and self.filter_worker.isRunning():
            self.filter_worker.finished.disconnect()
            self.filter_worker.terminate()
            self.filter_worker.wait()
        
        # Start background filtering
        self.filter_worker = CollisionZoneFilterWorker(collision_image, route_zone_colors)
        self.filter_worker.finished.connect(lambda pixmap: self._apply_filtered_zones(pixmap, route_name, len(route_zone_colors)))
        self.filter_worker.start()
    
    def _apply_filtered_zones(self, filtered_pixmap: QPixmap, route_name: str, num_zones: int):
        """Apply the filtered collision zones to the scene (called from background thread)"""
        # Make it semi-transparent (30% opacity)
        transparent_pixmap = QPixmap(filtered_pixmap.size())
        transparent_pixmap.fill(Qt.transparent)

        opacity_painter = QPainter(transparent_pixmap)
        opacity_painter.setOpacity(0.3)  # 30% opacity
        opacity_painter.drawPixmap(0, 0, filtered_pixmap)
        opacity_painter.end()

        # Add to scene at (0, 0) - same position as map
        self.collision_zones_item = self.scene.addPixmap(transparent_pixmap)
        self.collision_zones_item.setPos(0, 0)
        self.collision_zones_item.setZValue(0)  # Above map, below robot

        print(f"✓ SUCCESS: Displayed {num_zones} collision zone(s) for route '{route_name}' on mini map")

    def resizeEvent(self, event):
        """Handle resize events by re-centering on robot"""
        super().resizeEvent(event)
        if self.robot_item.isVisible():
            self.center_on_robot()

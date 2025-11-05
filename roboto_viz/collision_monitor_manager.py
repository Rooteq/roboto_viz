"""
Collision Monitor Manager for handling ROS2 collision monitor parameters.
This manager monitors the robot's position and dynamically updates collision
monitor polygon parameters based on painted zones on the map.
"""

from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QColor, QImage
from pathlib import Path
import json
from typing import Optional, Dict, List, Tuple
from rclpy.node import Node
import threading
import numpy as np


class CollisionZone:
    """Represents a collision zone with polygon points and painted regions"""

    def __init__(self, zone_id: int, polygon_points: str, color: List[int],
                 use_polygon_slow: bool = True, use_polygon_stop: bool = False,
                 painted_pixels: set = None, route_names: List[str] = None,
                 numpy_cache: dict = None):
        self.zone_id = zone_id
        self.polygon_points = polygon_points
        self.color = tuple(color)  # RGB tuple
        self.use_polygon_slow = use_polygon_slow
        self.use_polygon_stop = use_polygon_stop
        self.painted_pixels = painted_pixels or set()  # Legacy: Set of (x, y) pixel coordinates
        self.route_names = route_names or []  # List of route names this zone is associated with
        
        # NEW: numpy-based cache for ULTRA-FAST lookups
        self.numpy_cache = numpy_cache  # Dict with 'numpy_array', 'width', 'height'

    def contains_pixel(self, pixel_x: int, pixel_y: int) -> bool:
        """Check if this zone contains the given pixel - ULTRA-FAST numpy lookup"""
        # Use numpy cache if available (MUCH faster than set lookup)
        if self.numpy_cache:
            arr = self.numpy_cache.get('numpy_array')
            height = self.numpy_cache.get('height', 0)
            width = self.numpy_cache.get('width', 0)
            
            # Bounds check
            if arr is None or pixel_x < 0 or pixel_y < 0 or pixel_x >= width or pixel_y >= height:
                return False
            
            # Direct numpy array lookup: O(1) operation!
            pixel_color = arr[pixel_y, pixel_x]  # [R, G, B]
            
            # Compare with zone color
            matches = (pixel_color[0] == self.color[0] and
                      pixel_color[1] == self.color[1] and
                      pixel_color[2] == self.color[2])
            
            return matches
        
        # Fallback to legacy set lookup (slower)
        return (pixel_x, pixel_y) in self.painted_pixels

    def load_painted_pixels_from_image(self, pixmap: QPixmap):
        """Scan the collision mask image and find all pixels of this zone's color - DEPRECATED, use load_from_cache"""
        # This method is kept for backwards compatibility but should not be used
        # Use CollisionMonitorManager._build_color_cache() and load_from_cache() instead
        print(f"WARNING: Using slow pixel-by-pixel scan for zone {self.zone_id}. Use load_from_cache() instead.")
        image = pixmap.toImage()
        self.painted_pixels = set()

        # Scan entire image for pixels matching this zone's color
        for y in range(image.height()):
            for x in range(image.width()):
                pixel_color = QColor(image.pixel(x, y))
                if (pixel_color.red() == self.color[0] and
                    pixel_color.green() == self.color[1] and
                    pixel_color.blue() == self.color[2]):
                    self.painted_pixels.add((x, y))

    def load_from_cache(self, color_cache: dict):
        """Load from numpy-based cache (ULTRA-FAST - no pixel scanning needed!)"""
        # Store reference to numpy cache for instant pixel lookups
        self.numpy_cache = color_cache
        # Clear legacy painted_pixels set (not needed anymore)
        self.painted_pixels = set()



class CollisionMonitorManager(QObject):
    """
    Manager for collision monitor zones that updates ROS2 parameters
    based on the robot's position on the collision zone map.
    """

    collision_zone_changed = pyqtSignal(int)  # Emits zone_id when robot enters a zone
    collision_zone_cleared = pyqtSignal()  # Emits when robot exits all zones

    # ========================================================================
    # DEFAULT COLLISION POLYGONS - CHANGE THESE VALUES AS NEEDED
    # These are the default polygons that will be used when the robot exits zones
    # ========================================================================
    DEFAULT_POLYGON_SLOW_POINTS = "[[1.30, 0.55], [1.30, -0.55], [-0.8, -0.55], [-0.8, 0.55]]"
    DEFAULT_POLYGON_STOP_POINTS = "[[1.0, 0.45], [1.0, -0.45], [-0.8, -0.45], [-0.8, 0.45]]"

    def __init__(self, node: Node, parent=None):
        super().__init__(parent)
        self.node = node
        self.current_map_name: Optional[str] = None
        self.collision_mask_pixmap: Optional[QPixmap] = None
        self.zones: Dict[int, CollisionZone] = {}  # zone_id -> CollisionZone object
        self.current_zone_id: Optional[int] = None
        self.map_origin = None
        self.map_resolution = None
        self.enabled = False
        self.initial_check_done = False  # Track if we've done initial zone check

        # Track which polygon types are currently modified (not at default)
        self.slow_polygon_active = False  # True if PolygonSlow is set to non-default
        self.stop_polygon_active = False  # True if PolygonStop is set to non-default

        # Timer to check robot position periodically
        self.check_timer = QTimer()
        self.check_timer.timeout.connect(self.check_robot_position)
        self.check_timer.setInterval(200)  # Check every 200ms

        # Store robot pose
        self.robot_x = None
        self.robot_y = None

        # Debug counter for position checks
        self.check_counter = 0

        # Cache for fast collision zone pixel lookup: color (R,G,B) -> set of (x,y) pixels
        # This is built once per map and reused for all zones
        self.color_cache: Dict[Tuple[int, int, int], set] = {}

    def set_current_map(self, map_name: str, origin, resolution):
        """Set the current map - zones will be loaded when route navigation starts"""
        self.current_map_name = map_name
        self.map_origin = origin
        self.map_resolution = resolution
        self.initial_check_done = False  # Reset for new map

        if not map_name:
            self.enabled = False
            self.check_timer.stop()
            return

        # Load the collision mask image (without zones yet)
        maps_dir = Path.home() / ".robotroutes" / "maps"
        collision_map_path = maps_dir / f"collision_{map_name}.png"
        if not collision_map_path.exists():
            collision_map_path = maps_dir / f"collision_{map_name}.pgm"

        if collision_map_path.exists():
            self.collision_mask_pixmap = QPixmap(str(collision_map_path))
            # Clear color cache when map changes
            self.color_cache = {}
        else:
            self.collision_mask_pixmap = None
            self.color_cache = {}

    def _build_color_cache(self):
        """Build color cache from collision mask - ULTRA-FAST: just store the numpy image array"""
        if not self.collision_mask_pixmap:
            return

        # Check if cache already built
        if self.color_cache:
            return

        import time
        start_time = time.time()

        # Convert QPixmap to QImage to numpy array (MUCH faster than pixel-by-pixel access)
        image = self.collision_mask_pixmap.toImage()

        # Convert to format that numpy can read efficiently
        image = image.convertToFormat(QImage.Format_RGB888)

        width = image.width()
        height = image.height()
        ptr = image.bits()
        ptr.setsize(image.sizeInBytes())

        # Get bytes per line (stride) - this is CRITICAL for correct image reading!
        bytes_per_line = image.bytesPerLine()
        
        # Create numpy array from image data with proper stride handling
        # QImage may have padding at the end of each row, so we need to use bytesPerLine
        if bytes_per_line == width * 3:
            # No padding - can reshape directly
            arr = np.frombuffer(ptr, np.uint8).reshape((height, width, 3)).copy()
        else:
            # Has padding - need to handle stride properly
            arr_with_padding = np.frombuffer(ptr, np.uint8).reshape((height, bytes_per_line))
            # Extract only the actual image data (remove padding)
            arr = arr_with_padding[:, :width*3].reshape((height, width, 3)).copy()
        
        # Store the numpy array itself as the cache
        # Pixel lookup becomes: arr[y, x] = [R, G, B]
        self.color_cache = {
            'numpy_array': arr,
            'width': width,
            'height': height
        }

        elapsed = time.time() - start_time
        print(f"PERF: Built numpy image cache in {elapsed:.3f}s ({width}x{height} pixels = {width*height:,} total)")

        # Return the cache for use by mini_map_view
        return self.color_cache

    def get_color_cache(self) -> Dict[Tuple[int, int, int], set]:
        """Get the color cache (builds it if not already built)"""
        if not self.color_cache:
            self._build_color_cache()
        return self.color_cache

    def load_collision_zones_for_route(self, route_name: str):
        """Load collision zones for the current map, filtering by route name"""
        if not self.current_map_name:
            print(f"WARNING: No map loaded, cannot load collision zones")
            return

        maps_dir = Path.home() / ".robotroutes" / "maps"
        collision_zones_path = maps_dir / f"collision_{self.current_map_name}_zones.json"

        # Clear existing zones
        self.zones = {}

        # Check if collision mask exists
        if not self.collision_mask_pixmap:
            print(f"WARNING: No collision mask loaded, cannot load zones for map '{self.current_map_name}'")
            return

        # Load zone definitions for this map
        if not collision_zones_path.exists():
            print(f"INFO: No collision zones file found for map '{self.current_map_name}'")
            return

        try:
            import time
            start_time = time.time()

            # Build color cache ONCE for all zones (fast numpy operation)
            self._build_color_cache()

            with open(collision_zones_path, 'r') as f:
                zones_data = json.load(f)

            self.zones = {}
            zones_for_route = 0
            for zone_id_str, zone_data in zones_data.items():
                zone_id = int(zone_id_str)

                # Check if this zone applies to the current route
                route_names = zone_data.get('route_names', [])
                if route_name not in route_names:
                    # Skip zones that don't apply to this route
                    continue

                zones_for_route += 1
                zone = CollisionZone(
                    zone_id=zone_id,
                    polygon_points=zone_data['polygon_points'],
                    color=zone_data['color'],
                    use_polygon_slow=zone_data.get('use_polygon_slow', True),
                    use_polygon_stop=zone_data.get('use_polygon_stop', False),
                    route_names=route_names
                )
                # Load painted pixels from cache (O(1) lookup instead of O(width*height) scan)
                zone.load_from_cache(self.color_cache)
                self.zones[zone_id] = zone

            elapsed = time.time() - start_time
            print(f"INFO: Loaded {zones_for_route} collision zones for route '{route_name}' (from map '{self.current_map_name}') in {elapsed:.3f}s")

        except Exception as e:
            print(f"ERROR: Failed to load collision zones for route '{route_name}': {e}")
            import traceback
            traceback.print_exc()
            self.zones = {}

    def start_monitoring(self):
        """Start monitoring robot position for collision zones"""
        if self.zones:
            self.enabled = True
            # Reset initial check flag so we check the starting position
            self.initial_check_done = False
            print(f"DEBUG: start_monitoring() - robot pose: x={self.robot_x}, y={self.robot_y}")
            
            # Start periodic checking first
            self.check_timer.start()
            
            # Schedule initial check with a small delay to ensure:
            # 1. Latest robot pose is received
            # 2. All initialization is complete
            # Use QTimer.singleShot for delayed check
            if self.robot_x is not None and self.robot_y is not None:
                print(f"DEBUG: Scheduling delayed initial position check (500ms delay)")
                QTimer.singleShot(500, self._delayed_initial_check)
            else:
                print(f"WARNING: Robot position not available yet, will check on first pose update")
        else:
            self.enabled = False
    
    def _delayed_initial_check(self):
        """Perform initial position check after a short delay"""
        print(f"DEBUG: Performing delayed initial check - robot pose: x={self.robot_x}, y={self.robot_y}")
        if self.robot_x is not None and self.robot_y is not None:
            self.check_robot_position()
        else:
            print(f"WARNING: Robot position STILL not available after delay")

    def stop_monitoring(self):
        """Stop monitoring robot position"""
        self.enabled = False
        self.check_timer.stop()
        # Reset initial check flag for next navigation
        self.initial_check_done = False

    def update_robot_pose(self, x: float, y: float):
        """Update the robot's current pose"""
        self.robot_x = x
        self.robot_y = y

    def check_robot_position(self):
        """Check if robot is in a collision zone and update parameters accordingly"""
        self.check_counter += 1

        if not self.enabled:
            return

        if self.robot_x is None or self.robot_y is None:
            return

        if not self.collision_mask_pixmap or not self.zones:
            return

        # Convert robot world coordinates to map pixel coordinates
        pixel_x, pixel_y = self.world_to_pixel(self.robot_x, self.robot_y)

        # Check if pixel is within bounds
        if (pixel_x < 0 or pixel_y < 0 or
            pixel_x >= self.collision_mask_pixmap.width() or
            pixel_y >= self.collision_mask_pixmap.height()):
            return

        # Check which zone the robot is in (if any)
        zone_id = self.get_zone_at_pixel(pixel_x, pixel_y)

        # On first check, if robot starts in a zone, apply that zone's polygon immediately
        if not self.initial_check_done:
            self.initial_check_done = True
            
            if zone_id is not None:
                print(f"\n{'='*70}")
                print(f"INITIAL POSITION: Robot starts in collision zone {zone_id}")
                print(f"{'='*70}\n")
                self.current_zone_id = zone_id
                self.enter_zone(zone_id, pixel_x, pixel_y)
                return
            else:
                return

        # If zone changed, update parameters
        if zone_id != self.current_zone_id:
            if zone_id is not None:
                # Entered a zone (or moved to a different zone)
                self.enter_zone(zone_id, pixel_x, pixel_y)
            else:
                # Exited all zones (only if we were previously in a zone)
                if self.current_zone_id is not None:
                    self.exit_zone()

            self.current_zone_id = zone_id

    def world_to_pixel(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to pixel coordinates (same as mini_map_view.py)"""
        if not self.map_origin or not self.collision_mask_pixmap:
            return (0, 0)

        # Map origin is [x, y, theta]
        origin_x = self.map_origin[0]
        origin_y = self.map_origin[1]

        # Convert to pixel coordinates (same formula as mini_map_view.py lines 96-97)
        # Uses 20 pixels per meter scale factor (not 1/resolution)
        pixel_x = int((world_x - origin_x) * 20)
        pixel_y = int(self.collision_mask_pixmap.height() - ((world_y - origin_y) * 20))

        return (pixel_x, pixel_y)

    def get_zone_at_pixel(self, pixel_x: int, pixel_y: int) -> Optional[int]:
        """Get the zone ID at a specific pixel position by checking painted regions"""
        for zone_id, zone in self.zones.items():
            if zone.contains_pixel(pixel_x, pixel_y):
                return zone_id
        return None

    def enter_zone(self, zone_id: int, pixel_x: int, pixel_y: int):
        """Called when robot enters a collision zone"""
        if zone_id not in self.zones:
            return

        zone = self.zones[zone_id]
        polygon_points = zone.polygon_points

        # Build list of polygon types to set
        polygon_types = []
        if zone.use_polygon_slow:
            polygon_types.append("PolygonSlow")
        if zone.use_polygon_stop:
            polygon_types.append("PolygonStop")

        print(f"\n{'='*70}")
        print(f"ROBOT ENTERED COLLISION ZONE {zone_id}")
        print(f"{'='*70}")
        print(f"Robot world position: ({self.robot_x:.3f}, {self.robot_y:.3f})")
        print(f"Robot pixel position: ({pixel_x}, {pixel_y})")
        print(f"Polygon types: {' + '.join(polygon_types)}")
        print(f"Setting polygon points: {polygon_points}")
        print(f"{'='*70}\n")

        # Handle PolygonSlow transition
        if zone.use_polygon_slow:
            # This zone modifies PolygonSlow - set it
            self.set_collision_polygon(polygon_points, "PolygonSlow")
            self.slow_polygon_active = True
        else:
            # This zone doesn't modify PolygonSlow
            # If it was previously modified, restore to default
            if self.slow_polygon_active:
                self.set_collision_polygon(self.DEFAULT_POLYGON_SLOW_POINTS, "PolygonSlow")
                self.slow_polygon_active = False

        # Handle PolygonStop transition
        if zone.use_polygon_stop:
            # This zone modifies PolygonStop - set it
            self.set_collision_polygon(polygon_points, "PolygonStop")
            self.stop_polygon_active = True
        else:
            # This zone doesn't modify PolygonStop
            # If it was previously modified, restore to default
            if self.stop_polygon_active:
                self.set_collision_polygon(self.DEFAULT_POLYGON_STOP_POINTS, "PolygonStop")
                self.stop_polygon_active = False

        # Emit signal
        self.collision_zone_changed.emit(zone_id)

    def exit_zone(self):
        """Called when robot exits all collision zones"""
        print(f"\n{'='*70}")
        print(f"ROBOT EXITED ALL COLLISION ZONES")
        print(f"{'='*70}")
        print(f"Robot position: world=({self.robot_x:.3f}, {self.robot_y:.3f})")

        # Restore polygons that were modified
        if self.slow_polygon_active:
            print(f"Restoring PolygonSlow to default: {self.DEFAULT_POLYGON_SLOW_POINTS}")
            self.set_collision_polygon(self.DEFAULT_POLYGON_SLOW_POINTS, "PolygonSlow")
            self.slow_polygon_active = False

        if self.stop_polygon_active:
            print(f"Restoring PolygonStop to default: {self.DEFAULT_POLYGON_STOP_POINTS}")
            self.set_collision_polygon(self.DEFAULT_POLYGON_STOP_POINTS, "PolygonStop")
            self.stop_polygon_active = False

        print(f"{'='*70}\n")

        # Emit signal
        self.collision_zone_cleared.emit()

    def set_collision_polygon(self, polygon_points: str, polygon_type: str = "PolygonSlow"):
        """Set the collision monitor polygon parameter via ROS2 CLI in background thread"""
        # Run in background thread to prevent GUI freezing
        thread = threading.Thread(
            target=self._set_collision_polygon_thread,
            args=(polygon_points, polygon_type),
            daemon=True
        )
        thread.start()

    def _set_collision_polygon_thread(self, polygon_points: str, polygon_type: str):
        """Background thread function to set collision polygon parameter"""
        try:
            import subprocess

            cmd = [
                'ros2', 'param', 'set',
                '/collision_monitor',
                f'{polygon_type}.points',
                polygon_points
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                print(f"✓ Successfully set {polygon_type} polygon parameter")
                if result.stdout and result.stdout.strip():
                    print(f"  ROS2 response: {result.stdout.strip()}")
            else:
                print(f"✗ Failed to set {polygon_type} polygon parameter (non-critical, continuing)")
                if result.stderr:
                    print(f"  Error: {result.stderr.strip()}")

        except subprocess.TimeoutExpired:
            print(f"✗ ROS2 param set command timed out after 10 seconds (non-critical, continuing)")
        except FileNotFoundError:
            print(f"✗ ROS2 command not found - is ROS2 installed? (non-critical, continuing)")
        except Exception as e:
            print(f"✗ Exception setting collision polygon (non-critical, continuing): {e}")

"""
Collision Monitor Manager for handling ROS2 collision monitor parameters.
This manager monitors the robot's position and dynamically updates collision
monitor polygon parameters based on painted zones on the map.
"""

from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QColor
from pathlib import Path
import json
from typing import Optional, Dict, List, Tuple
from rclpy.node import Node


class CollisionZone:
    """Represents a collision zone with polygon points and painted regions"""

    def __init__(self, zone_id: int, polygon_points: str, color: List[int], painted_pixels: set = None):
        self.zone_id = zone_id
        self.polygon_points = polygon_points
        self.color = tuple(color)  # RGB tuple
        self.painted_pixels = painted_pixels or set()  # Set of (x, y) pixel coordinates for O(1) lookup

    def contains_pixel(self, pixel_x: int, pixel_y: int) -> bool:
        """Check if this zone contains the given pixel"""
        return (pixel_x, pixel_y) in self.painted_pixels

    def load_painted_pixels_from_image(self, pixmap: QPixmap):
        """Scan the collision mask image and find all pixels of this zone's color"""
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


class CollisionMonitorManager(QObject):
    """
    Manager for collision monitor zones that updates ROS2 parameters
    based on the robot's position on the collision zone map.
    """

    collision_zone_changed = pyqtSignal(int)  # Emits zone_id when robot enters a zone
    collision_zone_cleared = pyqtSignal()  # Emits when robot exits all zones

    # ========================================================================
    # DEFAULT COLLISION POLYGON - CHANGE THIS VALUE AS NEEDED
    # This is the default polygon that will be used when the robot exits all zones
    # ========================================================================
    DEFAULT_POLYGON_POINTS = "[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]"

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

        # Timer to check robot position periodically
        self.check_timer = QTimer()
        self.check_timer.timeout.connect(self.check_robot_position)
        self.check_timer.setInterval(200)  # Check every 200ms

        # Store robot pose
        self.robot_x = None
        self.robot_y = None

        # Debug counter for position checks
        self.check_counter = 0

    def set_current_map(self, map_name: str, origin, resolution):
        """Set the current map and load collision zones"""
        self.current_map_name = map_name
        self.map_origin = origin
        self.map_resolution = resolution
        self.initial_check_done = False  # Reset for new map

        if not map_name:
            self.enabled = False
            self.check_timer.stop()
            return

        # Load collision zones for this map
        self.load_collision_zones(map_name)

    def load_collision_zones(self, map_name: str):
        """Load collision zones from the map files"""
        maps_dir = Path.home() / ".robotroutes" / "maps"

        # Try PNG first (color format), fallback to PGM (grayscale)
        collision_map_path = maps_dir / f"collision_{map_name}.png"
        if not collision_map_path.exists():
            collision_map_path = maps_dir / f"collision_{map_name}.pgm"

        collision_zones_path = maps_dir / f"collision_{map_name}_zones.json"

        # Load the collision mask pixmap
        if not collision_map_path.exists():
            self.collision_mask_pixmap = None
            self.zones = {}
            return

        self.collision_mask_pixmap = QPixmap(str(collision_map_path))

        # Load zone definitions
        if not collision_zones_path.exists():
            self.zones = {}
            return

        try:
            with open(collision_zones_path, 'r') as f:
                zones_data = json.load(f)

            self.zones = {}
            for zone_id_str, zone_data in zones_data.items():
                zone_id = int(zone_id_str)
                zone = CollisionZone(
                    zone_id=zone_id,
                    polygon_points=zone_data['polygon_points'],
                    color=zone_data['color']
                )
                # Load painted pixels for this zone
                zone.load_painted_pixels_from_image(self.collision_mask_pixmap)
                self.zones[zone_id] = zone

        except Exception:
            print(f"ERROR: Failed to load collision zones")
            self.zones = {}

    def start_monitoring(self):
        """Start monitoring robot position for collision zones"""
        if self.zones:
            self.enabled = True
            self.check_timer.start()
        else:
            self.enabled = False

    def stop_monitoring(self):
        """Stop monitoring robot position"""
        self.enabled = False
        self.check_timer.stop()

    def update_robot_pose(self, x: float, y: float):
        """Update the robot's current pose"""
        self.robot_x = x
        self.robot_y = y

    def check_robot_position(self):
        """Check if robot is in a collision zone and update parameters accordingly"""
        self.check_counter += 1

        if not self.enabled or self.robot_x is None or self.robot_y is None:
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

        print(f"\n{'='*70}")
        print(f"ROBOT ENTERED COLLISION ZONE {zone_id}")
        print(f"{'='*70}")
        print(f"Robot world position: ({self.robot_x:.3f}, {self.robot_y:.3f})")
        print(f"Robot pixel position: ({pixel_x}, {pixel_y})")
        print(f"Setting polygon points: {polygon_points}")
        print(f"{'='*70}\n")

        # Set the collision monitor parameter
        self.set_collision_polygon(polygon_points)

        # Emit signal
        self.collision_zone_changed.emit(zone_id)

    def exit_zone(self):
        """Called when robot exits all collision zones"""
        print(f"\n{'='*70}")
        print(f"ROBOT EXITED ALL COLLISION ZONES")
        print(f"{'='*70}")
        print(f"Robot position: world=({self.robot_x:.3f}, {self.robot_y:.3f})")
        print(f"Restoring default polygon points: {self.DEFAULT_POLYGON_POINTS}")
        print(f"{'='*70}\n")

        # Restore default polygon points
        self.set_collision_polygon(self.DEFAULT_POLYGON_POINTS)

        # Emit signal
        self.collision_zone_cleared.emit()

    def set_collision_polygon(self, polygon_points: str):
        """Set the collision monitor polygon parameter via ROS2 CLI"""
        try:
            import subprocess

            cmd = [
                'ros2', 'param', 'set',
                '/collision_monitor',
                'PolygonSlow.points',
                polygon_points
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)

            if result.returncode == 0:
                print(f"✓ Successfully set collision polygon parameter")
                if result.stdout and result.stdout.strip():
                    print(f"  ROS2 response: {result.stdout.strip()}")
            else:
                print(f"✗ Failed to set collision polygon parameter!")
                print(f"  Error: {result.stderr}")

        except subprocess.TimeoutExpired:
            print(f"✗ ROS2 param set command timed out (5 seconds)")
        except Exception as e:
            print(f"✗ Exception setting collision polygon: {e}")

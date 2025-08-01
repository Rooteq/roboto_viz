from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItemGroup, QGraphicsRectItem, QPushButton, QHBoxLayout, QWidget
from PyQt5.QtGui import QMouseEvent, QPixmap, QPainter, QTransform
from PyQt5.QtCore import QRectF, pyqtSignal, Qt
from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem
from PyQt5.QtGui import QPen, QBrush, QColor, QFont
from PyQt5.QtCore import Qt, QPointF

import math

from roboto_viz.robot_item import RobotItem
from roboto_viz.goal_arrow import GoalArrow
from roboto_viz.route_manager import BezierRoute
from roboto_viz.bezier_graphics import BezierRouteGraphics, BezierNode, ControlHandle
from roboto_viz.dock_graphics import DockGraphicsManager


class MapView(QGraphicsView):
    goal_pose_set = pyqtSignal(float, float, float)
    dock_placed = pyqtSignal(str)  # dock_name

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setRenderHint(QPainter.Antialiasing)
        self.image_item = None
        self.setMouseTracking(True)  # Enable mouse tracking
        self.map_origin = tuple()

        self.robot_item = RobotItem()
        self.scene.addItem(self.robot_item)
        self.goal_arrow = GoalArrow()
        self.scene.addItem(self.goal_arrow)

        self.point_items = [] 
        self.line_items = []  # New list to store the line items

        self._enable_drawing = False

        self.drawing_arrow = False
        
        # Variables for panning
        self.panning = False
        self.last_pan_point = None
        
        # Zoom variables
        self.zoom_factor = 1.15  # How fast to zoom in/out
        self.current_zoom = 1.0
        self.min_zoom = 0.1      # Allow zooming out further
        
        # Store view state to preserve zoom and pan
        self.preserve_view_state = True
        self.stored_transform = None
        self.stored_center_point = None
        self.max_zoom = 100.0    # Allow extreme zoom levels
        
        # Set drag mode to make panning work with right mouse button
        self.setDragMode(QGraphicsView.NoDrag)
        
        # Important settings for proper zooming
        self.setRenderHint(QPainter.Antialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        
        # Disable the scrollbars to allow zooming beyond the scene boundaries
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        # Allow the view to extend beyond the scene
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        
        # Create control buttons
        self.zoom_in_btn = QPushButton("+", self)
        self.zoom_out_btn = QPushButton("-", self)
        self.pan_btn = QPushButton("pan", self)
        self.pan_btn.setCheckable(True)
        
        # Style the buttons
        button_style = """
            QPushButton {
                background-color: white;
                border: 1px solid gray;
                border-radius: 2px;
                padding: 3px;
                min-width: 24px;
                min-height: 24px;
                font-size: 10px;
            }
            QPushButton:checked {
                background-color: lightgray;
            }
        """
        self.zoom_in_btn.setStyleSheet(button_style)
        self.zoom_out_btn.setStyleSheet(button_style)
        self.pan_btn.setStyleSheet(button_style)
        
        # Create layout for buttons
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.zoom_in_btn)
        button_layout.addWidget(self.zoom_out_btn)
        button_layout.addWidget(self.pan_btn)
        button_layout.setSpacing(3)  # Reduced spacing
        button_layout.setContentsMargins(3, 3, 3, 3)  # Reduced margins
        
        # Create container widget for buttons
        button_container = QWidget(self)
        button_container.setLayout(button_layout)
        
        # Connect button signals
        self.zoom_in_btn.clicked.connect(self.zoom_in)
        self.zoom_out_btn.clicked.connect(self.zoom_out)
        self.pan_btn.clicked.connect(self.toggle_pan_mode)
        
        # Add pan mode flag
        self.left_pan_mode = False

        # Route editing variables
        self.editing_mode = False
        self.current_route_graphics = None
        
        # Dock management variables
        self.dock_graphics_manager = None
        self.dock_placement_mode = False
        self.pending_dock_name = None
        self.dock_editing_mode = False
        self.editing_dock_name = None
        
        # Speed zone editing variables
        self.speed_zone_editing_mode = False
        self.speed_zone_editor = None
        self.last_draw_position = None
        
        # Grid overlay variables
        self.grid_items = []
        self.map_resolution = None  # meters per pixel
        self.grid_visible = False
        self.min_zoom_for_grid = 0.8  # Hide grid when zoomed out below this level

    def load_image(self, image_path, origin_data, resolution=None):
        print(f"DEBUG: Loading new map: {image_path}")
        
        # Clear any existing route graphics first (force clear even in editing mode)
        self.clear_route(force=True)
        
        # Clear existing grid
        self.clear_grid()
        
        # Update map origin and current map path
        self.map_origin = (origin_data[0], origin_data[1], origin_data[2])
        self.current_map_path = image_path
        
        # Store map resolution if provided
        if resolution is not None:
            self.map_resolution = resolution
            print(f"DEBUG: Map resolution: {self.map_resolution} meters/pixel")

        # Load new pixmap
        self.pixmap = QPixmap(image_path)
        
        # Remove old image item if it exists
        if self.image_item:
            self.scene.removeItem(self.image_item)
            self.image_item = None
            
        # Add new image item
        self.image_item = self.scene.addPixmap(self.pixmap)
        
        # Update scene rectangle to match new image
        self.scene.setSceneRect(QRectF(self.pixmap.rect()))
        
        # Create grid overlay
        self.create_grid()
        
        # Force scene update
        self.scene.update()
        
        # Update view to fit new image (force reset for new maps)
        self.update_view(force_reset=True)
        
        print(f"DEBUG: Map loaded successfully, origin: {self.map_origin}")

    def save_view_state(self):
        """Save the current zoom and pan state"""
        if self.image_item:
            self.stored_transform = self.transform()
            self.stored_center_point = self.mapToScene(self.viewport().rect().center())
            print(f"DEBUG: Saved view state - zoom: {self.stored_transform.m11():.2f}, center: ({self.stored_center_point.x():.1f}, {self.stored_center_point.y():.1f})")

    def restore_view_state(self):
        """Restore the previously saved zoom and pan state"""
        if self.stored_transform is not None and self.stored_center_point is not None and self.image_item:
            self.setTransform(self.stored_transform)
            self.centerOn(self.stored_center_point)
            self.current_zoom = self.stored_transform.m11()
            print(f"DEBUG: Restored view state - zoom: {self.current_zoom:.2f}, center: ({self.stored_center_point.x():.1f}, {self.stored_center_point.y():.1f})")
            return True
        return False

    def preserve_view_during_operation(self, operation_func, *args, **kwargs):
        """Execute an operation while preserving the current view state"""
        self.save_view_state()
        try:
            result = operation_func(*args, **kwargs)
            return result
        finally:
            # Small delay to ensure any UI updates are processed
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(10, self.restore_view_state)

    def update_view(self, force_reset=False):
        if self.image_item:
            # If we should preserve view state and have stored state, restore it
            if not force_reset and self.preserve_view_state and self.restore_view_state():
                return
            
            # Otherwise, do the default fit-to-view behavior
            view_rect = self.viewport().rect()
            scene_rect = self.pixmap.rect()
            
            # Calculate scale to fit the image in view
            scale_x = view_rect.width() / scene_rect.width()
            scale_y = view_rect.height() / scene_rect.height()
            scale = min(scale_x, scale_y) * 0.95  # Add some margin
            
            # Reset transform and apply new scale
            self.resetTransform()
            transform = QTransform()
            transform.scale(scale, scale)
            
            self.setTransform(transform)
            self.current_zoom = scale
            
            # Center on the image
            self.centerOn(self.image_item)
            
            # Save the new view state
            self.save_view_state()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Save current view state before resize, then restore it
        if self.image_item:
            self.save_view_state()
        # Don't force reset on resize - just maintain current view

    def update_robot_pose(self, x, y, theta):
        map_x = (x - self.map_origin[0]) * 20
        map_y = self.pixmap.rect().height() - ((y - self.map_origin[1]) * 20)
        
        self.robot_item.update_pose(map_x, map_y, theta)
        self.scene.update()

    def mousePressEvent(self, event):
        # Handle right button press for panning
        if event.button() == Qt.RightButton:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.panning = True
            self.last_pan_point = event.pos()
            fake_event = QMouseEvent(
                event.type(), event.pos(), Qt.LeftButton,
                Qt.LeftButton, event.modifiers()
            )
            super().mousePressEvent(fake_event)
            return
            
        # Handle left button for panning when in pan mode
        if event.button() == Qt.LeftButton and self.left_pan_mode:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.panning = True
            super().mousePressEvent(event)
            return
            
        # Handle speed zone editing mode first
        if (event.button() == Qt.LeftButton and self.speed_zone_editing_mode and 
            not self.left_pan_mode and self.speed_zone_editor and 
            self.speed_zone_editor.drawing_enabled):
            scene_pos = self.mapToScene(event.pos())
            self.speed_zone_editor.start_box_drawing(scene_pos.x(), scene_pos.y())
            return
        
        # Handle dock placement mode first (independent of drawing mode)
        if event.button() == Qt.LeftButton and self.dock_placement_mode and not self.left_pan_mode:
            scene_pos = self.mapToScene(event.pos())
            print(f"DEBUG: In dock placement mode - placing dock '{self.pending_dock_name}' at {scene_pos.x()}, {scene_pos.y()}")
            if self.place_dock_at_position(scene_pos.x(), scene_pos.y()):
                print(f"DEBUG: Dock placed successfully")
            else:
                print(f"DEBUG: Failed to place dock")
            return
        
        # Handle left button for drawing when not in pan mode
        if event.button() == Qt.LeftButton and self.enable_drawing and not self.left_pan_mode:
            scene_pos = self.mapToScene(event.pos())
            print(f"DEBUG: Left button pressed - enable_drawing={self.enable_drawing}, editing_mode={self.editing_mode}, left_pan_mode={self.left_pan_mode}")
            
            # If we're in editing mode, check if we clicked on an existing item first
            if self.editing_mode and self.current_route_graphics:
                # Check if we clicked on an existing graphics item (node, control handle, etc.)
                item_at_pos = self.itemAt(event.pos())
                print(f"DEBUG: Item at mouse position: {item_at_pos}")
                print(f"DEBUG: Item type: {type(item_at_pos).__name__ if item_at_pos else 'None'}")
                
                if item_at_pos and item_at_pos != self.image_item:
                    # We clicked on an existing item - let Qt handle the interaction
                    print(f"DEBUG: Found interactive item, letting Qt handle it")
                    super().mousePressEvent(event)
                    return
                else:
                    # We clicked on empty space, add a new node
                    print(f"DEBUG: Left click in editing mode - adding node at scene position: {scene_pos.x()}, {scene_pos.y()}")
                    self.current_route_graphics.add_node_at_position(scene_pos.x(), scene_pos.y())
                    return
            else:
                print(f"DEBUG: Left click in drawing mode - editing_mode: {self.editing_mode}, current_route_graphics: {self.current_route_graphics}")
                if self.editing_mode:
                    print(f"DEBUG: No current_route_graphics available!")
                    return
                
                # Original arrow drawing logic for non-editing mode
                self.drawing_arrow = True
                self.goal_arrow.set_points(scene_pos, scene_pos)
                return
            
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.panning:
            # Pan is handled by ScrollHandDrag mode
            fake_event = QMouseEvent(
                event.type(), event.pos(), Qt.LeftButton,
                Qt.LeftButton, event.modifiers()
            )
            super().mouseMoveEvent(fake_event)
            return
            
        # Handle speed zone editing mode for box dragging
        if (self.speed_zone_editing_mode and self.speed_zone_editor and 
            self.speed_zone_editor.drawing_enabled and self.speed_zone_editor.drawing_box):
            scene_pos = self.mapToScene(event.pos())
            self.speed_zone_editor.update_box_drawing(scene_pos.x(), scene_pos.y())
            return
        
        # In editing mode, allow item dragging to work properly
        if self.editing_mode:
            super().mouseMoveEvent(event)
            return
            
        if self.drawing_arrow and self.enable_drawing and not self.editing_mode:
            scene_pos = self.mapToScene(event.pos())
            self.goal_arrow.set_points(self.goal_arrow.start_point, scene_pos)
            return
            
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if (event.button() == Qt.RightButton or 
            (event.button() == Qt.LeftButton and self.left_pan_mode)) and self.panning:
            self.panning = False
            self.setDragMode(QGraphicsView.NoDrag)
            if event.button() == Qt.RightButton:
                fake_event = QMouseEvent(
                    event.type(), event.pos(), Qt.LeftButton,
                    Qt.LeftButton, event.modifiers()
                )
                super().mouseReleaseEvent(fake_event)
            else:
                super().mouseReleaseEvent(event)
            # Save view state after panning
            self.save_view_state()
            return
            
        # Handle speed zone editing mode
        if (event.button() == Qt.LeftButton and self.speed_zone_editing_mode and 
            self.speed_zone_editor and self.speed_zone_editor.drawing_enabled):
            scene_pos = self.mapToScene(event.pos())
            self.speed_zone_editor.finish_box_drawing(scene_pos.x(), scene_pos.y())
            return
        
        # In editing mode, let items handle the release event
        if self.editing_mode:
            super().mouseReleaseEvent(event)
            return
            
        if event.button() == Qt.LeftButton and self.drawing_arrow and self.enable_drawing and not self.left_pan_mode and not self.editing_mode:
            self.drawing_arrow = False
            scene_pos = self.mapToScene(event.pos())
            self.goal_arrow.set_points(self.goal_arrow.start_point, scene_pos)
            
            # Convert to map coordinates
            start_x = (self.goal_arrow.start_point.x() * 0.05) + self.map_origin[0]
            start_y = (self.pixmap.rect().height() - self.goal_arrow.start_point.y()) * 0.05 + self.map_origin[1]
            
            angle = self.goal_arrow.get_angle()
            
            self.clear_goal_arrow()
            self.goal_pose_set.emit(start_x, start_y, angle)
            return
            
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event):
        """
        Handle zoom with mouse wheel
        """
        # Calculate zoom factor based on wheel delta
        zoomInFactor = self.zoom_factor
        zoomOutFactor = 1 / zoomInFactor

        # Save the scene pos
        oldPos = self.mapToScene(event.pos())

        # Get mouse wheel direction
        zoom_in = event.angleDelta().y() > 0
        
        # Check zoom before applying to prevent any scaling limitations
        if zoom_in:
            # Don't enforce max zoom - allow unlimited zooming in
            self.scale(zoomInFactor, zoomInFactor)
            self.current_zoom *= zoomInFactor
        else:
            # Only limit zooming out
            new_zoom = self.current_zoom * zoomOutFactor
            if new_zoom >= self.min_zoom:
                self.scale(zoomOutFactor, zoomOutFactor)
                self.current_zoom = new_zoom
                
        # Get the new position
        newPos = self.mapToScene(event.pos())
        
        # Move scene to keep mouse position fixed
        delta = newPos - oldPos
        self.translate(delta.x(), delta.y())
        
        # Update grid visibility based on new zoom level
        self.update_grid_visibility()
        
        # Save the new view state after zooming
        self.save_view_state()
                
        # Accept the event to prevent it from being propagated
        event.accept()

    def clear_goal_arrow(self):
        self.goal_arrow.hide_arrow()

    def display_points(self, points):
        """
        Display numbered gray dots with direction indicators at the specified points.
        Connect consecutive points with green lines.
        
        Args:
            points: List of (x,y,z,w) coordinates where w is the rotation in radians
        """
        self.clear_points()
        
        point_radius = 5
        point_color = QColor(64, 64, 64)  # Gray
        line_color = QColor(0, 255, 0)    # Green
        line_width = 2
        
        rect_width = 8
        rect_height = 1
        
        font = QFont()
        font.setPointSize(4)
        
        # Store map coordinates for drawing lines
        map_coordinates = []
        
        # First, create all the point markers
        for i, point in enumerate(points):
            map_x = (point[0] - self.map_origin[0]) * 20
            map_y = self.pixmap.rect().height() - ((point[1] - self.map_origin[1]) * 20)
            map_coordinates.append((map_x, map_y))
            
            point_group = QGraphicsItemGroup()
            
            ellipse = QGraphicsEllipseItem(
                map_x - point_radius/2,
                map_y - point_radius/2,
                point_radius,
                point_radius
            )
            ellipse.setBrush(QBrush(point_color))
            ellipse.setPen(QPen(point_color))
            point_group.addToGroup(ellipse)
            
            rect = QGraphicsRectItem(
                map_x - rect_width/2,
                map_y - rect_height/2,
                rect_width,
                rect_height
            )
            rect.setBrush(QBrush(point_color))
            rect.setPen(QPen(point_color))
            
            # Set the rotation center to the middle of the rectangle
            rect.setTransformOriginPoint(map_x, map_y)
            # Convert the w coordinate to degrees and rotate
            rotation_degrees = -point[3] * (180.0 / math.pi)  # Convert radians to degrees
            rect.setRotation(rotation_degrees)
            point_group.addToGroup(rect)
            
            text = QGraphicsTextItem(str(i))
            text.setFont(font)
            text.setDefaultTextColor(Qt.white)
            
            text_bounds = text.boundingRect()
            
            text_x = map_x - text_bounds.width()/2
            text_y = map_y - text_bounds.height()/2
            text.setPos(text_x, text_y)
            point_group.addToGroup(text)
            
            point_group.setZValue(2)
            self.scene.addItem(point_group)
            self.point_items.append(point_group)
        
        # Now draw lines connecting consecutive points
        if len(map_coordinates) > 1:
            for i in range(len(map_coordinates) - 1):
                start_x, start_y = map_coordinates[i]
                end_x, end_y = map_coordinates[i + 1]
                
                line = QGraphicsLineItem(start_x, start_y, end_x, end_y)
                pen = QPen(line_color)
                pen.setWidth(line_width)
                line.setPen(pen)
                
                line.setZValue(1)

                self.scene.addItem(line)
                self.line_items.append(line)

    def clear_points(self):
        """
        Remove all point markers and connecting lines from the map.
        """
        for point_group in self.point_items:
            self.scene.removeItem(point_group)
        self.point_items.clear()
        
        for line in self.line_items:
            self.scene.removeItem(line)
        self.line_items.clear()
        
    def reset_zoom(self):
        """
        Reset zoom level to the original scale that fits the viewport
        """
        self.resetTransform()
        self.update_view(force_reset=True)

    @property
    def enable_drawing(self):
        return self._enable_drawing

    @enable_drawing.setter
    def enable_drawing(self, value: bool):
        self._enable_drawing = value

    def zoom_in(self):
        """Zoom in by the zoom factor"""
        self.scale(self.zoom_factor, self.zoom_factor)
        self.current_zoom *= self.zoom_factor
        self.update_grid_visibility()
        
    def zoom_out(self):
        """Zoom out by the zoom factor"""
        new_zoom = self.current_zoom / self.zoom_factor
        if new_zoom >= self.min_zoom:
            self.scale(1/self.zoom_factor, 1/self.zoom_factor)
            self.current_zoom = new_zoom
            self.update_grid_visibility()
            
    def toggle_pan_mode(self):
        """Toggle panning mode for left mouse button"""
        self.left_pan_mode = self.pan_btn.isChecked()

    def clear_route(self, force=False):
        """Clear the current route graphics"""
        # Skip clearing route graphics if in editing mode to preserve the current editing session
        # unless force is True (for map changes)
        if self.editing_mode and not force:
            print(f"DEBUG: Skipping route clear - in editing mode")
            return
            
        print(f"DEBUG: Clearing route graphics (force={force})")
        if hasattr(self, 'current_route_graphics') and self.current_route_graphics:
            # Clear all graphics items managed by the route graphics
            self.current_route_graphics.clear_graphics()
            self.current_route_graphics = None
            print(f"DEBUG: Route graphics cleared")

    def display_bezier_route(self, bezier_route: BezierRoute, force_update: bool = False):
        """
        Display a bezier route with interactive nodes and curves.
        
        Args:
            bezier_route: BezierRoute object to display
            force_update: If True, display route even in editing mode (used for starting editing)
        """
        print(f"DEBUG: display_bezier_route called with bezier_route: {bezier_route}, force_update: {force_update}")
        
        # Skip displaying routes when in editing mode unless forced (to preserve the current editing session)
        if self.editing_mode and not force_update:
            print(f"DEBUG: Skipping route display - in editing mode and not forced")
            return
            
        self.clear_route()
        
        if bezier_route:
            # Check if we have a map loaded
            if hasattr(self, 'pixmap') and self.pixmap and hasattr(self, 'map_origin'):
                print(f"DEBUG: Creating BezierRouteGraphics with map_origin: {self.map_origin}")
                # Create graphics even for empty routes so users can add nodes
                self.current_route_graphics = BezierRouteGraphics(
                    bezier_route, self.map_origin, self.pixmap.rect().height(), self.scene
                )
                print(f"DEBUG: Created current_route_graphics: {self.current_route_graphics}")
            else:
                print(f"DEBUG: No map loaded - cannot create route graphics yet")
                print(f"DEBUG: Has pixmap: {hasattr(self, 'pixmap')}, Has map_origin: {hasattr(self, 'map_origin')}")
        else:
            print(f"DEBUG: No bezier_route provided")

    def start_route_editing(self, bezier_route: BezierRoute = None):
        """
        Start editing mode for creating/modifying routes.
        
        Args:
            bezier_route: Existing route to edit, or None to create new
        """
        print(f"DEBUG: start_route_editing called with bezier_route: {bezier_route}")
        
        # Force clear any existing route graphics before starting editing
        self.clear_route(force=True)
        
        self.editing_mode = True
        self.enable_drawing = True
        print(f"DEBUG: Set editing_mode={self.editing_mode}, enable_drawing={self.enable_drawing}")
        
        if bezier_route is None:
            # Create new empty route
            bezier_route = BezierRoute()
            print(f"DEBUG: Created new empty BezierRoute: {bezier_route}")
            
        # Force update to create graphics even in editing mode
        self.display_bezier_route(bezier_route, force_update=True)
        
    def stop_route_editing(self):
        """Stop route editing mode"""
        self.editing_mode = False
        self.enable_drawing = False
        self.clear_route()
        
    def get_current_route(self):
        """Get the current route from the graphics display"""
        if hasattr(self, 'current_route_graphics') and self.current_route_graphics:
            return self.current_route_graphics.bezier_route
        return None
    
    # Dock management methods
    def init_dock_graphics(self, dock_manager):
        """Initialize dock graphics manager with the current map"""
        if hasattr(self, 'map_origin') and self.map_origin and hasattr(self, 'image_item') and self.image_item:
            pixmap_height = self.image_item.pixmap().height()
            self.dock_graphics_manager = DockGraphicsManager(
                dock_manager, self.map_origin, pixmap_height, self.scene
            )
            print(f"DEBUG: Initialized dock graphics manager")
        else:
            print(f"DEBUG: Cannot initialize dock graphics - missing map data")
    
    def start_dock_placement(self, dock_name):
        """Start dock placement mode"""
        self.dock_placement_mode = True
        self.pending_dock_name = dock_name
        print(f"DEBUG: Started dock placement mode for '{dock_name}'")
    
    def stop_dock_placement(self):
        """Stop dock placement mode"""
        self.dock_placement_mode = False
        self.pending_dock_name = None
        print(f"DEBUG: Stopped dock placement mode")
    
    def place_dock_at_position(self, scene_x, scene_y):
        """Place a dock at the specified scene position (temporary, not saved)"""
        if not self.dock_placement_mode or not self.pending_dock_name:
            return False
        
        if self.dock_graphics_manager:
            # Create temporary dock for editing
            success = self.dock_graphics_manager.create_temporary_dock(
                scene_x, scene_y, self.pending_dock_name
            )
            if success:
                # Switch to dock editing mode
                self.dock_editing_mode = True
                self.editing_dock_name = self.pending_dock_name
                self.dock_placement_mode = False
                self.pending_dock_name = None
                print(f"DEBUG: Created temporary dock '{self.editing_dock_name}' for editing")
                return True
        return False
    
    def update_dock_graphics(self):
        """Update dock graphics display"""
        if self.dock_graphics_manager:
            self.dock_graphics_manager.update_graphics()
    
    def clear_dock_graphics(self):
        """Clear all dock graphics"""
        if self.dock_graphics_manager:
            self.dock_graphics_manager.clear_graphics()
    
    def show_dock(self, dock_name: str):
        """Show a specific dock on the map"""
        if self.dock_graphics_manager:
            self.dock_graphics_manager.show_dock(dock_name)
    
    def hide_dock(self, dock_name: str):
        """Hide a specific dock from the map"""
        if self.dock_graphics_manager:
            self.dock_graphics_manager.hide_dock(dock_name)
    
    def hide_all_docks(self):
        """Hide all docks from the map"""
        if self.dock_graphics_manager:
            self.dock_graphics_manager.hide_all_docks()
    
    def show_all_docks(self):
        """Show all docks on the map"""
        if self.dock_graphics_manager:
            self.dock_graphics_manager.show_all_docks()
    
    def start_dock_editing(self, dock_name: str):
        """Start editing mode for a dock"""
        self.dock_editing_mode = True
        self.editing_dock_name = dock_name
        
        if self.dock_graphics_manager:
            # Enable editing for this dock
            self.dock_graphics_manager.set_dock_editing_mode(dock_name, True)
            # Show only this dock for editing
            self.dock_graphics_manager.hide_all_docks()
            self.dock_graphics_manager.show_dock(dock_name)
        
        print(f"DEBUG: Started dock editing for '{dock_name}'")
    
    def stop_dock_editing(self):
        """Stop dock editing mode"""
        if self.dock_editing_mode and self.editing_dock_name:
            if self.dock_graphics_manager:
                # Disable editing for the current dock
                self.dock_graphics_manager.set_dock_editing_mode(self.editing_dock_name, False)
                # Hide the dock after editing
                self.dock_graphics_manager.hide_dock(self.editing_dock_name)
        
        self.dock_editing_mode = False
        self.editing_dock_name = None
        print(f"DEBUG: Stopped dock editing mode")
    
    def save_current_dock(self) -> bool:
        """Save the current dock position"""
        if self.dock_editing_mode and self.editing_dock_name and self.dock_graphics_manager:
            # Check if this is a temporary dock (new dock) or existing dock
            docks = self.dock_graphics_manager.dock_manager.load_docks()
            if self.editing_dock_name in docks:
                # Existing dock - update position
                return self.dock_graphics_manager.save_dock_position(self.editing_dock_name)
            else:
                # Temporary dock - save to dock manager
                return self.dock_graphics_manager.save_temporary_dock(self.editing_dock_name)
        return False
    
    def cancel_dock_editing(self):
        """Cancel dock editing and revert to saved position"""
        if self.dock_editing_mode and self.editing_dock_name and self.dock_graphics_manager:
            # Check if this is a temporary dock (new dock) or existing dock
            docks = self.dock_graphics_manager.dock_manager.load_docks()
            if self.editing_dock_name in docks:
                # Existing dock - revert to saved position
                self.dock_graphics_manager.revert_dock_position(self.editing_dock_name)
            else:
                # Temporary dock - delete it
                self.dock_graphics_manager.cancel_temporary_dock(self.editing_dock_name)
        
        self.stop_dock_editing()
    
    # Speed zone editing methods
    def start_speed_zone_editing(self, speed_zone_editor):
        """Start speed zone editing mode"""
        self.speed_zone_editing_mode = True
        self.speed_zone_editor = speed_zone_editor
        self.enable_drawing = True
        print(f"DEBUG: Started speed zone editing mode")
        
    def stop_speed_zone_editing(self):
        """Stop speed zone editing mode"""
        print(f"DEBUG: Stopping speed zone editing mode")
        
        # Cancel any ongoing box drawing
        if self.speed_zone_editor and self.speed_zone_editor.drawing_box:
            self.speed_zone_editor.cancel_box_drawing()
        
        # Clear all speed zone editing state
        self.speed_zone_editing_mode = False
        self.speed_zone_editor = None
        self.last_draw_position = None
        self.enable_drawing = False
        
        print(f"DEBUG: Speed zone editing mode stopped")
        
    def reload_current_map(self):
        """Reload the current map image (for when speed zones are updated)"""
        if hasattr(self, 'current_map_path') and self.current_map_path:
            try:
                # Reload the pixmap from file
                new_pixmap = QPixmap(self.current_map_path)
                if self.image_item:
                    self.image_item.setPixmap(new_pixmap)
                    self.pixmap = new_pixmap
                    self.scene.update()
                    print(f"DEBUG: Reloaded map image")
            except Exception as e:
                print(f"DEBUG: Error reloading map: {e}")
                
    def set_current_map_path(self, map_path: str):
        """Set the current map path for reloading"""
        self.current_map_path = map_path
    
    # Grid overlay methods
    def create_grid(self):
        """Create grid overlay based on map resolution"""
        print(f"DEBUG: create_grid called - resolution: {self.map_resolution}, has_pixmap: {self.pixmap is not None}")
        
        if not self.map_resolution or not self.pixmap:
            print(f"DEBUG: Cannot create grid - missing resolution or pixmap")
            return
        
        # Clear existing grid
        self.clear_grid()
        
        # Calculate grid spacing in pixels for 50cm (0.5m) grid
        grid_spacing_meters = 0.5  # 50cm
        grid_spacing_pixels = grid_spacing_meters / self.map_resolution
        
        print(f"DEBUG: Grid spacing: {grid_spacing_pixels:.2f} pixels for {grid_spacing_meters}m at {self.map_resolution:.6f} m/pixel")
        
        # Get map dimensions
        map_width = self.pixmap.width()
        map_height = self.pixmap.height()
        
        print(f"DEBUG: Map dimensions: {map_width}x{map_height} pixels")
        
        # Create vertical grid lines
        x = 0
        while x <= map_width:
            line = QGraphicsLineItem(x, 0, x, map_height)
            pen = QPen(QColor(128, 128, 128))  # Light grey, solid
            pen.setWidth(0)  # Cosmetic pen - thin lines
            line.setPen(pen)
            line.setZValue(1)  # Above map but below other elements
            self.scene.addItem(line)
            self.grid_items.append(line)
            x += grid_spacing_pixels
        
        # Create horizontal grid lines
        y = 0
        while y <= map_height:
            line = QGraphicsLineItem(0, y, map_width, y)
            pen = QPen(QColor(128, 128, 128))  # Light grey, solid
            pen.setWidth(0)  # Cosmetic pen - thin lines
            line.setPen(pen)
            line.setZValue(1)  # Above map but below other elements
            self.scene.addItem(line)
            self.grid_items.append(line)
            y += grid_spacing_pixels
        
        # Update grid visibility based on current zoom
        self.update_grid_visibility()
        
        print(f"DEBUG: Created grid with {len(self.grid_items)} lines, spacing: {grid_spacing_pixels:.2f} pixels")
    
    def clear_grid(self):
        """Remove all grid lines from the scene"""
        for item in self.grid_items:
            self.scene.removeItem(item)
        self.grid_items.clear()
    
    def update_grid_visibility(self):
        """Update grid visibility based on zoom level"""
        should_show_grid = (len(self.grid_items) > 0 and 
                           self.current_zoom >= self.min_zoom_for_grid)
        
        if should_show_grid != self.grid_visible:
            self.grid_visible = should_show_grid
            for item in self.grid_items:
                item.setVisible(self.grid_visible)
            
            if self.grid_visible:
                print(f"DEBUG: Grid shown at zoom {self.current_zoom:.2f}")
            else:
                print(f"DEBUG: Grid hidden at zoom {self.current_zoom:.2f}")
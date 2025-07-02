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


class MapView(QGraphicsView):
    goal_pose_set = pyqtSignal(float, float, float)

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
                padding: 5px;
                min-width: 30px;
                min-height: 30px;
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
        button_layout.setSpacing(5)
        button_layout.setContentsMargins(5, 5, 5, 5)
        
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

    def load_image(self, image_path, origin_data):
        print(f"DEBUG: Loading new map: {image_path}")
        
        # Clear any existing route graphics first (force clear even in editing mode)
        self.clear_route(force=True)
        
        # Update map origin
        self.map_origin = (origin_data[0], origin_data[1], origin_data[2])

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
        
        # Force scene update
        self.scene.update()
        
        # Update view to fit new image
        self.update_view()
        
        print(f"DEBUG: Map loaded successfully, origin: {self.map_origin}")

    def update_view(self):
        if self.image_item:
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

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_view()

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
            
        # Handle left button for drawing when not in pan mode
        if event.button() == Qt.LeftButton and self.enable_drawing and not self.left_pan_mode:
            scene_pos = self.mapToScene(event.pos())
            
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
        self.update_view()

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
        
    def zoom_out(self):
        """Zoom out by the zoom factor"""
        new_zoom = self.current_zoom / self.zoom_factor
        if new_zoom >= self.min_zoom:
            self.scale(1/self.zoom_factor, 1/self.zoom_factor)
            self.current_zoom = new_zoom
            
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
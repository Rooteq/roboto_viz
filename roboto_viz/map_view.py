from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItemGroup, QGraphicsRectItem
from PyQt5.QtGui import QMouseEvent, QPixmap, QPainter, QTransform
from PyQt5.QtCore import QRectF, pyqtSignal, Qt
from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem
from PyQt5.QtGui import QPen, QBrush, QColor, QFont
from PyQt5.QtCore import Qt, QPointF

import math

from roboto_viz.robot_item import RobotItem
from roboto_viz.goal_arrow import GoalArrow


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

    def load_image(self, image_path, origin_data):
        self.map_origin = (origin_data[0], origin_data[1], origin_data[2])

        self.pixmap = QPixmap(image_path)
        if self.image_item:
            self.scene.removeItem(self.image_item)
        self.image_item = self.scene.addPixmap(self.pixmap)
        self.scene.setSceneRect(QRectF(self.pixmap.rect()))
        self.update_view()

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
            # Create a fake left button event to start the drag
            fake_event = QMouseEvent(
                event.type(), event.pos(), Qt.LeftButton,
                Qt.LeftButton, event.modifiers()
            )
            super().mousePressEvent(fake_event)
            return
            
        # Handle left button for drawing
        if event.button() == Qt.LeftButton and self.enable_drawing:
            self.drawing_arrow = True
            scene_pos = self.mapToScene(event.pos())
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
            
        if self.drawing_arrow and self.enable_drawing:
            scene_pos = self.mapToScene(event.pos())
            self.goal_arrow.set_points(self.goal_arrow.start_point, scene_pos)
            return
            
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton and self.panning:
            self.panning = False
            self.setDragMode(QGraphicsView.NoDrag)
            # Create a fake left button event to end the drag
            fake_event = QMouseEvent(
                event.type(), event.pos(), Qt.LeftButton,
                Qt.LeftButton, event.modifiers()
            )
            super().mouseReleaseEvent(fake_event)
            return
            
        if event.button() == Qt.LeftButton and self.drawing_arrow and self.enable_drawing:
            self.drawing_arrow = False
            scene_pos = self.mapToScene(event.pos())
            self.goal_arrow.set_points(self.goal_arrow.start_point, scene_pos)
            
            # Convert to map coordinates
            start_x = (self.goal_arrow.start_point.x() * 0.05) + self.map_origin[0]
            start_y = (self.pixmap.rect().height() - self.goal_arrow.start_point.y()) * 0.05 + self.map_origin[1]
            
            # Calculate angle
            angle = self.goal_arrow.get_angle()
            
            self.clear_goal_arrow()

            # Emit the goal pose
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
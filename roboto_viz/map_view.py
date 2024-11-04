from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItemGroup, QGraphicsRectItem
from PyQt5.QtGui import QMouseEvent, QPixmap, QPainter, QTransform
from PyQt5.QtCore import QRectF, pyqtSignal, Qt
from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsTextItem
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

        self.drawing_arrow = False

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
            
            scale_x = view_rect.width() / scene_rect.width()
            scale_y = view_rect.height() / scene_rect.height()
            scale = min(scale_x, scale_y)
            
            transform = QTransform()
            transform.scale(scale, scale)
            
            self.setTransform(transform)
            
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
        if event.button() == Qt.LeftButton:
            self.drawing_arrow = True
            scene_pos = self.mapToScene(event.pos())
            self.goal_arrow.set_points(scene_pos, scene_pos)

    def mouseMoveEvent(self, event):
        if self.drawing_arrow:
            scene_pos = self.mapToScene(event.pos())
            self.goal_arrow.set_points(self.goal_arrow.start_point, scene_pos)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.drawing_arrow:
            self.drawing_arrow = False
            scene_pos = self.mapToScene(event.pos())
            self.goal_arrow.set_points(self.goal_arrow.start_point, scene_pos)
            
            # Convert to map coordinates
            start_x = (self.goal_arrow.start_point.x() * 0.05) + self.map_origin[0]
            start_y = (self.pixmap.rect().height() - self.goal_arrow.start_point.y()) * 0.05 + self.map_origin[1]
            end_x = (scene_pos.x() * 0.05) + self.map_origin[0]
            end_y = (self.pixmap.rect().height() - scene_pos.y()) * 0.05 + self.map_origin[1]
            
            # Calculate angle
            angle = self.goal_arrow.get_angle()
            
            self.clear_goal_arrow()

            # Emit the goal pose
            self.goal_pose_set.emit(start_x, start_y, angle)

    def clear_goal_arrow(self):
        self.goal_arrow.hide_arrow()

    def display_points(self, points):
        """
        Display numbered gray dots with direction indicators at the specified points.
        Args:
            points: List of (x,y,z,w) coordinates where w is the rotation in radians
        """
        # Clear existing points first
        self.clear_points()
        
        # Define point appearance
        point_radius = 5
        point_color = QColor(64, 64, 64)  # Gray
        
        # Define direction indicator rectangle
        rect_width = 8
        rect_height = 1
        
        # Create small font for numbers
        font = QFont()
        font.setPointSize(4)
        
        for i, point in enumerate(points):
            # Convert map coordinates to scene coordinates
            map_x = (point[0] - self.map_origin[0]) * 20
            map_y = self.pixmap.rect().height() - ((point[1] - self.map_origin[1]) * 20)
            
            # Create group to hold all elements of the point
            point_group = QGraphicsItemGroup()
            
            # Create the circular background
            ellipse = QGraphicsEllipseItem(
                map_x - point_radius/2,
                map_y - point_radius/2,
                point_radius,
                point_radius
            )
            ellipse.setBrush(QBrush(point_color))
            ellipse.setPen(QPen(point_color))
            point_group.addToGroup(ellipse)
            
            # Create direction indicator rectangle
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
            
            # Create the number label
            text = QGraphicsTextItem(str(i))
            text.setFont(font)
            text.setDefaultTextColor(Qt.white)
            
            # Get the exact bounding rectangle of the text
            text_bounds = text.boundingRect()
            
            # Position text so its center aligns with dot's center
            text_x = map_x - text_bounds.width()/2
            text_y = map_y - text_bounds.height()/2
            text.setPos(text_x, text_y)
            point_group.addToGroup(text)
            
            # Add group to scene
            self.scene.addItem(point_group)
            
            # Store group for later removal
            self.point_items.append(point_group)

    def clear_points(self):
        """
        Remove all point markers from the map.
        """
        for point_group in self.point_items:
            self.scene.removeItem(point_group)
        self.point_items.clear()

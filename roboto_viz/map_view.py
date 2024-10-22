from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene
from PyQt5.QtGui import QMouseEvent, QPixmap, QPainter, QTransform
from PyQt5.QtCore import QRectF, pyqtSignal, Qt

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
        # print("pose update!")
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

    # def mousePressEvent(self, event) -> None:
    #     scene_pos = self.mapToScene(event.pos())
    #     x = (scene_pos.x() * 0.05) + self.map_origin[0]
    #     y = (self.pixmap.rect().height() - scene_pos.y()) * 0.05 + self.map_origin[1]
    #     self.mouse_clicked.emit(x,y)

    # def mouseMoveEvent(self, event):
    #     if self.image_item:
    #         scene_pos = self.mapToScene(event.pos())
    #         x = scene_pos.x()
    #         y = scene_pos.y()
    #         self.mouse_moved.emit(x, y)
    #     super().mouseMoveEvent(event)

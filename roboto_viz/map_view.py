from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene
from PyQt5.QtGui import QMouseEvent, QPixmap, QPainter, QTransform
from PyQt5.QtCore import QRectF, pyqtSignal

from roboto_viz.robot_item import RobotItem

class MapView(QGraphicsView):
    mouse_clicked = pyqtSignal(float, float)

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

    def mousePressEvent(self, event: QMouseEvent) -> None:
        scene_pos = self.mapToScene(event.pos())
        x = (scene_pos.x() * 0.05) + self.map_origin[0]
        y = (self.pixmap.rect().height() - scene_pos.y()) * 0.05 + self.map_origin[1]
        self.mouse_clicked.emit(x,y)

    # def mouseMoveEvent(self, event):
    #     if self.image_item:
    #         scene_pos = self.mapToScene(event.pos())
    #         x = scene_pos.x()
    #         y = scene_pos.y()
    #         self.mouse_moved.emit(x, y)
    #     super().mouseMoveEvent(event)

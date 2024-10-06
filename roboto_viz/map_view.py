from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene
from PyQt5.QtGui import QMouseEvent, QPixmap, QPainter, QTransform
from PyQt5.QtCore import QRectF, pyqtSignal

class MapView(QGraphicsView):
    mouse_clicked = pyqtSignal(float, float)  # New signal for mouse movement

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setRenderHint(QPainter.Antialiasing)
        self.image_item = None
        self.setMouseTracking(True)  # Enable mouse tracking
        self.map_origin = tuple()

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
            
            # Calculate the scaling factor to fit the image in the view
            scale_x = view_rect.width() / scene_rect.width()
            scale_y = view_rect.height() / scene_rect.height()
            scale = min(scale_x, scale_y)
            
            # Create a new transform with the calculated scale
            transform = QTransform()
            transform.scale(scale, scale)
            
            # Apply the transform to the QGraphicsView
            self.setTransform(transform)
            
            # Center the image in the view
            self.centerOn(self.image_item)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_view()

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

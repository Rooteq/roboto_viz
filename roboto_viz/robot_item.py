from PyQt5.QtWidgets import QGraphicsItem
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import QPointF, QRectF
import math

class RobotItem(QGraphicsItem):
    def __init__(self):
        super().__init__()
        self.arrow_color = QColor(255, 0, 0)  # Red arrow
        self.arrow_size = 10  # Size of the arrow
        self.setZValue(1)  # Ensure the arrow is drawn on top of the map

    def boundingRect(self):
        return QRectF(-self.arrow_size/2, -self.arrow_size/2, self.arrow_size, self.arrow_size)

    def paint(self, painter, option, widget):
        painter.setPen(QPen(self.arrow_color, 2))
        painter.setBrush(self.arrow_color)

        # Draw the arrow body
        painter.drawLine(0, 0, int(self.arrow_size/2), 0)

        # Draw the arrow head
        head = QPointF(self.arrow_size/2, 0)
        painter.drawPolygon([
            head,
            head + QPointF(-self.arrow_size/4, -self.arrow_size/4),
            head + QPointF(-self.arrow_size/4, self.arrow_size/4)
        ])

    def update_pose(self, x, y, theta):
        self.setPos(x, y)
        self.setRotation(math.degrees(theta))
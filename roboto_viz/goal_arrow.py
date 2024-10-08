from PyQt5.QtWidgets import QGraphicsItem
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import QPointF, QRectF, Qt
import math

class GoalArrow(QGraphicsItem):
    def __init__(self):
        super().__init__()
        self.start_point = QPointF()
        self.end_point = QPointF()
        self.arrow_color = QColor(0, 255, 0)  # Green arrow
        self.arrow_size = 10
        self.setZValue(2)  # Ensure it's drawn on top of the map and robot
        self.visible = False

    def boundingRect(self):
        if not self.visible:
            return QRectF()
        return QRectF(self.start_point, self.end_point).normalized().adjusted(-self.arrow_size, -self.arrow_size, self.arrow_size, self.arrow_size)

    def paint(self, painter, option, widget):
        if not self.visible:
            return

        painter.setPen(QPen(self.arrow_color, 2, Qt.SolidLine))
        painter.setBrush(self.arrow_color)

        # Draw the line
        painter.drawLine(self.start_point, self.end_point)

        # Calculate the angle of the line
        angle = math.atan2(self.end_point.y() - self.start_point.y(), self.end_point.x() - self.start_point.x())

        # Draw the arrowhead
        arrowhead_p1 = self.end_point - QPointF(math.cos(angle - math.pi/6) * self.arrow_size,
                                                math.sin(angle - math.pi/6) * self.arrow_size)
        arrowhead_p2 = self.end_point - QPointF(math.cos(angle + math.pi/6) * self.arrow_size,
                                                math.sin(angle + math.pi/6) * self.arrow_size)
        painter.drawPolygon(self.end_point, arrowhead_p1, arrowhead_p2)

    def set_points(self, start, end):
        self.prepareGeometryChange()  # This is crucial for smooth updates
        self.start_point = start
        self.end_point = end
        self.visible = True
        self.update()

    def hide_arrow(self):
        self.visible = False
        self.update()

    def get_angle(self):
        return math.atan2(self.end_point.y() - self.start_point.y(), self.end_point.x() - self.start_point.x())
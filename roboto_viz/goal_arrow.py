from PyQt5.QtWidgets import QGraphicsItem
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import QPointF, QRectF, Qt
import math

class GoalArrow(QGraphicsItem):
    def __init__(self):
        super().__init__()
        self.start_point = QPointF()
        self.end_point = QPointF()
        self.arrow_color = QColor(0, 255, 0)
        self.arrow_size = 10

        self.fixed_end_point = QPointF()
        self.arrow_length = 30
        
        self.setZValue(2)
        self.visible = False

    def boundingRect(self):
        if not self.visible:
            return QRectF()
        # return QRectF(self.start_point, self.end_point).normalized().adjusted(-self.arrow_size, -self.arrow_size, self.arrow_size, self.arrow_size)
        return QRectF(
            self.start_point.x() - self.arrow_length - self.arrow_size,
            self.start_point.y() - self.arrow_length - self.arrow_size,
            (self.arrow_length + self.arrow_size) * 2,
            (self.arrow_length + self.arrow_size) * 2
        )

    def paint(self, painter, option, widget):
        if not self.visible:
            return

        painter.setPen(QPen(self.arrow_color, 2, Qt.SolidLine))
        painter.setBrush(self.arrow_color)

        math.atan2(self.end_point.y() - self.start_point.y(), self.end_point.x() - self.start_point.x())

        angle = math.atan2(self.end_point.y() - self.start_point.y(), self.end_point.x() - self.start_point.x())

        self.fixed_end_point = QPointF(
            self.start_point.x() + math.cos(angle) * self.arrow_length,
            self.start_point.y() + math.sin(angle) * self.arrow_length
        )
        painter.drawLine(self.start_point, self.fixed_end_point)

        # arrowhead
        arrowhead_p1 = self.fixed_end_point - QPointF(math.cos(angle - math.pi/6) * self.arrow_size,
                                                math.sin(angle - math.pi/6) * self.arrow_size)
        arrowhead_p2 = self.fixed_end_point - QPointF(math.cos(angle + math.pi/6) * self.arrow_size,
                                                math.sin(angle + math.pi/6) * self.arrow_size)
        painter.drawPolygon(self.fixed_end_point, arrowhead_p1, arrowhead_p2)

    def set_points(self, start, end):
        self.prepareGeometryChange() 
        self.start_point = start
        self.end_point = end
        self.visible = True
        self.update()

    def hide_arrow(self):
        self.visible = False
        self.update()

    def get_angle(self):
        return math.atan2(self.end_point.y() - self.start_point.y(), self.end_point.x() - self.start_point.x())
from PyQt5.QtWidgets import QGraphicsItem
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF
from PyQt5.QtCore import QPointF, QRectF, Qt
import math

class RobotItem(QGraphicsItem):
    def __init__(self):
        super().__init__()
        self.diameter = 12
        self.setZValue(100)  # Always on top, above all route elements 
        
    def boundingRect(self):
        # Make bounding rect large enough to contain both circle and arrow
        return QRectF(-self.diameter/2, -self.diameter/2, self.diameter, self.diameter)
        
    def paint(self, painter: QPainter, option, widget):
        # Set up the black pen for the circle
        painter.scale(1.5,1.5)
        black_pen = QPen(Qt.red)
        black_pen.setWidth(1)
        painter.setPen(black_pen)
        
        # Draw the circle
        painter.drawEllipse(int(-self.diameter/2), int(-self.diameter/2), int(self.diameter), int(self.diameter))
        
        # Set up the brush and pen for the arrow
        painter.setBrush(QBrush(Qt.red))
        
        # Create and draw the arrow polygon
        arrow_polygon = QPolygonF([
            QPointF(6.4, 0),      # Arrow tip
            QPointF(-3.84, -3.84),   # Left corner
            QPointF(-3.84, 3.84),    # Right corner
            QPointF(6.4, 0)       # Back to tip to close polygon
        ])

        painter.drawPolygon(arrow_polygon)
        
    def update_pose(self, x, y, theta):
        """
        Update the robot's position and orientation
        Args:
            x, y: position in scene coordinates
            theta: orientation in radians
        """
        self.setPos(x, y)
        self.setRotation(math.degrees(theta))
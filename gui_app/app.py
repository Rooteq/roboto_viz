import sys
from PyQt5.QtWidgets import (
    QApplication
)
import rclpy

from roboto_viz.main_window import Window

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    # win.updateMapView()
    app.exec_()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
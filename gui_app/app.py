from roboto_viz.gui_state_machine import Gui, DisconnectedState

from PyQt5.QtWidgets import QApplication
import sys

class App(QApplication):
    def __init__(self, argv):
        super(App, self).__init__(argv)
        self.gui = Gui()
        self.gui.setup()
        self.gui.main_view.show()

def main():
    app = App(sys.argv)
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

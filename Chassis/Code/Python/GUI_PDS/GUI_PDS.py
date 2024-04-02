from PyQt6 import QtWidgets, QtCore
from pds_ui import Ui_MainWindow
import time

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self.colors = ["red", "blue", "green", "yellow", "pink"]
        self.current_color = 0
        
        
    def change_color(self):
        self.front_1.setStyleSheet("background-color: {}".format(self.colors[self.current_color]))
        self.current_color = (self.current_color + 1) % len(self.colors)

def change_label_color(window, label, color):
    label.setStyleSheet("background-color: {}".format(color))

app = QtWidgets.QApplication([])
window = MainWindow()
window.show()

# change the color of front_1 to orange
change_label_color(window, window.front_1, "orange")
window.repaint()  # update the window to show the change
time.sleep(5)

# change the color of front_1 to pink
change_label_color(window, window.front_1, "pink")
window.repaint()  # update the window to show the change
app.exec()

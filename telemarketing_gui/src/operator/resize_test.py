from PyQt5 import QtCore, QtGui, QtWidgets
import sys

class MainWindow(QtWidgets.QMainWindow):
    def resizeEvent(self, event):
        print("Window has been resized")
        QtWidgets.QMainWindow.resizeEvent(self, event)


app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
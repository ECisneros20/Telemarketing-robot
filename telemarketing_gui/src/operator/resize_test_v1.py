from PyQt5 import QtCore, QtGui, QtWidgets
import sys

class TestMainWindow(object):
    def resizeEvent(self, event):
        print("Window has been resized")
        QtWidgets.QMainWindow.resizeEvent(self, event)


'''
app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
'''
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = TestMainWindow()
    MainWindow.resizeEvent(None)
    #ui.setupUi(MainWindow)
    #while not rospy.is_shutdown():
    MainWindow.show()
    sys.exit(app.exec_())
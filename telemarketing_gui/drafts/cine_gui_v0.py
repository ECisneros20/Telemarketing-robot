# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'cine_gui_v0.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5.QtGui import QPixmap
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt, pyqtSignal
import cv2

class QLabelClickable(QLabel):
    clicked = pyqtSignal()
    def __init__(self, *args):
        QLabel.__init__(self, *args)
   
    def mouseReleaseEvent(self, ev):
        self.clicked.emit()

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(694, 820)

        #self.setStyleSheet("background-color: yellow;")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.centralwidget.setStyleSheet("background-color: black;")
        #self.centralwidget.setContentsMargins(0,0,0,0)
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.stackedWidget = QtWidgets.QStackedWidget(self.centralwidget)
        self.stackedWidget.setObjectName("stackedWidget")
        self.page = QtWidgets.QWidget()
        self.page.setObjectName("page")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.page)
        self.verticalLayout_2.setObjectName("verticalLayout_2")

        self.label_inicio = QtWidgets.QLabel(self.page)
        self.label_inicio.setObjectName("label_inicio")
        pixmap_inicio = QPixmap("telemarketing_gui/drafts/inicio.png")
        pixmap_inicio=pixmap_inicio.scaled(562,900)
        self.label_inicio.setPixmap(pixmap_inicio)
        self.label_inicio.setScaledContents(True)
        '''
        self.label_button1 = QLabelClickable(self.label_inicio)
        self.label_button1.setObjectName("label_button1")
        pixmap_boton1 = QPixmap("telemarketing_gui/drafts/festival_cine_boton.png")
        pixmap_boton1 = pixmap_boton1.scaled(300,300)
        self.label_button1.setPixmap(pixmap_boton1)
        '''
        '''
        self.layout_inicio = QtWidgets.QVBoxLayout(self.label_inicio)
        self.layout_inicio.setObjectName("layout_inicio")
        self.layout_inicio.addWidget(self.label_button1)
        '''
        self.verticalLayout_2.addWidget(self.label_inicio)



        self.stackedWidget.addWidget(self.page)
        self.page_2 = QtWidgets.QWidget()
        self.page_2.setObjectName("page_2")
        self.stackedWidget.addWidget(self.page_2)
        self.verticalLayout.addWidget(self.stackedWidget)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        #self.label.setText(_translate("MainWindow", "TextLabel"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()

    #img_grayscale = cv2.imread('Cat03.jpg')
    #cv2.imshow('image',img_grayscale)
    #cv2.waitKey(0)

    sys.exit(app.exec_())

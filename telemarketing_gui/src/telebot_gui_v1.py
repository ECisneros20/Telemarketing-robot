#!/usr/bin/env python3

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'telebot_gui_draft1.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!

#comentario para git

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge, CvBridgeError

class ros_setup(QThread):
    Imageupd = pyqtSignal(QImage)
    Batteryupd = pyqtSignal(int)
    Speedupd = pyqtSignal(float)

    #sizeSlot = pyqtSlot(list)

    def run(self):

        self.width = 640
        self.heigth = 480
        image_topic="/usb_cam/image_raw"
        #image_topic="/camera/color/image_raw/compressed"
        speed_topic="/speed"
        battery_topic="/battery_level"

        self.bridge = CvBridge()
        #self.width.connect(self.getwdata)
        #self.height.connect(self.getwdata)
        
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback_image)
        self.speed_sub = rospy.Subscriber(speed_topic,Float32,self.callback_speed)
        self.battery_sub = rospy.Subscriber(battery_topic,Int32,self.callback_battery)

    def size_slot(self,list):
        self.wlabel=list[0]
        self.hlabel=list[1]

    def callback_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        flip = cv_image
        convertir_QT = QImage(flip.data, flip.shape[1], flip.shape[0], QImage.Format_RGB888)
        pic = convertir_QT
        pic = convertir_QT.scaled(self.width,self.heigth, Qt.KeepAspectRatio)
        #pic = convertir_QT.scaled(500, 500, Qt.KeepAspectRatio)
        #pic = convertir_QT.scaled(width, height, Qt.KeepAspectRatio)
        self.Imageupd.emit(pic)

    def callback_battery(self,data):
        battery = data.data
        self.Batteryupd.emit(battery)

    def callback_speed(self,data):
        speed = round(data.data,2)
        self.Speedupd.emit(speed)


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(900, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setMinimumSize(QtCore.QSize(0, 50))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.verticalLayout_2.addWidget(self.label_5)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.label = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setMinimumSize(QtCore.QSize(640, 480))
        self.label.setMaximumSize(QtCore.QSize(1280, 720))
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        spacerItem2 = QtWidgets.QSpacerItem(10, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem2)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.label_3)
        spacerItem3 = QtWidgets.QSpacerItem(20, 60, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        self.verticalLayout.addItem(spacerItem3)
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.verticalLayout.addWidget(self.label_4)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem4)
        self.progressBar = QtWidgets.QProgressBar(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.progressBar.sizePolicy().hasHeightForWidth())
        self.progressBar.setSizePolicy(sizePolicy)
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.horizontalLayout_2.addWidget(self.progressBar)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem5)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        spacerItem6 = QtWidgets.QSpacerItem(10, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem6)
        self.horizontalLayout.addLayout(self.verticalLayout)
        spacerItem7 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem7)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.ros_start()

        #self.sizeList = pyqtSignal(list)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_5.setText(_translate("MainWindow", "Telebot GUI"))
        self.label.setText(_translate("MainWindow", "TextLabel"))
        self.label_2.setText(_translate("MainWindow", "Speed"))
        self.label_3.setText(_translate("MainWindow", "0.0 m/s"))
        self.label_4.setText(_translate("MainWindow", "Battery level"))
        

    def ros_start(self):
        ## Using ros_setup class ##
        self.ros_node = ros_setup()
        rospy.init_node('Telebot_GUI', anonymous=False)
        self.ros_node.start()
        self.ros_node.Imageupd.connect(self.Imageupd_slot)
        self.ros_node.Batteryupd.connect(self.Battery_slot)
        self.ros_node.Speedupd.connect(self.Speed_slot)
        #self.sizeList.connect(self.ros_node.size_slot)



        ## Usign ros in pyqt class ##
        #rospy.init_node('Telebot_GUI', anonymous=False)
        #self.bridge = CvBridge()
        #speed_topic="/speed"
        #battery_topic="/battery_level"
        #image_topic="/usb_cam/image_raw"
        
        #self.image_sub = rospy.Subscriber(image_topic,Image,self.callback_image)
        #self.speed_sub = rospy.Subscriber(speed_topic,Float32,self.callback_speed)
        #self.battery_sub = rospy.Subscriber(battery_topic,Int32,self.callback_battery) # Float32

    def Imageupd_slot(self, Image):
        self.label.setPixmap(QPixmap.fromImage(Image))
        #self.label_width = self.label.width()
        #self.label_height = self.label.height()
        #self.sizeList.emit([self.label_width,self.label_height])

    def Battery_slot(self, data):
        self.progressBar.setValue(data)

    def Speed_slot(self, data):
        self.label_3.setText(str(data)+" m/s")

    def callback_battery(self,data):
        self.progressBar.setValue(data.data)

    def callback_speed(self,data):
        speed = round(data.data,2)
        self.label_3.setText(str(speed)+" m/s")

    def callback_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        flip = cv_image
        convertir_QT = QImage(flip.data, flip.shape[1], flip.shape[0], QImage.Format_RGB888)
        width = self.label.width()
        height = self.label.height()
        pic = convertir_QT.scaled(width, height, Qt.KeepAspectRatio)
        self.label.setPixmap(QPixmap.fromImage(pic))

    def resizeEvent(self, event):
        print("Window has been resized")
        #QtWidgets.QMainWindow.resizeEvent(MainWindow,event)
        self.ros_node.width = self.label.width
        self.ros_node.heigth = self.label.height

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    #while not rospy.is_shutdown():
    MainWindow.show()
    sys.exit(app.exec_())
    #print("Se debe cerrar")
    

    #app = QtWidgets.QApplication(sys.argv)
    #window = Ui_MainWindow()
    #window.show()
    #sys.exit(app.exec_())

    #MainWindow.close()
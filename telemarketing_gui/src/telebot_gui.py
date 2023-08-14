#!/usr/bin/env python3

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import cv2

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge, CvBridgeError

import sys

class image_converter(QThread):
  
  Imageupd = pyqtSignal(QImage)

  Batteryupd = pyqtSignal(int)
  #label = pyqtSignal(QtWidgets.QLabel)
  #progressBar = pyqtSignal(QtWidgets.QProgressBar)

  def run(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    image_topic="/usb_cam/image_raw"
    #image_topic="/camera/color/image_raw/compressed"
    #speed_topic="/speed"
    battery_topic="/battery_level"

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(image_topic,Image,self.callback_image)
    #self.speed_sub = rospy.Subscriber(speed_topic,Float32,self.callback_speed)
    self.battery_sub = rospy.Subscriber(battery_topic,Int32,self.callback_battery)

  def callback_image(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    except CvBridgeError as e:
        print(e)

    (rows,cols,channels) = cv_image.shape
    flip = cv_image
    convertir_QT = QImage(flip.data, flip.shape[1], flip.shape[0], QImage.Format_RGB888)
    pic = convertir_QT.scaled(500, 500, Qt.KeepAspectRatio)
    self.Imageupd.emit(pic)

  def callback_battery(self,data):
    #battery = int(round(data.data,2)*100)
    battery = data.data
    self.Batteryupd.emit(battery)
    #self.progressBar.setProperty("value", battery)

  def callback_speed(self,data):
    speed = round(data.data,2)
    self.label.setText(str(speed))
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

'''
class VideoStream(QThread):
    Imageupd = pyqtSignal(QImage)
    def run(self):
        self.hilo_corriendo = True
        cap = cv2.VideoCapture(6)
        while self.hilo_corriendo:
            ret, frame = cap.read()
            if ret:
                Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                flip = Image
                #flip = cv2.flip(Image, 1)
                convertir_QT = QImage(flip.data, flip.shape[1], flip.shape[0], QImage.Format_RGB888)
                pic = convertir_QT.scaled(500, 500, Qt.KeepAspectRatio)
                #pic = convertir_QT.scaled(320, 240, Qt.KeepAspectRatio)
                self.Imageupd.emit(pic)
    def stop(self):
        self.hilo_corriendo = False
        self.quit()
'''

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(900, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(50, 70, 500, 500))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(320, 10, 161, 41))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(640, 130, 91, 17))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(600, 230, 181, 17))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(660, 180, 67, 17))
        self.label_5.setObjectName("label_5")
        self.label_5.setText("0.00 m/s")
        self.progressBar = QtWidgets.QProgressBar(self.centralwidget)
        self.progressBar.setGeometry(QtCore.QRect(640, 280, 118, 23))
        #self.progressBar.setProperty("value", 24)
        #self.progressBar.setObjectName("progressBar")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.start_video()

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "TextLabel"))
        self.label_2.setText(_translate("MainWindow", "Telebot - GUI"))
        self.label_3.setText(_translate("MainWindow", "Speed"))
        self.label_4.setText(_translate("MainWindow", "Battery state level"))
        #self.label_5.setText(_translate("MainWindow", "TextLabel"))


    def cancel(self):
        self.label.clear()
        self.video.stop()

    def Imageupd_slot(self, Image):
        self.label.setPixmap(QPixmap.fromImage(Image))

    def Battery_slot(self, data):
        self.progressBar.setValue(data)

    def start_video(self):
        #self.video = VideoStream()
        #self.video.start()
        #self.video.Imageupd.connect(self.Imageupd_slot)
        self.video_converter = image_converter()
        rospy.init_node('Telebot_GUI', anonymous=False)

        self.video_converter.start()

        speed_topic="/speed"
        #battery_topic="/battery_level"

        #image_topic="/usb_cam/image_raw"
        #self.image_sub = rospy.Subscriber(image_topic,Image,self.callback_image)

        self.speed_sub = rospy.Subscriber(speed_topic,Float32,self.callback_speed)
        #self.battery_sub = rospy.Subscriber(battery_topic,Int32,self.callback_battery) # Float32
        #rospy.spin()
        #self.video_converter.label = self.label_5
        #self.video_converter.progressBar = self.progressBar
        self.video_converter.Imageupd.connect(self.Imageupd_slot)
        self.video_converter.Batteryupd.connect(self.Battery_slot)

        

    def callback_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        flip = cv_image
        convertir_QT = QImage(flip.data, flip.shape[1], flip.shape[0], QImage.Format_RGB888)
        pic = convertir_QT.scaled(500, 500, Qt.KeepAspectRatio)
        self.label.setPixmap(QPixmap.fromImage(pic))
        #self.Imageupd.emit(pic)

    def callback_battery(self,data):
        #battery = int(round(data.data,2)*100)
        battery = data.data
        print(battery)
        #QPainter.begin(self, self.progressBar)
        self.progressBar.setValue(battery)
        #QPainter.end(self)
        

    def callback_speed(self,data):
        speed = round(data.data,2)
        self.label_5.setText(str(speed)+" m/s")
        

if __name__ == "__main__":
	#try:
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    #rospy.spin()
    sys.exit(app.exec_())
    #except rospy.ROSInterruptException:
    #    MainWindow.close()
    #    pass

    	#except KeyboardInterrupt:
    		#print("Shutting down")


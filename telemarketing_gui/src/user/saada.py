from PyQt5.QtGui import QPixmap
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt, pyqtSignal
import cv2

#from custom_qstacked_widgets import QStackedWidget as StackedWidget


#Sizes for LG Display
'''
WIDTH_FULL = 980
HEIGHT_FULL = 1860

WIDTH_BUTTON_HOME = 400
HEIGHT_BUTTON_HOME = 400

WIDTH_BUTTON_SHORTSMENU = 400
HEIGHT_BUTTON_SHORTSMENU = 400
'''

#Sizes for Develop
WIDTH_FULL = 500
HEIGHT_FULL = 900

WIDTH_BUTTON_HOME = 200
HEIGHT_BUTTON_HOME = 200

WIDTH_BUTTON_SHORTSMENU = 200
HEIGHT_BUTTON_SHORTSMENU = 200

class QLabelClickable(QLabel):
    clicked = pyqtSignal()
    def __init__(self, *args):
        QLabel.__init__(self, *args)
   
    def mouseReleaseEvent(self, ev):
        self.clicked.emit()

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(562, 820)

        #self.setStyleSheet("background-color: yellow;")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.centralwidget.setStyleSheet("background-color: black;")
        #self.centralwidget.setContentsMargins(0,0,0,0)
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.stackedWidget = QtWidgets.QStackedWidget(self.centralwidget)
        #self.stackedWidget = StackedWidget()#self.stackedWidget_qt)
        self.stackedWidget.setObjectName("stackedWidget")
        ##
        #self.stackedWidget.setFadeTransition(True)
        # Set the fade animation duration
        #self.stackedWidget.setFadeSpeed(500)
        # Set the fade easing curve
        #self.stackedWidget.setFadeCurve(QtCore.QEasingCurve.Linear)
        ##
        self.page = QtWidgets.QWidget()
        self.page.setObjectName("page")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.page)
        self.verticalLayout_2.setObjectName("verticalLayout_2")

        self.label_inicio = QtWidgets.QLabel(self.page)
        self.label_inicio.setObjectName("label_inicio")
        pixmap_inicio = QPixmap("src/user/images/inicio/inicio.png")
        pixmap_inicio=pixmap_inicio.scaled(WIDTH_FULL,HEIGHT_FULL)
        self.label_inicio.setPixmap(pixmap_inicio)
        self.label_inicio.setScaledContents(True)

        self.label_button1 = QLabelClickable(self.label_inicio)
        self.label_button1.setObjectName("label_button1")
        pixmap_boton1 = QPixmap("src/user/images/inicio/festival_cine_boton.png")
        pixmap_boton1 = pixmap_boton1.scaled(WIDTH_BUTTON_HOME,HEIGHT_BUTTON_HOME)
        self.label_button1.setPixmap(pixmap_boton1)
        self.label_button1.setScaledContents(True)

        self.label_button2 = QLabelClickable(self.label_inicio)
        self.label_button2.setObjectName("label_button2")
        pixmap_boton2 = QPixmap("src/user/images/inicio/cortometrajes_boton.png")
        pixmap_boton2 = pixmap_boton2.scaled(WIDTH_BUTTON_HOME,HEIGHT_BUTTON_HOME)
        self.label_button2.setPixmap(pixmap_boton2)
        self.label_button2.setScaledContents(True)
        
        
        self.layout_inicio = QtWidgets.QVBoxLayout(self.label_inicio)
        self.layout_inicio.setObjectName("layout_inicio")
        self.layout_inicio.addWidget(self.label_button1)
        #Spacer de los 2 botones en el menu principal
        verticalSpacer1 = QtWidgets.QSpacerItem(20, 450, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.layout_inicio.addItem(verticalSpacer1)
        layout_h = QtWidgets.QHBoxLayout()
        layout_h.setObjectName("layout_h")
        horizontalSpacer1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        horizontalSpacer2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        horizontalSpacer3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        verticalSpacer2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)

        layout_h.addItem(horizontalSpacer1)
        layout_h.addWidget(self.label_button1)
        layout_h.addItem(horizontalSpacer2)
        layout_h.addWidget(self.label_button2)
        layout_h.addItem(horizontalSpacer3)

        self.layout_inicio.addLayout(layout_h)
        self.layout_inicio.addItem(verticalSpacer2)
        #self.label_inicio.setLayout(self.layout_inicio)

        self.verticalLayout_2.addWidget(self.label_inicio)

        self.stackedWidget.addWidget(self.page)
        
        ### Menu de Cortometrajes
        self.page_2 = QtWidgets.QWidget()
        self.page_2.setObjectName("page_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.page_2)
        self.verticalLayout_3.setObjectName("verticalLayout_3")

        self.label_cortometraje = QtWidgets.QLabel(self.page_2)
        self.label_cortometraje.setObjectName("label_cortometraje")
        pixmap_fondo_general = QPixmap("src/user/images/fondo_general.png")
        pixmap_fondo_general=pixmap_fondo_general.scaled(WIDTH_FULL,HEIGHT_FULL)
        self.label_cortometraje.setPixmap(pixmap_fondo_general)
        self.label_cortometraje.setScaledContents(True)
        self.verticalLayout_3.addWidget(self.label_cortometraje)

        self.verticalLayout_cortoMenu = QtWidgets.QVBoxLayout(self.label_cortometraje)
        self.verticalLayout_cortoMenu.setObjectName("verticalLayout_cortoMenu")
        gridLayout = QtWidgets.QGridLayout()#self.label_cortometraje)
        verticalSpacer3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        horizontalSpacer4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)


        horizontalLayout_home = QtWidgets.QHBoxLayout()
        self.label_buttonHome = QLabelClickable(self.label_cortometraje)
        self.label_buttonHome.setObjectName("label_buttonHome")
        pixmap_home = QPixmap("src/user/images/home1.png")
        pixmap_home = pixmap_home.scaled(100,100)
        self.label_buttonHome.setPixmap(pixmap_home)
        self.label_buttonHome.setScaledContents(True)
        horizontalLayout_home.addWidget(self.label_buttonHome)
        horizontalLayout_home.addItem(horizontalSpacer4)

        self.label_corto1 = QLabelClickable(self.label_cortometraje)
        self.label_corto1.setObjectName("label_corto1")
        pixmap_corto1 = QPixmap("src/user/images/corto_menu_1.png")
        pixmap_corto1 = pixmap_corto1.scaled(WIDTH_BUTTON_SHORTSMENU,HEIGHT_BUTTON_SHORTSMENU)
        self.label_corto1.setPixmap(pixmap_corto1)
        self.label_corto1.setScaledContents(True)

        self.label_corto2 = QLabelClickable(self.label_cortometraje)
        self.label_corto2.setObjectName("label_corto2")
        pixmap_corto2 = QPixmap("src/user/images/corto_menu_2.png")
        pixmap_corto2 = pixmap_corto2.scaled(WIDTH_BUTTON_SHORTSMENU,HEIGHT_BUTTON_SHORTSMENU)
        self.label_corto2.setPixmap(pixmap_corto2)
        self.label_corto2.setScaledContents(True)

        self.label_corto3 = QLabelClickable(self.label_cortometraje)
        self.label_corto3.setObjectName("label_corto3")
        pixmap_corto3 = QPixmap("src/user/images/corto_menu_3.png")
        pixmap_corto3 = pixmap_corto3.scaled(WIDTH_BUTTON_SHORTSMENU,HEIGHT_BUTTON_SHORTSMENU)
        self.label_corto3.setPixmap(pixmap_corto3)
        self.label_corto3.setScaledContents(True)

        gridLayout.addItem(verticalSpacer3,0,1)
        gridLayout.addWidget(self.label_corto1,1,1)
        gridLayout.addItem(verticalSpacer3,2,1)
        gridLayout.addWidget(self.label_corto2,3,1)
        gridLayout.addItem(verticalSpacer3,4,1)
        gridLayout.addWidget(self.label_corto3,5,1)
        gridLayout.addItem(verticalSpacer3,6,1)
        gridLayout.addItem(horizontalSpacer4,0,0)
        gridLayout.addItem(horizontalSpacer4,0,2)

        self.verticalLayout_cortoMenu.addLayout(horizontalLayout_home)
        self.verticalLayout_cortoMenu.addLayout(gridLayout)

        self.stackedWidget.addWidget(self.page_2)

        self.page_3 = QtWidgets.QWidget()
        self.page_3.setObjectName("page_3")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.page_3)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        
        self.label_nacional1 = QtWidgets.QLabel(self.page_3)
        self.label_nacional1.setObjectName("label_nacional1")
        pixmap_nacional1 = QPixmap("src/user/images/competencia_nacional/nacional1.png")
        pixmap_nacional1=pixmap_nacional1.scaled(WIDTH_FULL,HEIGHT_FULL)
        self.label_nacional1.setPixmap(pixmap_nacional1)
        self.label_nacional1.setScaledContents(True)

        self.verticalLayout_4.addWidget(self.label_nacional1)


        gridImage1 = QtWidgets.QGridLayout()#self.label_nacional1)
        gridImage1.setObjectName("gridImage1")
        verticalSpacerI1 = QtWidgets.QSpacerItem(20, 350, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        horizontalSpacerI1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        ##Boton Izquierda
        self.label_izq1 = QLabelClickable(self.label_nacional1)
        self.label_izq1.setObjectName("label_izq1")
        pixmap_izq1 = QPixmap("src/user/images/izquierda.png")
        pixmap_izq1=pixmap_izq1.scaled(60,60)
        self.label_izq1.setPixmap(pixmap_izq1)
        self.label_izq1.setStyleSheet("background-color: transparent;")
        #self.label_izq1.setScaledContents(True)
        ##Boton derecha
        self.label_der1 = QLabelClickable(self.label_nacional1)
        self.label_der1.setObjectName("label_der1")
        pixmap_der1 = QPixmap("src/user/images/derecha.png")
        pixmap_der1=pixmap_der1.scaled(60,60)
        #pixmap_der1.fill(Qt.transparent)
        self.label_der1.setPixmap(pixmap_der1)
        self.label_der1.setStyleSheet("background-color: transparent;")
        #self.label_der1.setScaledContents(True)

        gridImage1.addItem(verticalSpacerI1,0,1)
        gridImage1.addWidget(self.label_izq1,1,0)
        gridImage1.addItem(horizontalSpacerI1,1,1)
        gridImage1.addWidget(self.label_der1,1,2)
        gridImage1.addItem(verticalSpacerI1,2,1)



        self.stackedWidget.addWidget(self.page_3)

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

        self.label_button2.clicked.connect(self.gotoCortometrajesMenu)
        self.label_corto1.clicked.connect(self.gotoNacional)
        self.label_buttonHome.clicked.connect(self.gotoHome)

    def gotoCortometrajesMenu(self):
        self.stackedWidget.setCurrentIndex(1)
        #self.stackedWidget.slideToWidgetIndex(1)

    def gotoNacional(self):
        self.stackedWidget.setCurrentIndex(2)

    def gotoHome(self):
        self.stackedWidget.setCurrentIndex(0)

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

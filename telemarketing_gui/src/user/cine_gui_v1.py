from PyQt5.QtGui import QPixmap
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QLabel, QMainWindow, QPushButton, QSlider, QSizePolicy, QStyle, QWidget, QHBoxLayout, QVBoxLayout, QFileDialog
from PyQt5.QtCore import QDir, Qt, QUrl
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
#import cv2

#from custom_qstacked_widgets import QStackedWidget as StackedWidget


#Sizes for LG Display

WIDTH_FULL = 980
HEIGHT_FULL = 1860

WIDTH_BUTTON_HOME = 400
HEIGHT_BUTTON_HOME = 400

WIDTH_BUTTON_SHORTSMENU = 400
HEIGHT_BUTTON_SHORTSMENU = 400

WIDTH_BUTTON_BACKTOHOME = 100
HEIGHT_BUTTON_BACKTOHOME = 100

DISTANCE_MAIN_TOP = 450
DISTANCE_BUTTONS_TOP = 550

'''
#Sizes for Develop
WIDTH_FULL = 500
HEIGHT_FULL = 900

WIDTH_BUTTON_HOME = 200
HEIGHT_BUTTON_HOME = 200

WIDTH_BUTTON_SHORTSMENU = 200
HEIGHT_BUTTON_SHORTSMENU = 200

WIDTH_BUTTON_BACKTOHOME = 200
HEIGHT_BUTTON_BACKTOHOME = 200

DISTANCE_MAIN_TOP = 200
DISTANCE_BUTTONS_TOP = 300
'''

class VideoPlayer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt5 Video Player") 
 
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
 
        videoWidget = QVideoWidget()
        self.videotag = None
        self.playButton = QPushButton()
        self.playButton.setEnabled(False)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)
 
        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderMoved.connect(self.setPosition)
 
        self.error = QLabel()
        self.error.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
 
        #openButton = QPushButton("Open Video")   
        #openButton.setToolTip("Open Video File")
        #openButton.setStatusTip("Open Video File")
        #openButton.setFixedHeight(24)
        #openButton.clicked.connect(self.openFile)
 
 
        # Create a widget for window contents
        wid = QWidget(self)
        self.setCentralWidget(wid)
 
        # Create layouts to place inside widget
        controlLayout = QHBoxLayout()
        controlLayout.setContentsMargins(0, 0, 0, 0)
        controlLayout.addWidget(self.playButton)
        controlLayout.addWidget(self.positionSlider)
 
        layout = QVBoxLayout()
        layout.addWidget(videoWidget)
        layout.addLayout(controlLayout)
        layout.addWidget(self.error)
        #layout.addWidget(openButton)
 
        # Set widget to contain window contents
        wid.setLayout(layout)

        self.openVideo()
        self.mediaPlayer.setVideoOutput(videoWidget)
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)
        self.mediaPlayer.error.connect(self.handleError)
 
    def openVideo(self):
        self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile("videos/cansat1.mp4")))
        self.playButton.setEnabled(True)
        self.mediaPlayer.play()
        

    def openFile(self):
        fileName, _ = QFileDialog.getOpenFileName(self, "Open Movie",
                QDir.homePath())
 
        if fileName != '':
            self.mediaPlayer.setMedia(
                    QMediaContent(QUrl.fromLocalFile(fileName)))
            self.playButton.setEnabled(True)
 
    def exitCall(self):
        sys.exit(app.exec_())
 
    def play(self):
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.mediaPlayer.pause()
        else:
            self.mediaPlayer.play()
 
    def mediaStateChanged(self, state):
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.playButton.setIcon(
                    self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(
                    self.style().standardIcon(QStyle.SP_MediaPlay))
 
    def positionChanged(self, position):
        self.positionSlider.setValue(position)
 
    def durationChanged(self, duration):
        self.positionSlider.setRange(0, duration)
 
    def setPosition(self, position):
        self.mediaPlayer.setPosition(position)
 
    def handleError(self):
        self.playButton.setEnabled(False)
        self.error.setText("Error: " + self.mediaPlayer.errorString())

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
        verticalSpacer1 = QtWidgets.QSpacerItem(20, DISTANCE_MAIN_TOP, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
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
        pixmap_home = QPixmap("src/user/images/home.jpg")
        pixmap_home = pixmap_home.scaled(WIDTH_BUTTON_BACKTOHOME,HEIGHT_BUTTON_BACKTOHOME)
        self.label_buttonHome.setPixmap(pixmap_home)
        self.label_back = QLabelClickable(self.label_cortometraje)
        self.label_back.setObjectName("label_back")
        pixmap_back = QPixmap("src/user/images/back.jpg")
        pixmap_back = pixmap_back.scaled(WIDTH_BUTTON_BACKTOHOME,HEIGHT_BUTTON_BACKTOHOME)
        self.label_back.setPixmap(pixmap_back)
        #self.label_buttonHome.setScaledContents(True)
        horizontalLayout_home.addWidget(self.label_buttonHome)
        horizontalLayout_home.addItem(horizontalSpacer4)
        horizontalLayout_home.addWidget(self.label_back)

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
        
        # INDEX 2
        self.page_3 = QtWidgets.QWidget()
        self.page_3.setObjectName("page_3")
        self.verticalLayOutNacional1 = QtWidgets.QVBoxLayout(self.page_3)
        self.verticalLayOutNacional1.setObjectName("verticalLayOutNacional1")
        self.label_nacional1 = QtWidgets.QLabel(self.page_3)
        self.label_nacional1.setObjectName("label_nacional1")
        pixmap_nacional1 = QPixmap("src/user/images/competencia_nacional/nacional1.png")
        pixmap_nacional1=pixmap_nacional1.scaled(WIDTH_FULL,HEIGHT_FULL)
        self.label_nacional1.setPixmap(pixmap_nacional1)
        self.label_nacional1.setScaledContents(True)
        self.verticalLayOutNacional1.addWidget(self.label_nacional1)
        # INDEX 3
        self.page_4 = QtWidgets.QWidget()
        self.page_4.setObjectName("page_4")
        self.verticalLayOutNacional2 = QtWidgets.QVBoxLayout(self.page_4)
        self.verticalLayOutNacional2.setObjectName("verticalLayOutNacional2")
        self.label_nacional2 = QtWidgets.QLabel(self.page_4)
        self.label_nacional2.setObjectName("label_nacional2")
        pixmap_nacional2 = QPixmap("src/user/images/competencia_nacional/nacional2.png")
        pixmap_nacional2=pixmap_nacional2.scaled(WIDTH_FULL,HEIGHT_FULL)
        self.label_nacional2.setPixmap(pixmap_nacional2)
        self.label_nacional2.setScaledContents(True)
        self.verticalLayOutNacional2.addWidget(self.label_nacional2)
        # INDEX 4
        self.page_5 = QtWidgets.QWidget()
        self.page_5.setObjectName("page_5")
        self.verticalLayOutNacional3 = QtWidgets.QVBoxLayout(self.page_5)
        self.verticalLayOutNacional3.setObjectName("verticalLayOutNacional3")
        self.label_nacional3 = QtWidgets.QLabel(self.page_5)
        self.label_nacional3.setObjectName("label_nacional3")
        pixmap_nacional3 = QPixmap("src/user/images/competencia_nacional/nacional3.png")
        pixmap_nacional3=pixmap_nacional3.scaled(WIDTH_FULL,HEIGHT_FULL)
        self.label_nacional3.setPixmap(pixmap_nacional3)
        self.label_nacional3.setScaledContents(True)
        self.verticalLayOutNacional3.addWidget(self.label_nacional3)
        # INDEX 5
        self.page_6 = QtWidgets.QWidget()
        self.page_6.setObjectName("page_6")
        self.verticalLayOutNacional4 = QtWidgets.QVBoxLayout(self.page_6)
        self.verticalLayOutNacional4.setObjectName("verticalLayOutNacional4")
        self.label_nacional4 = QtWidgets.QLabel(self.page_6)
        self.label_nacional4.setObjectName("label_nacional4")
        pixmap_nacional4 = QPixmap("src/user/images/competencia_nacional/nacional4.png")
        pixmap_nacional4=pixmap_nacional4.scaled(WIDTH_FULL,HEIGHT_FULL)
        self.label_nacional4.setPixmap(pixmap_nacional4)
        self.label_nacional4.setScaledContents(True)
        self.verticalLayOutNacional4.addWidget(self.label_nacional4)
        # INDEX 6
        self.page_7 = QtWidgets.QWidget()
        self.page_7.setObjectName("page_7")
        self.verticalLayOutNacional5 = QtWidgets.QVBoxLayout(self.page_7)
        self.verticalLayOutNacional5.setObjectName("verticalLayOutNacional5")
        self.label_nacional5 = QtWidgets.QLabel(self.page_4)
        self.label_nacional5.setObjectName("label_nacional5")
        pixmap_nacional5 = QPixmap("src/user/images/competencia_nacional/nacional5.png")
        pixmap_nacional5=pixmap_nacional5.scaled(WIDTH_FULL,HEIGHT_FULL)
        self.label_nacional5.setPixmap(pixmap_nacional5)
        self.label_nacional5.setScaledContents(True)
        self.verticalLayOutNacional5.addWidget(self.label_nacional5)

        self.stackedWidget.addWidget(self.page_3)
        self.stackedWidget.addWidget(self.page_4)
        self.stackedWidget.addWidget(self.page_5)
        self.stackedWidget.addWidget(self.page_6)
        self.stackedWidget.addWidget(self.page_7)

        self.labelNacional_list=[]
        self.layoutImagesList = []
        self.labelHomelist = []
        self.labelBacklist = []
        for i in range(5):
            vlayout_image = QtWidgets.QVBoxLayout()
            vlayout_image.setObjectName("vlayout_image"+str(i+1))
            hlayout_home = QtWidgets.QHBoxLayout()
            label_buttonHome = QLabelClickable()
            label_buttonHome.setObjectName("label_buttonHome"+str(i+1))
            pixmap_home = QPixmap("src/user/images/home.jpg")
            pixmap_home = pixmap_home.scaled(WIDTH_BUTTON_BACKTOHOME,HEIGHT_BUTTON_BACKTOHOME)
            label_buttonHome.setPixmap(pixmap_home)
            #label_buttonHome.setScaledContents(True)
            self.labelHomelist.append(label_buttonHome)
            #self.label_buttonHome.setScaledContents(True)
            label_back = QLabelClickable()
            label_back.setObjectName("label_back"+str(i+1))
            pixmap_back = QPixmap("src/user/images/back.jpg")
            pixmap_back = pixmap_back.scaled(WIDTH_BUTTON_BACKTOHOME,HEIGHT_BUTTON_BACKTOHOME)
            label_back.setPixmap(pixmap_back)
            self.labelBacklist.append(label_back)
            hlayout_home.addWidget(label_buttonHome)
            horizontalSpacerHome = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
            hlayout_home.addItem(horizontalSpacerHome)
            hlayout_home.addWidget(label_back)
            gridImage = QtWidgets.QGridLayout()
            gridImage.setObjectName("gridImage"+str(i+1))
            verticalSpacerIm = QtWidgets.QSpacerItem(20, DISTANCE_BUTTONS_TOP, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
            verticalSpacerIm2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
            horizontalSpacerIm = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
            ##Boton Izquierda
            self.label_izq = QLabelClickable()#self.label_nacional1)
            self.label_izq.setObjectName("label_izq")
            pixmap_izq1 = QPixmap("src/user/images/left.jpg")
            pixmap_izq1=pixmap_izq1.scaled(100,100)
            self.label_izq.setPixmap(pixmap_izq1)
            self.label_izq.setStyleSheet("background-color: transparent;")
            #self.label_izq1.setScaledContents(True)
            ##Boton derecha
            self.label_der = QLabelClickable()#self.label_nacional1)
            self.label_der.setObjectName("label_der")
            pixmap_der1 = QPixmap("src/user/images/right.jpg")
            pixmap_der1=pixmap_der1.scaled(100,100)
            self.label_der.setPixmap(pixmap_der1)
            self.label_der.setStyleSheet("background-color: transparent;")
            #self.label_der1.setScaledContents(True)
            labels=[self.label_izq,self.label_der]
            self.labelNacional_list.append(labels)
            gridImage.addItem(verticalSpacerIm,0,1)
            gridImage.addWidget(self.label_izq,1,0)
            gridImage.addItem(horizontalSpacerIm,1,1)
            gridImage.addWidget(self.label_der,1,2)
            gridImage.addItem(verticalSpacerIm2,2,1)

            vlayout_image.addLayout(hlayout_home)
            vlayout_image.addLayout(gridImage)
            self.layoutImagesList.append(vlayout_image)

        self.label_nacional1.setLayout(self.layoutImagesList[0])
        self.label_nacional2.setLayout(self.layoutImagesList[1])
        self.label_nacional3.setLayout(self.layoutImagesList[2])
        self.label_nacional4.setLayout(self.layoutImagesList[3])
        self.label_nacional5.setLayout(self.layoutImagesList[4])

        #Ibero Americana
        # INDEX 2
        label_iberoamericana_list = []
        self.labels_iberoamericana = []
        self.labelBacklist_ibero = []
        self.layoutImagesListIbero = []
        self.labelHomelistIbero = []
        for i in range(7):
            page = QtWidgets.QWidget()
            page.setObjectName("page_"+str(i+8))
            verticaliberoamericana = QtWidgets.QVBoxLayout(page)
            verticaliberoamericana.setObjectName("verticaliberoamericana"+str(i+1))
            label_iberoamericana = QtWidgets.QLabel(page)
            label_iberoamericana.setObjectName("label_iberoamericana"+str(i+1))
            file = "iberoamericana"+str(i+1)+".png"
            pixmap_iberoamericana = QPixmap("src/user/images/iberoamericana/"+file)
            pixmap_iberoamericana=pixmap_iberoamericana.scaled(WIDTH_FULL,HEIGHT_FULL)
            label_iberoamericana.setPixmap(pixmap_iberoamericana)
            label_iberoamericana.setScaledContents(True)
            label_iberoamericana_list.append(label_iberoamericana)
            verticaliberoamericana.addWidget(label_iberoamericana)
            self.stackedWidget.addWidget(page)

            ##Layout en cada hoja
            vlayout_image = QtWidgets.QVBoxLayout()
            vlayout_image.setObjectName("vlayout_image"+str(i+6))
            hlayout_home = QtWidgets.QHBoxLayout()
            label_buttonHome = QLabelClickable()
            label_buttonHome.setObjectName("label_buttonHome"+str(i+6))
            pixmap_home = QPixmap("src/user/images/home.jpg")
            pixmap_home = pixmap_home.scaled(WIDTH_BUTTON_BACKTOHOME,HEIGHT_BUTTON_BACKTOHOME)
            label_buttonHome.setPixmap(pixmap_home)
            #label_buttonHome.setScaledContents(True)
            self.labelHomelistIbero.append(label_buttonHome)
            #self.label_buttonHome.setScaledContents(True)
            label_back = QLabelClickable()
            label_back.setObjectName("label_back"+str(i+6))
            pixmap_back = QPixmap("src/user/images/back.jpg")
            pixmap_back = pixmap_back.scaled(WIDTH_BUTTON_BACKTOHOME,HEIGHT_BUTTON_BACKTOHOME)
            label_back.setPixmap(pixmap_back)
            self.labelBacklist_ibero.append(label_back)
            hlayout_home.addWidget(label_buttonHome)
            horizontalSpacerHome = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
            hlayout_home.addItem(horizontalSpacerHome)
            hlayout_home.addWidget(label_back)
            gridImage = QtWidgets.QGridLayout()
            gridImage.setObjectName("gridImage"+str(i+6))
            verticalSpacerIm = QtWidgets.QSpacerItem(20, DISTANCE_BUTTONS_TOP, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
            verticalSpacerIm2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
            horizontalSpacerIm = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
            ##Boton Izquierda
            self.label_izq = QLabelClickable()#self.label_nacional1)
            self.label_izq.setObjectName("label_izq")
            pixmap_izq1 = QPixmap("src/user/images/left.jpg")
            pixmap_izq1=pixmap_izq1.scaled(100,100)
            self.label_izq.setPixmap(pixmap_izq1)
            self.label_izq.setStyleSheet("background-color: transparent;")
            #self.label_izq1.setScaledContents(True)
            ##Boton derecha
            self.label_der = QLabelClickable()#self.label_nacional1)
            self.label_der.setObjectName("label_der")
            pixmap_der1 = QPixmap("src/user/images/right.jpg")
            pixmap_der1=pixmap_der1.scaled(100,100)
            self.label_der.setPixmap(pixmap_der1)
            self.label_der.setStyleSheet("background-color: transparent;")
            #self.label_der1.setScaledContents(True)
            labels=[self.label_izq,self.label_der]
            self.labels_iberoamericana.append(labels)
            gridImage.addItem(verticalSpacerIm,0,1)
            gridImage.addWidget(self.label_izq,1,0)
            gridImage.addItem(horizontalSpacerIm,1,1)
            gridImage.addWidget(self.label_der,1,2)
            gridImage.addItem(verticalSpacerIm2,2,1)
            vlayout_image.addLayout(hlayout_home)
            vlayout_image.addLayout(gridImage)
            self.layoutImagesListIbero.append(vlayout_image)

        for i in range(len(label_iberoamericana_list)):
            label_iberoamericana_list[i].setLayout(self.layoutImagesListIbero[i])
        
        # INDEX 14
        self.page_14 = QtWidgets.QWidget()
        self.page_14.setObjectName("page_14")
        self.layoutVideo1 = QtWidgets.QVBoxLayout(self.page_14)
        self.layoutVideo1.setObjectName("layoutVideo1")
        #self.label_nacional2 = QtWidgets.QLabel(self.page_14)
        #self.label_nacional2.setObjectName("label_nacional2")
        #pixmap_nacional2 = QPixmap("src/user/videos/Aeroespacial_GRP.mp4")
        #pixmap_nacional2=pixmap_nacional2.scaled(WIDTH_FULL,HEIGHT_FULL)
        #self.label_nacional2.setPixmap(pixmap_nacional2)
        #self.label_nacional2.setScaledContents(True)
        #self.layoutVideo1.addWidget(self.label_nacional2)
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        videoWidget = QVideoWidget()
        self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile("src/user/videos/Aeroespacial_GRP.mp4")))
        self.mediaPlayer.setVideoOutput(videoWidget)
        self.layoutVideo1.addWidget(videoWidget)
        self.stackedWidget.addWidget(self.page_14)
        
        ####
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

        #Solo de prueba de video
        self.label_button1.clicked.connect(self.gotoVideo1)
        #
        self.label_button2.clicked.connect(self.gotoCortometrajesMenu)
        self.label_corto1.clicked.connect(self.gotoNacional)
        self.label_corto2.clicked.connect(self.gotoIberoamericana)
        self.label_buttonHome.clicked.connect(self.gotoHome)
        self.label_back.clicked.connect(self.goback)

        for label in self.labelHomelist:
            label.clicked.connect(self.gotoHome)
        for label in self.labelHomelistIbero:
            label.clicked.connect(self.gotoHome)

        #self.label_izq1.clicked.connect(self.gotoPrevious)
        #self.label_der1.clicked.connect(self.gotoNext)
        for labels in self.labelNacional_list:
            label_izq = labels[0]
            label_der = labels[1]
            label_izq.clicked.connect(self.gotoPrevious)
            label_der.clicked.connect(self.gotoNext)

        for labels in self.labels_iberoamericana:
            label_izq = labels[0]
            label_der = labels[1]
            label_izq.clicked.connect(self.gotoPreviousIbero)
            label_der.clicked.connect(self.gotoNextIbero)
        
        for label in self.labelBacklist:
            label.clicked.connect(self.goback)
        
        for label in self.labelBacklist_ibero:
            label.clicked.connect(self.goback)

    def gotoVideo1(self):
        self.stackedWidget.setCurrentIndex(14)
        self.mediaPlayer.play()

    def gotoIberoamericana(self):
        self.stackedWidget.setCurrentIndex(7)

    def goback(self):
        actual_index = self.stackedWidget.currentIndex()
        if actual_index==1:
            self.gotoHome()
        elif actual_index>=2 and actual_index<=13:
            self.gotoCortometrajesMenu()

    def gotoCortometrajesMenu(self):
        self.stackedWidget.setCurrentIndex(1)
        #self.stackedWidget.slideToWidgetIndex(1)

    def gotoNacional(self):
        self.stackedWidget.setCurrentIndex(2)

    def gotoHome(self):
        self.stackedWidget.setCurrentIndex(0)

    def gotoNext(self):
        actual_index = self.stackedWidget.currentIndex()
        if actual_index<6:
            self.stackedWidget.setCurrentIndex(actual_index+1)
        else:
            self.stackedWidget.setCurrentIndex(2)

    def gotoNextIbero(self):
        actual_index = self.stackedWidget.currentIndex()
        if actual_index<13:
            self.stackedWidget.setCurrentIndex(actual_index+1)
        else:
            self.stackedWidget.setCurrentIndex(7)

    def gotoPrevious(self):
        actual_index = self.stackedWidget.currentIndex()
        if actual_index>2:
            self.stackedWidget.setCurrentIndex(actual_index-1)
        else:
            self.stackedWidget.setCurrentIndex(6)

    def gotoPreviousIbero(self):
        actual_index = self.stackedWidget.currentIndex()
        if actual_index>7:
            self.stackedWidget.setCurrentIndex(actual_index-1)
        else:
            self.stackedWidget.setCurrentIndex(13)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    #MainWindow.show()
    MainWindow.showMaximized()
    #img_grayscale = cv2.imread('Cat03.jpg')
    #cv2.imshow('image',img_grayscale)
    #cv2.waitKey(0)

    # xinput set-prop "Multi touch   Multi touch overlay device" --type=float "Coordinate Transformation Matrix" 0 -1 1 1 0 0 0 0 1


    sys.exit(app.exec_())

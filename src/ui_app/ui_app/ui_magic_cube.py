# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ui_magic_cube.ui'
##
## Created by: Qt User Interface Compiler version 6.9.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QButtonGroup, QFrame, QGridLayout,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPushButton, QScrollArea, QSizePolicy, QStackedWidget,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1280, 799)
        MainWindow.setMouseTracking(False)
        MainWindow.setStyleSheet(u"")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.centralwidget.setEnabled(True)
        self.centralwidget.setAutoFillBackground(False)
        self.centralwidget.setStyleSheet(u"background-color: #0E3D52;")
        self.horizontalLayout_4 = QHBoxLayout(self.centralwidget)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.BackgroundWidget = QWidget(self.centralwidget)
        self.BackgroundWidget.setObjectName(u"BackgroundWidget")
        self.BackgroundWidget.setEnabled(True)
        self.BackgroundWidget.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: #000000;\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: rgba(11, 118, 160, 0.3); /* soft blue overlay */\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"QLabel {\n"
"	color:white;\n"
"}")
        self.SystemSettingsButton = QPushButton(self.BackgroundWidget)
        self.MenuButtonGroup = QButtonGroup(MainWindow)
        self.MenuButtonGroup.setObjectName(u"MenuButtonGroup")
        self.MenuButtonGroup.addButton(self.SystemSettingsButton)
        self.SystemSettingsButton.setObjectName(u"SystemSettingsButton")
        self.SystemSettingsButton.setGeometry(QRect(10, 680, 223, 94))
        font = QFont()
        font.setPointSize(12)
        self.SystemSettingsButton.setFont(font)
        self.SystemSettingsButton.setLayoutDirection(Qt.LayoutDirection.LeftToRight)
        self.SystemSettingsButton.setStyleSheet(u"")
        icon = QIcon()
        icon.addFile(u":/icons/white/settings.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.SystemSettingsButton.setIcon(icon)
        self.SystemSettingsButton.setCheckable(True)
        self.SignalLightsWidget = QWidget(self.BackgroundWidget)
        self.SignalLightsWidget.setObjectName(u"SignalLightsWidget")
        self.SignalLightsWidget.setGeometry(QRect(10, 90, 221, 81))
        self.SignalLightsWidget.setStyleSheet(u"background-color:#000000; ")
        self.horizontalLayout = QHBoxLayout(self.SignalLightsWidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(22, 22, 22, 22)
        self.RedSignal = QLabel(self.SignalLightsWidget)
        self.RedSignal.setObjectName(u"RedSignal")
        self.RedSignal.setMaximumSize(QSize(30, 30))
        self.RedSignal.setStyleSheet(u"background-color: #660000; border-radius: 10px;")

        self.horizontalLayout.addWidget(self.RedSignal)

        self.YellowSignal = QLabel(self.SignalLightsWidget)
        self.YellowSignal.setObjectName(u"YellowSignal")
        self.YellowSignal.setMaximumSize(QSize(30, 30))
        self.YellowSignal.setStyleSheet(u"background-color: #666633; border-radius: 10px;")

        self.horizontalLayout.addWidget(self.YellowSignal)

        self.GreenSignal = QLabel(self.SignalLightsWidget)
        self.GreenSignal.setObjectName(u"GreenSignal")
        self.GreenSignal.setMaximumSize(QSize(30, 30))
        self.GreenSignal.setStyleSheet(u"background-color: #336633; border-radius: 10px;")

        self.horizontalLayout.addWidget(self.GreenSignal)

        self.Line = QFrame(self.BackgroundWidget)
        self.Line.setObjectName(u"Line")
        self.Line.setGeometry(QRect(0, 74, 1271, 16))
        self.Line.setStyleSheet(u"border-top: 1px solid white;")
        self.Line.setFrameShape(QFrame.Shape.HLine)
        self.Line.setFrameShadow(QFrame.Shadow.Plain)
        self.Line.setLineWidth(1)
        self.DeltaLogo = QLabel(self.BackgroundWidget)
        self.DeltaLogo.setObjectName(u"DeltaLogo")
        self.DeltaLogo.setGeometry(QRect(20, 20, 131, 41))
        self.DeltaLogo.setPixmap(QPixmap(u":/images/delta-logo.png"))
        self.DeltaLogo.setScaledContents(True)
        self.MenuButtons = QWidget(self.BackgroundWidget)
        self.MenuButtons.setObjectName(u"MenuButtons")
        self.MenuButtons.setGeometry(QRect(10, 170, 221, 461))
        self.verticalLayout_11 = QVBoxLayout(self.MenuButtons)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.MainPageButton = QPushButton(self.MenuButtons)
        self.MenuButtonGroup.addButton(self.MainPageButton)
        self.MainPageButton.setObjectName(u"MainPageButton")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.MainPageButton.sizePolicy().hasHeightForWidth())
        self.MainPageButton.setSizePolicy(sizePolicy)
        self.MainPageButton.setFont(font)
        self.MainPageButton.setAutoFillBackground(False)
        self.MainPageButton.setStyleSheet(u"")
        icon1 = QIcon()
        icon1.addFile(u":/icons/white/home-simple-door.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.MainPageButton.setIcon(icon1)
        self.MainPageButton.setCheckable(True)
        self.MainPageButton.setChecked(True)

        self.verticalLayout_11.addWidget(self.MainPageButton)

        self.ComponentControlButton = QPushButton(self.MenuButtons)
        self.MenuButtonGroup.addButton(self.ComponentControlButton)
        self.ComponentControlButton.setObjectName(u"ComponentControlButton")
        sizePolicy.setHeightForWidth(self.ComponentControlButton.sizePolicy().hasHeightForWidth())
        self.ComponentControlButton.setSizePolicy(sizePolicy)
        self.ComponentControlButton.setFont(font)
        self.ComponentControlButton.setStyleSheet(u"")
        icon2 = QIcon()
        icon2.addFile(u":/icons/white/dimmer-switch.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ComponentControlButton.setIcon(icon2)
        self.ComponentControlButton.setCheckable(True)

        self.verticalLayout_11.addWidget(self.ComponentControlButton)

        self.ProductionRecordButton = QPushButton(self.MenuButtons)
        self.MenuButtonGroup.addButton(self.ProductionRecordButton)
        self.ProductionRecordButton.setObjectName(u"ProductionRecordButton")
        sizePolicy.setHeightForWidth(self.ProductionRecordButton.sizePolicy().hasHeightForWidth())
        self.ProductionRecordButton.setSizePolicy(sizePolicy)
        self.ProductionRecordButton.setFont(font)
        self.ProductionRecordButton.setStyleSheet(u"")
        icon3 = QIcon()
        icon3.addFile(u":/icons/white/page.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ProductionRecordButton.setIcon(icon3)
        self.ProductionRecordButton.setCheckable(True)

        self.verticalLayout_11.addWidget(self.ProductionRecordButton)

        self.LogsButton = QPushButton(self.MenuButtons)
        self.MenuButtonGroup.addButton(self.LogsButton)
        self.LogsButton.setObjectName(u"LogsButton")
        sizePolicy.setHeightForWidth(self.LogsButton.sizePolicy().hasHeightForWidth())
        self.LogsButton.setSizePolicy(sizePolicy)
        self.LogsButton.setFont(font)
        self.LogsButton.setStyleSheet(u"")
        icon4 = QIcon()
        icon4.addFile(u":/icons/white/multiple-pages.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.LogsButton.setIcon(icon4)
        self.LogsButton.setCheckable(True)

        self.verticalLayout_11.addWidget(self.LogsButton)

        self.ParentStackedWidgetToChangeMenuOptions = QStackedWidget(self.BackgroundWidget)
        self.ParentStackedWidgetToChangeMenuOptions.setObjectName(u"ParentStackedWidgetToChangeMenuOptions")
        self.ParentStackedWidgetToChangeMenuOptions.setGeometry(QRect(240, 90, 1021, 691))
        font1 = QFont()
        font1.setPointSize(18)
        self.ParentStackedWidgetToChangeMenuOptions.setFont(font1)
        self.ParentStackedWidgetToChangeMenuOptions.setStyleSheet(u"")
        self.MainPagePage = QWidget()
        self.MainPagePage.setObjectName(u"MainPagePage")
        self.CameraWidget = QWidget(self.MainPagePage)
        self.CameraWidget.setObjectName(u"CameraWidget")
        self.CameraWidget.setGeometry(QRect(0, 0, 701, 491))
        self.CameraWidget.setStyleSheet(u"background-color: #000000;")
        self.horizontalLayout_2 = QHBoxLayout(self.CameraWidget)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.VisionText = QLabel(self.CameraWidget)
        self.VisionText.setObjectName(u"VisionText")
        self.VisionText.setStyleSheet(u"color:white;")
        self.VisionText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_2.addWidget(self.VisionText, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)

        self.AutoAndManualWidget = QWidget(self.MainPagePage)
        self.AutoAndManualWidget.setObjectName(u"AutoAndManualWidget")
        self.AutoAndManualWidget.setGeometry(QRect(720, 10, 291, 681))
        self.verticalLayout_24 = QVBoxLayout(self.AutoAndManualWidget)
        self.verticalLayout_24.setObjectName(u"verticalLayout_24")
        self.AutoManualButtons = QWidget(self.AutoAndManualWidget)
        self.AutoManualButtons.setObjectName(u"AutoManualButtons")
        self.horizontalLayout_5 = QHBoxLayout(self.AutoManualButtons)
        self.horizontalLayout_5.setSpacing(25)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.AutoButton = QPushButton(self.AutoManualButtons)
        self.AutoAndManualButtonGroup = QButtonGroup(MainWindow)
        self.AutoAndManualButtonGroup.setObjectName(u"AutoAndManualButtonGroup")
        self.AutoAndManualButtonGroup.addButton(self.AutoButton)
        self.AutoButton.setObjectName(u"AutoButton")
        sizePolicy.setHeightForWidth(self.AutoButton.sizePolicy().hasHeightForWidth())
        self.AutoButton.setSizePolicy(sizePolicy)
        self.AutoButton.setMinimumSize(QSize(0, 74))
        self.AutoButton.setFont(font)
        self.AutoButton.setStyleSheet(u"")
        self.AutoButton.setCheckable(True)
        self.AutoButton.setChecked(False)

        self.horizontalLayout_5.addWidget(self.AutoButton)

        self.ManualButton = QPushButton(self.AutoManualButtons)
        self.AutoAndManualButtonGroup.addButton(self.ManualButton)
        self.ManualButton.setObjectName(u"ManualButton")
        sizePolicy.setHeightForWidth(self.ManualButton.sizePolicy().hasHeightForWidth())
        self.ManualButton.setSizePolicy(sizePolicy)
        self.ManualButton.setMinimumSize(QSize(0, 74))
        self.ManualButton.setFont(font)
        self.ManualButton.setStyleSheet(u"")
        self.ManualButton.setCheckable(True)
        self.ManualButton.setChecked(False)

        self.horizontalLayout_5.addWidget(self.ManualButton)


        self.verticalLayout_24.addWidget(self.AutoManualButtons)

        self.ActionButtons = QStackedWidget(self.AutoAndManualWidget)
        self.ActionButtons.setObjectName(u"ActionButtons")
        self.ActionButtons.setStyleSheet(u"")
        self.ActionButtonsPage = QWidget()
        self.ActionButtonsPage.setObjectName(u"ActionButtonsPage")
        self.verticalLayout_12 = QVBoxLayout(self.ActionButtonsPage)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.INITButton = QPushButton(self.ActionButtonsPage)
        self.INITButton.setObjectName(u"INITButton")
        sizePolicy.setHeightForWidth(self.INITButton.sizePolicy().hasHeightForWidth())
        self.INITButton.setSizePolicy(sizePolicy)
        self.INITButton.setFont(font)
        self.INITButton.setStyleSheet(u"\n"
"\n"
"QPushButton#INITButton:pressed {\n"
"    background-color: #FFB300;     /* darker pressed */\n"
"}")

        self.verticalLayout_12.addWidget(self.INITButton)

        self.RunButton = QPushButton(self.ActionButtonsPage)
        self.RunButton.setObjectName(u"RunButton")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Ignored)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.RunButton.sizePolicy().hasHeightForWidth())
        self.RunButton.setSizePolicy(sizePolicy1)
        self.RunButton.setFont(font)
        self.RunButton.setStyleSheet(u"\n"
"QPushButton#RunButton:pressed {\n"
"    background-color: transparent;\n"
"}")

        self.verticalLayout_12.addWidget(self.RunButton)

        self.ActionButtons.addWidget(self.ActionButtonsPage)
        self.ManualButtonsPage = QWidget()
        self.ManualButtonsPage.setObjectName(u"ManualButtonsPage")
        self.ManualButtonsPage.setStyleSheet(u"")
        self.verticalLayout_20 = QVBoxLayout(self.ManualButtonsPage)
        self.verticalLayout_20.setObjectName(u"verticalLayout_20")
        self.RoughAlignButton = QPushButton(self.ManualButtonsPage)
        self.ManualButtons = QButtonGroup(MainWindow)
        self.ManualButtons.setObjectName(u"ManualButtons")
        self.ManualButtons.setExclusive(False)
        self.ManualButtons.addButton(self.RoughAlignButton)
        self.RoughAlignButton.setObjectName(u"RoughAlignButton")
        sizePolicy.setHeightForWidth(self.RoughAlignButton.sizePolicy().hasHeightForWidth())
        self.RoughAlignButton.setSizePolicy(sizePolicy)
        self.RoughAlignButton.setMaximumSize(QSize(16777215, 90))
        self.RoughAlignButton.setFont(font)
        self.RoughAlignButton.setStyleSheet(u"")
        self.RoughAlignButton.setCheckable(True)

        self.verticalLayout_20.addWidget(self.RoughAlignButton)

        self.PreciseAlignButton = QPushButton(self.ManualButtonsPage)
        self.ManualButtons.addButton(self.PreciseAlignButton)
        self.PreciseAlignButton.setObjectName(u"PreciseAlignButton")
        sizePolicy.setHeightForWidth(self.PreciseAlignButton.sizePolicy().hasHeightForWidth())
        self.PreciseAlignButton.setSizePolicy(sizePolicy)
        self.PreciseAlignButton.setMaximumSize(QSize(16777215, 90))
        self.PreciseAlignButton.setFont(font)
        self.PreciseAlignButton.setStyleSheet(u"")
        self.PreciseAlignButton.setCheckable(True)

        self.verticalLayout_20.addWidget(self.PreciseAlignButton)

        self.PickButton = QPushButton(self.ManualButtonsPage)
        self.ManualButtons.addButton(self.PickButton)
        self.PickButton.setObjectName(u"PickButton")
        sizePolicy.setHeightForWidth(self.PickButton.sizePolicy().hasHeightForWidth())
        self.PickButton.setSizePolicy(sizePolicy)
        self.PickButton.setMaximumSize(QSize(16777215, 90))
        self.PickButton.setFont(font)
        self.PickButton.setStyleSheet(u"")
        self.PickButton.setCheckable(True)

        self.verticalLayout_20.addWidget(self.PickButton)

        self.AssemblyButton = QPushButton(self.ManualButtonsPage)
        self.ManualButtons.addButton(self.AssemblyButton)
        self.AssemblyButton.setObjectName(u"AssemblyButton")
        sizePolicy.setHeightForWidth(self.AssemblyButton.sizePolicy().hasHeightForWidth())
        self.AssemblyButton.setSizePolicy(sizePolicy)
        self.AssemblyButton.setMaximumSize(QSize(16777215, 90))
        self.AssemblyButton.setFont(font)
        self.AssemblyButton.setStyleSheet(u"")
        self.AssemblyButton.setCheckable(True)

        self.verticalLayout_20.addWidget(self.AssemblyButton)

        self.ActionButtons.addWidget(self.ManualButtonsPage)

        self.verticalLayout_24.addWidget(self.ActionButtons)

        self.verticalLayoutAutoPauseAndStop = QVBoxLayout()
        self.verticalLayoutAutoPauseAndStop.setSpacing(6)
        self.verticalLayoutAutoPauseAndStop.setObjectName(u"verticalLayoutAutoPauseAndStop")
        self.verticalLayoutAutoPauseAndStop.setContentsMargins(-1, 0, -1, -1)
        self.AutoPauseButton = QPushButton(self.AutoAndManualWidget)
        self.AutoPauseButton.setObjectName(u"AutoPauseButton")
        sizePolicy.setHeightForWidth(self.AutoPauseButton.sizePolicy().hasHeightForWidth())
        self.AutoPauseButton.setSizePolicy(sizePolicy)
        self.AutoPauseButton.setMinimumSize(QSize(0, 80))
        self.AutoPauseButton.setFont(font)
        self.AutoPauseButton.setStyleSheet(u"")
        self.AutoPauseButton.setCheckable(True)

        self.verticalLayoutAutoPauseAndStop.addWidget(self.AutoPauseButton)

        self.AutoStopButton = QPushButton(self.AutoAndManualWidget)
        self.AutoStopButton.setObjectName(u"AutoStopButton")
        sizePolicy.setHeightForWidth(self.AutoStopButton.sizePolicy().hasHeightForWidth())
        self.AutoStopButton.setSizePolicy(sizePolicy)
        self.AutoStopButton.setMinimumSize(QSize(0, 80))
        self.AutoStopButton.setFont(font)
        self.AutoStopButton.setStyleSheet(u"\n"
"QPushButton#AutoStopButton:pressed {\n"
"    background-color: #990000;       /* darker red when pressed */\n"
"}")
        self.AutoStopButton.setCheckable(False)

        self.verticalLayoutAutoPauseAndStop.addWidget(self.AutoStopButton)


        self.verticalLayout_24.addLayout(self.verticalLayoutAutoPauseAndStop)

        self.Timeline = QFrame(self.MainPagePage)
        self.Timeline.setObjectName(u"Timeline")
        self.Timeline.setGeometry(QRect(0, 500, 701, 191))
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.Timeline.sizePolicy().hasHeightForWidth())
        self.Timeline.setSizePolicy(sizePolicy2)
        self.Timeline.setStyleSheet(u"background-color: #000000;")
        self.Timeline.setFrameShape(QFrame.Shape.StyledPanel)
        self.Timeline.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_3 = QHBoxLayout(self.Timeline)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.StartWidget = QWidget(self.Timeline)
        self.StartWidget.setObjectName(u"StartWidget")
        self.verticalLayout_2 = QVBoxLayout(self.StartWidget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.StartProcessText = QLabel(self.StartWidget)
        self.StartProcessText.setObjectName(u"StartProcessText")
        self.StartProcessText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_2.addWidget(self.StartProcessText)

        self.StartCircle = QLabel(self.StartWidget)
        self.StartCircle.setObjectName(u"StartCircle")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.StartCircle.sizePolicy().hasHeightForWidth())
        self.StartCircle.setSizePolicy(sizePolicy3)
        self.StartCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.StartCircle.setScaledContents(True)
        self.StartCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_2.addWidget(self.StartCircle, 0, Qt.AlignmentFlag.AlignHCenter)


        self.horizontalLayout_3.addWidget(self.StartWidget, 0, Qt.AlignmentFlag.AlignVCenter)

        self.ConnectWidget = QWidget(self.Timeline)
        self.ConnectWidget.setObjectName(u"ConnectWidget")
        self.verticalLayout = QVBoxLayout(self.ConnectWidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.ConnectProcessText = QLabel(self.ConnectWidget)
        self.ConnectProcessText.setObjectName(u"ConnectProcessText")
        self.ConnectProcessText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout.addWidget(self.ConnectProcessText)

        self.ConnectCircle = QLabel(self.ConnectWidget)
        self.ConnectCircle.setObjectName(u"ConnectCircle")
        sizePolicy3.setHeightForWidth(self.ConnectCircle.sizePolicy().hasHeightForWidth())
        self.ConnectCircle.setSizePolicy(sizePolicy3)
        self.ConnectCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.ConnectCircle.setScaledContents(True)
        self.ConnectCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout.addWidget(self.ConnectCircle, 0, Qt.AlignmentFlag.AlignHCenter)


        self.horizontalLayout_3.addWidget(self.ConnectWidget, 0, Qt.AlignmentFlag.AlignVCenter)

        self.INITWidget = QWidget(self.Timeline)
        self.INITWidget.setObjectName(u"INITWidget")
        self.verticalLayout_3 = QVBoxLayout(self.INITWidget)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.INITProcessText = QLabel(self.INITWidget)
        self.INITProcessText.setObjectName(u"INITProcessText")
        self.INITProcessText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_3.addWidget(self.INITProcessText)

        self.INITCircle = QLabel(self.INITWidget)
        self.INITCircle.setObjectName(u"INITCircle")
        sizePolicy3.setHeightForWidth(self.INITCircle.sizePolicy().hasHeightForWidth())
        self.INITCircle.setSizePolicy(sizePolicy3)
        self.INITCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.INITCircle.setScaledContents(True)
        self.INITCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_3.addWidget(self.INITCircle, 0, Qt.AlignmentFlag.AlignHCenter)


        self.horizontalLayout_3.addWidget(self.INITWidget, 0, Qt.AlignmentFlag.AlignVCenter)

        self.IdleWidget = QWidget(self.Timeline)
        self.IdleWidget.setObjectName(u"IdleWidget")
        self.verticalLayout_6 = QVBoxLayout(self.IdleWidget)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.IdleProcessText = QLabel(self.IdleWidget)
        self.IdleProcessText.setObjectName(u"IdleProcessText")
        self.IdleProcessText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_6.addWidget(self.IdleProcessText)

        self.IdleCircle = QLabel(self.IdleWidget)
        self.IdleCircle.setObjectName(u"IdleCircle")
        sizePolicy3.setHeightForWidth(self.IdleCircle.sizePolicy().hasHeightForWidth())
        self.IdleCircle.setSizePolicy(sizePolicy3)
        self.IdleCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.IdleCircle.setScaledContents(True)
        self.IdleCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_6.addWidget(self.IdleCircle, 0, Qt.AlignmentFlag.AlignHCenter)


        self.horizontalLayout_3.addWidget(self.IdleWidget, 0, Qt.AlignmentFlag.AlignVCenter)

        self.ManualAlignWidget = QWidget(self.Timeline)
        self.ManualAlignWidget.setObjectName(u"ManualAlignWidget")
        self.verticalLayout_7 = QVBoxLayout(self.ManualAlignWidget)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.RoughAlignProcessText = QLabel(self.ManualAlignWidget)
        self.RoughAlignProcessText.setObjectName(u"RoughAlignProcessText")
        self.RoughAlignProcessText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_7.addWidget(self.RoughAlignProcessText)

        self.RoughAlignCircle = QLabel(self.ManualAlignWidget)
        self.RoughAlignCircle.setObjectName(u"RoughAlignCircle")
        sizePolicy3.setHeightForWidth(self.RoughAlignCircle.sizePolicy().hasHeightForWidth())
        self.RoughAlignCircle.setSizePolicy(sizePolicy3)
        self.RoughAlignCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.RoughAlignCircle.setScaledContents(True)
        self.RoughAlignCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_7.addWidget(self.RoughAlignCircle, 0, Qt.AlignmentFlag.AlignHCenter)


        self.horizontalLayout_3.addWidget(self.ManualAlignWidget, 0, Qt.AlignmentFlag.AlignVCenter)

        self.AutoAlignWidget = QWidget(self.Timeline)
        self.AutoAlignWidget.setObjectName(u"AutoAlignWidget")
        self.verticalLayout_8 = QVBoxLayout(self.AutoAlignWidget)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.PreciseAlignProcessText = QLabel(self.AutoAlignWidget)
        self.PreciseAlignProcessText.setObjectName(u"PreciseAlignProcessText")
        self.PreciseAlignProcessText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_8.addWidget(self.PreciseAlignProcessText)

        self.PreciseAlignCircle = QLabel(self.AutoAlignWidget)
        self.PreciseAlignCircle.setObjectName(u"PreciseAlignCircle")
        sizePolicy3.setHeightForWidth(self.PreciseAlignCircle.sizePolicy().hasHeightForWidth())
        self.PreciseAlignCircle.setSizePolicy(sizePolicy3)
        self.PreciseAlignCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.PreciseAlignCircle.setScaledContents(True)
        self.PreciseAlignCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_8.addWidget(self.PreciseAlignCircle, 0, Qt.AlignmentFlag.AlignHCenter)


        self.horizontalLayout_3.addWidget(self.AutoAlignWidget, 0, Qt.AlignmentFlag.AlignVCenter)

        self.AutoPickWidget = QWidget(self.Timeline)
        self.AutoPickWidget.setObjectName(u"AutoPickWidget")
        self.verticalLayout_9 = QVBoxLayout(self.AutoPickWidget)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.PickProcessText = QLabel(self.AutoPickWidget)
        self.PickProcessText.setObjectName(u"PickProcessText")
        self.PickProcessText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_9.addWidget(self.PickProcessText)

        self.PickCircle = QLabel(self.AutoPickWidget)
        self.PickCircle.setObjectName(u"PickCircle")
        sizePolicy3.setHeightForWidth(self.PickCircle.sizePolicy().hasHeightForWidth())
        self.PickCircle.setSizePolicy(sizePolicy3)
        self.PickCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.PickCircle.setScaledContents(True)
        self.PickCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_9.addWidget(self.PickCircle, 0, Qt.AlignmentFlag.AlignHCenter)


        self.horizontalLayout_3.addWidget(self.AutoPickWidget, 0, Qt.AlignmentFlag.AlignVCenter)

        self.AssemblyWidget = QWidget(self.Timeline)
        self.AssemblyWidget.setObjectName(u"AssemblyWidget")
        self.verticalLayout_4 = QVBoxLayout(self.AssemblyWidget)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.AssemblyProcessText = QLabel(self.AssemblyWidget)
        self.AssemblyProcessText.setObjectName(u"AssemblyProcessText")
        self.AssemblyProcessText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_4.addWidget(self.AssemblyProcessText, 0, Qt.AlignmentFlag.AlignTop)

        self.AssemblyCircle = QLabel(self.AssemblyWidget)
        self.AssemblyCircle.setObjectName(u"AssemblyCircle")
        sizePolicy3.setHeightForWidth(self.AssemblyCircle.sizePolicy().hasHeightForWidth())
        self.AssemblyCircle.setSizePolicy(sizePolicy3)
        self.AssemblyCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.AssemblyCircle.setScaledContents(True)
        self.AssemblyCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_4.addWidget(self.AssemblyCircle, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.horizontalLayout_3.addWidget(self.AssemblyWidget, 0, Qt.AlignmentFlag.AlignVCenter)

        self.ParentStackedWidgetToChangeMenuOptions.addWidget(self.MainPagePage)
        self.ComponentControlPage = QWidget()
        self.ComponentControlPage.setObjectName(u"ComponentControlPage")
        self.ComponentControlStackedWidget = QStackedWidget(self.ComponentControlPage)
        self.ComponentControlStackedWidget.setObjectName(u"ComponentControlStackedWidget")
        self.ComponentControlStackedWidget.setGeometry(QRect(710, 10, 311, 681))
        self.ComponentControlStackedWidget.setFrameShape(QFrame.Shape.StyledPanel)
        self.ComponentControlStackedWidget.setFrameShadow(QFrame.Shadow.Raised)
        self.BeforeComponentControl = QWidget()
        self.BeforeComponentControl.setObjectName(u"BeforeComponentControl")
        self.verticalLayout_28 = QVBoxLayout(self.BeforeComponentControl)
        self.verticalLayout_28.setObjectName(u"verticalLayout_28")
        self.ChooseListOptionsWidget = QWidget(self.BeforeComponentControl)
        self.ChooseListOptionsWidget.setObjectName(u"ChooseListOptionsWidget")
        sizePolicy2.setHeightForWidth(self.ChooseListOptionsWidget.sizePolicy().hasHeightForWidth())
        self.ChooseListOptionsWidget.setSizePolicy(sizePolicy2)
        self.verticalLayout_27 = QVBoxLayout(self.ChooseListOptionsWidget)
        self.verticalLayout_27.setObjectName(u"verticalLayout_27")
        self.ChooseMotor = QPushButton(self.ChooseListOptionsWidget)
        self.ChooseMotor.setObjectName(u"ChooseMotor")
        sizePolicy.setHeightForWidth(self.ChooseMotor.sizePolicy().hasHeightForWidth())
        self.ChooseMotor.setSizePolicy(sizePolicy)
        self.ChooseMotor.setStyleSheet(u"")

        self.verticalLayout_27.addWidget(self.ChooseMotor)

        self.ChooseVision = QPushButton(self.ChooseListOptionsWidget)
        self.ChooseVision.setObjectName(u"ChooseVision")
        sizePolicy.setHeightForWidth(self.ChooseVision.sizePolicy().hasHeightForWidth())
        self.ChooseVision.setSizePolicy(sizePolicy)
        self.ChooseVision.setStyleSheet(u"background-color: #000000;")

        self.verticalLayout_27.addWidget(self.ChooseVision)

        self.ChooseClipper = QPushButton(self.ChooseListOptionsWidget)
        self.ChooseClipper.setObjectName(u"ChooseClipper")
        sizePolicy.setHeightForWidth(self.ChooseClipper.sizePolicy().hasHeightForWidth())
        self.ChooseClipper.setSizePolicy(sizePolicy)
        self.ChooseClipper.setStyleSheet(u"")

        self.verticalLayout_27.addWidget(self.ChooseClipper)

        self.ChooseForklift = QPushButton(self.ChooseListOptionsWidget)
        self.ChooseForklift.setObjectName(u"ChooseForklift")
        sizePolicy.setHeightForWidth(self.ChooseForklift.sizePolicy().hasHeightForWidth())
        self.ChooseForklift.setSizePolicy(sizePolicy)
        self.ChooseForklift.setStyleSheet(u"")

        self.verticalLayout_27.addWidget(self.ChooseForklift)

        self.ChooseDIDO = QPushButton(self.ChooseListOptionsWidget)
        self.ChooseDIDO.setObjectName(u"ChooseDIDO")
        sizePolicy.setHeightForWidth(self.ChooseDIDO.sizePolicy().hasHeightForWidth())
        self.ChooseDIDO.setSizePolicy(sizePolicy)
        self.ChooseDIDO.setStyleSheet(u"")

        self.verticalLayout_27.addWidget(self.ChooseDIDO)


        self.verticalLayout_28.addWidget(self.ChooseListOptionsWidget)

        self.ComponentControlStackedWidget.addWidget(self.BeforeComponentControl)
        self.ComponentControlMain = QWidget()
        self.ComponentControlMain.setObjectName(u"ComponentControlMain")
        self.ComponentControlOptionsWidget = QWidget(self.ComponentControlMain)
        self.ComponentControlOptionsWidget.setObjectName(u"ComponentControlOptionsWidget")
        self.ComponentControlOptionsWidget.setGeometry(QRect(10, 0, 291, 101))
        self.horizontalLayout_16 = QHBoxLayout(self.ComponentControlOptionsWidget)
        self.horizontalLayout_16.setSpacing(25)
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.horizontalLayout_16.setContentsMargins(9, -1, -1, -1)
        self.MotorStartedButton = QPushButton(self.ComponentControlOptionsWidget)
        self.MotorStartedButton.setObjectName(u"MotorStartedButton")
        sizePolicy2.setHeightForWidth(self.MotorStartedButton.sizePolicy().hasHeightForWidth())
        self.MotorStartedButton.setSizePolicy(sizePolicy2)
        self.MotorStartedButton.setStyleSheet(u"background-color: #0B76A0;")
        self.MotorStartedButton.setCheckable(False)
        self.MotorStartedButton.setChecked(False)

        self.horizontalLayout_16.addWidget(self.MotorStartedButton)

        self.HamburgerMenu = QPushButton(self.ComponentControlOptionsWidget)
        self.HamburgerMenu.setObjectName(u"HamburgerMenu")
        sizePolicy.setHeightForWidth(self.HamburgerMenu.sizePolicy().hasHeightForWidth())
        self.HamburgerMenu.setSizePolicy(sizePolicy)
        self.HamburgerMenu.setStyleSheet(u"")
        icon5 = QIcon()
        icon5.addFile(u":/icons/white/menu.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.HamburgerMenu.setIcon(icon5)
        self.HamburgerMenu.setIconSize(QSize(30, 30))

        self.horizontalLayout_16.addWidget(self.HamburgerMenu)

        self.ChangeComponentControlStackedWidget = QStackedWidget(self.ComponentControlMain)
        self.ChangeComponentControlStackedWidget.setObjectName(u"ChangeComponentControlStackedWidget")
        self.ChangeComponentControlStackedWidget.setGeometry(QRect(-10, 90, 321, 591))
        self.ChangeComponentControlStackedWidget.setStyleSheet(u"")
        self.MotorPage = QWidget()
        self.MotorPage.setObjectName(u"MotorPage")
        self.MotorStackedWidget = QStackedWidget(self.MotorPage)
        self.MotorStackedWidget.setObjectName(u"MotorStackedWidget")
        self.MotorStackedWidget.setGeometry(QRect(10, 30, 301, 561))
        self.MotorConfigPage = QWidget()
        self.MotorConfigPage.setObjectName(u"MotorConfigPage")
        self.ServoON = QPushButton(self.MotorConfigPage)
        self.ServoON.setObjectName(u"ServoON")
        self.ServoON.setEnabled(True)
        self.ServoON.setGeometry(QRect(70, 40, 161, 71))
        sizePolicy.setHeightForWidth(self.ServoON.sizePolicy().hasHeightForWidth())
        self.ServoON.setSizePolicy(sizePolicy)
        self.ServoON.setFont(font)
        self.ServoON.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.ServoON.setCheckable(False)
        self.ServoON.setChecked(False)
        self.HomeMotor = QPushButton(self.MotorConfigPage)
        self.HomeMotor.setObjectName(u"HomeMotor")
        self.HomeMotor.setEnabled(True)
        self.HomeMotor.setGeometry(QRect(90, 280, 121, 71))
        sizePolicy.setHeightForWidth(self.HomeMotor.sizePolicy().hasHeightForWidth())
        self.HomeMotor.setSizePolicy(sizePolicy)
        self.HomeMotor.setFont(font)
        self.HomeMotor.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.HomeMotor.setCheckable(False)
        self.ServoOFF = QPushButton(self.MotorConfigPage)
        self.ServoOFF.setObjectName(u"ServoOFF")
        self.ServoOFF.setEnabled(True)
        self.ServoOFF.setGeometry(QRect(70, 140, 161, 71))
        sizePolicy.setHeightForWidth(self.ServoOFF.sizePolicy().hasHeightForWidth())
        self.ServoOFF.setSizePolicy(sizePolicy)
        self.ServoOFF.setFont(font)
        self.ServoOFF.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.ServoOFF.setCheckable(False)
        self.ServoOFF.setChecked(False)
        self.MotorConfigNextButton = QPushButton(self.MotorConfigPage)
        self.MotorConfigNextButton.setObjectName(u"MotorConfigNextButton")
        self.MotorConfigNextButton.setEnabled(True)
        self.MotorConfigNextButton.setGeometry(QRect(210, 450, 91, 71))
        sizePolicy.setHeightForWidth(self.MotorConfigNextButton.sizePolicy().hasHeightForWidth())
        self.MotorConfigNextButton.setSizePolicy(sizePolicy)
        self.MotorConfigNextButton.setFont(font)
        self.MotorConfigNextButton.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.MotorConfigNextButton.setCheckable(False)
        self.MotorStackedWidget.addWidget(self.MotorConfigPage)
        self.JogPage = QWidget()
        self.JogPage.setObjectName(u"JogPage")
        self.YawPlusCP = QPushButton(self.JogPage)
        self.YawPlusCP.setObjectName(u"YawPlusCP")
        self.YawPlusCP.setGeometry(QRect(200, 20, 101, 91))
        sizePolicy4 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Maximum)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.YawPlusCP.sizePolicy().hasHeightForWidth())
        self.YawPlusCP.setSizePolicy(sizePolicy4)
        self.YawPlusCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon6 = QIcon()
        icon6.addFile(u":/controlArrows/cartesian/yawPlus.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.YawPlusCP.setIcon(icon6)
        self.YawPlusCP.setIconSize(QSize(80, 80))
        self.ControlUpCP = QPushButton(self.JogPage)
        self.ControlUpCP.setObjectName(u"ControlUpCP")
        self.ControlUpCP.setGeometry(QRect(100, 50, 101, 91))
        sizePolicy4.setHeightForWidth(self.ControlUpCP.sizePolicy().hasHeightForWidth())
        self.ControlUpCP.setSizePolicy(sizePolicy4)
        self.ControlUpCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon7 = QIcon()
        icon7.addFile(u":/controlArrows/cartesian/up.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ControlUpCP.setIcon(icon7)
        self.ControlUpCP.setIconSize(QSize(80, 80))
        self.ControlLeftCP = QPushButton(self.JogPage)
        self.ControlLeftCP.setObjectName(u"ControlLeftCP")
        self.ControlLeftCP.setGeometry(QRect(0, 140, 101, 91))
        self.ControlLeftCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon8 = QIcon()
        icon8.addFile(u":/controlArrows/cartesian/left.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ControlLeftCP.setIcon(icon8)
        self.ControlLeftCP.setIconSize(QSize(80, 80))
        self.ControlDownCP = QPushButton(self.JogPage)
        self.ControlDownCP.setObjectName(u"ControlDownCP")
        self.ControlDownCP.setGeometry(QRect(100, 230, 101, 91))
        self.ControlDownCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon9 = QIcon()
        icon9.addFile(u":/controlArrows/cartesian/down.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ControlDownCP.setIcon(icon9)
        self.ControlDownCP.setIconSize(QSize(80, 80))
        self.ControlRightCP = QPushButton(self.JogPage)
        self.ControlRightCP.setObjectName(u"ControlRightCP")
        self.ControlRightCP.setGeometry(QRect(200, 140, 101, 91))
        self.ControlRightCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon10 = QIcon()
        icon10.addFile(u":/controlArrows/cartesian/right.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ControlRightCP.setIcon(icon10)
        self.ControlRightCP.setIconSize(QSize(80, 80))
        self.YawMinusCP = QPushButton(self.JogPage)
        self.YawMinusCP.setObjectName(u"YawMinusCP")
        self.YawMinusCP.setGeometry(QRect(0, 270, 101, 91))
        sizePolicy4.setHeightForWidth(self.YawMinusCP.sizePolicy().hasHeightForWidth())
        self.YawMinusCP.setSizePolicy(sizePolicy4)
        self.YawMinusCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon11 = QIcon()
        icon11.addFile(u":/controlArrows/cartesian/yawMinus.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.YawMinusCP.setIcon(icon11)
        self.YawMinusCP.setIconSize(QSize(80, 80))
        self.YawMinusCP.setCheckable(False)
        self.ClipperAndResetWidget_3 = QWidget(self.JogPage)
        self.ClipperAndResetWidget_3.setObjectName(u"ClipperAndResetWidget_3")
        self.ClipperAndResetWidget_3.setGeometry(QRect(-10, 410, 311, 141))
        self.verticalLayout_10 = QVBoxLayout(self.ClipperAndResetWidget_3)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.PauseMotor = QPushButton(self.ClipperAndResetWidget_3)
        self.PauseMotor.setObjectName(u"PauseMotor")
        self.PauseMotor.setEnabled(True)
        sizePolicy.setHeightForWidth(self.PauseMotor.sizePolicy().hasHeightForWidth())
        self.PauseMotor.setSizePolicy(sizePolicy)
        self.PauseMotor.setFont(font)
        self.PauseMotor.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.PauseMotor.setCheckable(True)

        self.verticalLayout_10.addWidget(self.PauseMotor)

        self.StopMotor = QPushButton(self.ClipperAndResetWidget_3)
        self.StopMotor.setObjectName(u"StopMotor")
        sizePolicy.setHeightForWidth(self.StopMotor.sizePolicy().hasHeightForWidth())
        self.StopMotor.setSizePolicy(sizePolicy)
        self.StopMotor.setFont(font)
        self.StopMotor.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #0f0f0f, stop:1 #000000\n"
"    );\n"
"    border-style: inset;\n"
"	background-color: #0B76A0;\n"
"}\n"
"\n"
"")

        self.verticalLayout_10.addWidget(self.StopMotor)

        self.MotorJogNextButton = QPushButton(self.JogPage)
        self.MotorJogNextButton.setObjectName(u"MotorJogNextButton")
        self.MotorJogNextButton.setEnabled(True)
        self.MotorJogNextButton.setGeometry(QRect(210, 330, 91, 71))
        sizePolicy.setHeightForWidth(self.MotorJogNextButton.sizePolicy().hasHeightForWidth())
        self.MotorJogNextButton.setSizePolicy(sizePolicy)
        self.MotorJogNextButton.setFont(font)
        self.MotorJogNextButton.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.MotorJogNextButton.setCheckable(False)
        self.MotorChooseDistance = QPushButton(self.JogPage)
        self.MotorChooseDistance.setObjectName(u"MotorChooseDistance")
        self.MotorChooseDistance.setEnabled(True)
        self.MotorChooseDistance.setGeometry(QRect(0, 0, 211, 51))
        sizePolicy.setHeightForWidth(self.MotorChooseDistance.sizePolicy().hasHeightForWidth())
        self.MotorChooseDistance.setSizePolicy(sizePolicy)
        self.MotorChooseDistance.setFont(font)
        self.MotorChooseDistance.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.MotorChooseDistance.setCheckable(False)
        self.MotorStackedWidget.addWidget(self.JogPage)
        self.YAxisPage = QWidget()
        self.YAxisPage.setObjectName(u"YAxisPage")
        self.MotorYAxisNextButton = QPushButton(self.YAxisPage)
        self.MotorYAxisNextButton.setObjectName(u"MotorYAxisNextButton")
        self.MotorYAxisNextButton.setEnabled(True)
        self.MotorYAxisNextButton.setGeometry(QRect(210, 470, 91, 71))
        sizePolicy.setHeightForWidth(self.MotorYAxisNextButton.sizePolicy().hasHeightForWidth())
        self.MotorYAxisNextButton.setSizePolicy(sizePolicy)
        self.MotorYAxisNextButton.setFont(font)
        self.MotorYAxisNextButton.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.MotorYAxisNextButton.setCheckable(False)
        self.HomeY = QPushButton(self.YAxisPage)
        self.HomeY.setObjectName(u"HomeY")
        self.HomeY.setEnabled(True)
        self.HomeY.setGeometry(QRect(150, 20, 151, 71))
        sizePolicy.setHeightForWidth(self.HomeY.sizePolicy().hasHeightForWidth())
        self.HomeY.setSizePolicy(sizePolicy)
        self.HomeY.setFont(font)
        self.HomeY.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.HomeY.setCheckable(False)
        self.ReadyY = QPushButton(self.YAxisPage)
        self.ReadyY.setObjectName(u"ReadyY")
        self.ReadyY.setEnabled(True)
        self.ReadyY.setGeometry(QRect(150, 100, 151, 71))
        sizePolicy.setHeightForWidth(self.ReadyY.sizePolicy().hasHeightForWidth())
        self.ReadyY.setSizePolicy(sizePolicy)
        self.ReadyY.setFont(font)
        self.ReadyY.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.ReadyY.setCheckable(False)
        self.AssemblyY = QPushButton(self.YAxisPage)
        self.AssemblyY.setObjectName(u"AssemblyY")
        self.AssemblyY.setEnabled(True)
        self.AssemblyY.setGeometry(QRect(150, 180, 151, 71))
        sizePolicy.setHeightForWidth(self.AssemblyY.sizePolicy().hasHeightForWidth())
        self.AssemblyY.setSizePolicy(sizePolicy)
        self.AssemblyY.setFont(font)
        self.AssemblyY.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.AssemblyY.setCheckable(False)
        self.SendYCommand = QPushButton(self.YAxisPage)
        self.SendYCommand.setObjectName(u"SendYCommand")
        self.SendYCommand.setEnabled(True)
        self.SendYCommand.setGeometry(QRect(30, 330, 231, 71))
        sizePolicy.setHeightForWidth(self.SendYCommand.sizePolicy().hasHeightForWidth())
        self.SendYCommand.setSizePolicy(sizePolicy)
        self.SendYCommand.setFont(font)
        self.SendYCommand.setStyleSheet(u"QPushButton {\n"
"    padding: 6px 12px;\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #444;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #2a2a2a, stop:1 #111111\n"
"    );\n"
"}\n"
"\n"
"QPushButton:checked {\n"
"    background-color: #0B76A0;\n"
"    border: 1px solid #0B76A0;\n"
"}\n"
"\n"
"")
        self.SendYCommand.setCheckable(False)
        self.InputMotorDistance = QLineEdit(self.YAxisPage)
        self.InputMotorDistance.setObjectName(u"InputMotorDistance")
        self.InputMotorDistance.setGeometry(QRect(10, 50, 121, 41))
        self.InputMotorDistance.setStyleSheet(u"background-color: #FFFFFF;")
        self.InputMotorSpeed = QLineEdit(self.YAxisPage)
        self.InputMotorSpeed.setObjectName(u"InputMotorSpeed")
        self.InputMotorSpeed.setGeometry(QRect(10, 180, 121, 41))
        self.InputMotorSpeed.setStyleSheet(u"background-color: #FFFFFF;")
        self.label_6 = QLabel(self.YAxisPage)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(40, 20, 67, 17))
        self.label_10 = QLabel(self.YAxisPage)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setGeometry(QRect(40, 150, 67, 17))
        self.MotorStackedWidget.addWidget(self.YAxisPage)
        self.ChangeComponentControlStackedWidget.addWidget(self.MotorPage)
        self.VisionPage = QWidget()
        self.VisionPage.setObjectName(u"VisionPage")
        self.VisionOne = QPushButton(self.VisionPage)
        self.VisionButtonGroup = QButtonGroup(MainWindow)
        self.VisionButtonGroup.setObjectName(u"VisionButtonGroup")
        self.VisionButtonGroup.setExclusive(False)
        self.VisionButtonGroup.addButton(self.VisionOne)
        self.VisionOne.setObjectName(u"VisionOne")
        self.VisionOne.setGeometry(QRect(50, 23, 233, 79))
        sizePolicy.setHeightForWidth(self.VisionOne.sizePolicy().hasHeightForWidth())
        self.VisionOne.setSizePolicy(sizePolicy)
        self.VisionOne.setStyleSheet(u"")
        self.VisionOne.setCheckable(True)
        self.VisionTwo = QPushButton(self.VisionPage)
        self.VisionButtonGroup.addButton(self.VisionTwo)
        self.VisionTwo.setObjectName(u"VisionTwo")
        self.VisionTwo.setGeometry(QRect(50, 120, 233, 79))
        sizePolicy.setHeightForWidth(self.VisionTwo.sizePolicy().hasHeightForWidth())
        self.VisionTwo.setSizePolicy(sizePolicy)
        self.VisionTwo.setStyleSheet(u"")
        self.VisionTwo.setCheckable(True)
        self.VisionThree = QPushButton(self.VisionPage)
        self.VisionButtonGroup.addButton(self.VisionThree)
        self.VisionThree.setObjectName(u"VisionThree")
        self.VisionThree.setGeometry(QRect(50, 210, 233, 79))
        sizePolicy.setHeightForWidth(self.VisionThree.sizePolicy().hasHeightForWidth())
        self.VisionThree.setSizePolicy(sizePolicy)
        self.VisionThree.setStyleSheet(u"")
        self.VisionThree.setCheckable(True)
        self.VisionSendButton = QPushButton(self.VisionPage)
        self.VisionSendButton.setObjectName(u"VisionSendButton")
        self.VisionSendButton.setGeometry(QRect(50, 510, 233, 79))
        sizePolicy.setHeightForWidth(self.VisionSendButton.sizePolicy().hasHeightForWidth())
        self.VisionSendButton.setSizePolicy(sizePolicy)
        self.VisionSendButton.setStyleSheet(u"")
        self.VisionSendButton.setCheckable(False)
        self.VisionPoseWidget = QWidget(self.VisionPage)
        self.VisionPoseWidget.setObjectName(u"VisionPoseWidget")
        self.VisionPoseWidget.setGeometry(QRect(20, 300, 281, 201))
        self.VisionPoseWidget.setStyleSheet(u"background-color: #000000;")
        self.gridLayout = QGridLayout(self.VisionPoseWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.yVisionLabel = QLabel(self.VisionPoseWidget)
        self.yVisionLabel.setObjectName(u"yVisionLabel")

        self.gridLayout.addWidget(self.yVisionLabel, 1, 1, 1, 1)

        self.yVision = QPushButton(self.VisionPoseWidget)
        self.yVision.setObjectName(u"yVision")
        self.yVision.setEnabled(False)
        sizePolicy.setHeightForWidth(self.yVision.sizePolicy().hasHeightForWidth())
        self.yVision.setSizePolicy(sizePolicy)
        self.yVision.setStyleSheet(u"")
        self.yVision.setCheckable(False)

        self.gridLayout.addWidget(self.yVision, 1, 0, 1, 1)

        self.xVisionLabel = QLabel(self.VisionPoseWidget)
        self.xVisionLabel.setObjectName(u"xVisionLabel")

        self.gridLayout.addWidget(self.xVisionLabel, 0, 1, 1, 1)

        self.Yaw = QPushButton(self.VisionPoseWidget)
        self.Yaw.setObjectName(u"Yaw")
        self.Yaw.setEnabled(False)
        sizePolicy.setHeightForWidth(self.Yaw.sizePolicy().hasHeightForWidth())
        self.Yaw.setSizePolicy(sizePolicy)
        self.Yaw.setStyleSheet(u"")
        self.Yaw.setCheckable(False)

        self.gridLayout.addWidget(self.Yaw, 2, 0, 1, 1)

        self.xVision = QPushButton(self.VisionPoseWidget)
        self.xVision.setObjectName(u"xVision")
        self.xVision.setEnabled(False)
        sizePolicy.setHeightForWidth(self.xVision.sizePolicy().hasHeightForWidth())
        self.xVision.setSizePolicy(sizePolicy)
        self.xVision.setStyleSheet(u"")
        self.xVision.setCheckable(False)

        self.gridLayout.addWidget(self.xVision, 0, 0, 1, 1)

        self.zVision = QPushButton(self.VisionPoseWidget)
        self.zVision.setObjectName(u"zVision")
        self.zVision.setEnabled(False)
        sizePolicy.setHeightForWidth(self.zVision.sizePolicy().hasHeightForWidth())
        self.zVision.setSizePolicy(sizePolicy)
        self.zVision.setStyleSheet(u"")
        self.zVision.setCheckable(False)

        self.gridLayout.addWidget(self.zVision, 4, 0, 1, 1)

        self.YawVisionLabel = QLabel(self.VisionPoseWidget)
        self.YawVisionLabel.setObjectName(u"YawVisionLabel")

        self.gridLayout.addWidget(self.YawVisionLabel, 2, 1, 1, 1)

        self.zVisionLabel = QLabel(self.VisionPoseWidget)
        self.zVisionLabel.setObjectName(u"zVisionLabel")

        self.gridLayout.addWidget(self.zVisionLabel, 4, 1, 1, 1)

        self.ChangeComponentControlStackedWidget.addWidget(self.VisionPage)
        self.ClipperPage = QWidget()
        self.ClipperPage.setObjectName(u"ClipperPage")
        self.OpenClipper = QPushButton(self.ClipperPage)
        self.OpenClipper.setObjectName(u"OpenClipper")
        self.OpenClipper.setGeometry(QRect(40, 70, 111, 79))
        sizePolicy.setHeightForWidth(self.OpenClipper.sizePolicy().hasHeightForWidth())
        self.OpenClipper.setSizePolicy(sizePolicy)
        self.OpenClipper.setStyleSheet(u"")
        self.OpenClipper.setCheckable(False)
        self.CloseClipper = QPushButton(self.ClipperPage)
        self.CloseClipper.setObjectName(u"CloseClipper")
        self.CloseClipper.setGeometry(QRect(170, 70, 111, 79))
        sizePolicy.setHeightForWidth(self.CloseClipper.sizePolicy().hasHeightForWidth())
        self.CloseClipper.setSizePolicy(sizePolicy)
        self.CloseClipper.setStyleSheet(u"")
        self.CloseClipper.setCheckable(False)
        self.StopClipper = QPushButton(self.ClipperPage)
        self.StopClipper.setObjectName(u"StopClipper")
        self.StopClipper.setGeometry(QRect(40, 190, 111, 79))
        sizePolicy.setHeightForWidth(self.StopClipper.sizePolicy().hasHeightForWidth())
        self.StopClipper.setSizePolicy(sizePolicy)
        self.StopClipper.setStyleSheet(u"")
        self.StopClipper.setCheckable(False)
        self.ResetClipper = QPushButton(self.ClipperPage)
        self.ResetClipper.setObjectName(u"ResetClipper")
        self.ResetClipper.setGeometry(QRect(170, 190, 111, 79))
        sizePolicy.setHeightForWidth(self.ResetClipper.sizePolicy().hasHeightForWidth())
        self.ResetClipper.setSizePolicy(sizePolicy)
        self.ResetClipper.setStyleSheet(u"")
        self.ResetClipper.setCheckable(False)
        self.ChangeComponentControlStackedWidget.addWidget(self.ClipperPage)
        self.ForkliftPage = QWidget()
        self.ForkliftPage.setObjectName(u"ForkliftPage")
        self.LiftUp = QPushButton(self.ForkliftPage)
        self.buttonGroup_2 = QButtonGroup(MainWindow)
        self.buttonGroup_2.setObjectName(u"buttonGroup_2")
        self.buttonGroup_2.addButton(self.LiftUp)
        self.LiftUp.setObjectName(u"LiftUp")
        self.LiftUp.setGeometry(QRect(30, 480, 101, 91))
        sizePolicy4.setHeightForWidth(self.LiftUp.sizePolicy().hasHeightForWidth())
        self.LiftUp.setSizePolicy(sizePolicy4)
        self.LiftUp.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon12 = QIcon()
        icon12.addFile(u":/controlArrows/lift.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.LiftUp.setIcon(icon12)
        self.LiftUp.setIconSize(QSize(80, 80))
        self.LiftUp.setCheckable(False)
        self.LowerDown = QPushButton(self.ForkliftPage)
        self.buttonGroup_2.addButton(self.LowerDown)
        self.LowerDown.setObjectName(u"LowerDown")
        self.LowerDown.setGeometry(QRect(190, 480, 101, 91))
        sizePolicy4.setHeightForWidth(self.LowerDown.sizePolicy().hasHeightForWidth())
        self.LowerDown.setSizePolicy(sizePolicy4)
        self.LowerDown.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon13 = QIcon()
        icon13.addFile(u":/controlArrows/lower.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.LowerDown.setIcon(icon13)
        self.LowerDown.setIconSize(QSize(80, 80))
        self.LowerDown.setCheckable(False)
        self.FastForktLiftButton = QPushButton(self.ForkliftPage)
        self.buttonGroup = QButtonGroup(MainWindow)
        self.buttonGroup.setObjectName(u"buttonGroup")
        self.buttonGroup.addButton(self.FastForktLiftButton)
        self.FastForktLiftButton.setObjectName(u"FastForktLiftButton")
        self.FastForktLiftButton.setGeometry(QRect(20, 170, 88, 71))
        self.FastForktLiftButton.setCheckable(True)
        self.SlowLiftButton = QPushButton(self.ForkliftPage)
        self.buttonGroup.addButton(self.SlowLiftButton)
        self.SlowLiftButton.setObjectName(u"SlowLiftButton")
        self.SlowLiftButton.setGeometry(QRect(220, 170, 88, 71))
        self.SlowLiftButton.setCheckable(True)
        self.StopForkliftButton = QPushButton(self.ForkliftPage)
        self.StopForkliftButton.setObjectName(u"StopForkliftButton")
        self.StopForkliftButton.setGeometry(QRect(40, 30, 251, 81))
        self.StopForkliftButton.setCheckable(False)
        self.label_4 = QLabel(self.ForkliftPage)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(130, 140, 67, 17))
        self.label_4.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.SendForkliftCommand = QPushButton(self.ForkliftPage)
        self.SendForkliftCommand.setObjectName(u"SendForkliftCommand")
        self.SendForkliftCommand.setGeometry(QRect(40, 360, 251, 81))
        self.SendForkliftCommand.setCheckable(False)
        self.MediumLiftButton = QPushButton(self.ForkliftPage)
        self.buttonGroup.addButton(self.MediumLiftButton)
        self.MediumLiftButton.setObjectName(u"MediumLiftButton")
        self.MediumLiftButton.setGeometry(QRect(120, 170, 88, 71))
        self.MediumLiftButton.setCheckable(True)
        self.label_7 = QLabel(self.ForkliftPage)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(100, 270, 141, 20))
        self.label_7.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_2 = QLabel(self.ForkliftPage)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(50, 570, 67, 17))
        self.label_2.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_9 = QLabel(self.ForkliftPage)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setGeometry(QRect(210, 570, 67, 17))
        self.label_9.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.InputDistance = QLineEdit(self.ForkliftPage)
        self.InputDistance.setObjectName(u"InputDistance")
        self.InputDistance.setGeometry(QRect(70, 300, 191, 41))
        self.InputDistance.setStyleSheet(u"background-color: #FFFFFF;")
        self.ChangeComponentControlStackedWidget.addWidget(self.ForkliftPage)
        self.DIDOPage = QWidget()
        self.DIDOPage.setObjectName(u"DIDOPage")
        self.ChangeComponentControlStackedWidget.addWidget(self.DIDOPage)
        self.ListOptionsWidget = QWidget(self.ComponentControlMain)
        self.ListOptionsWidget.setObjectName(u"ListOptionsWidget")
        self.ListOptionsWidget.setGeometry(QRect(10, 100, 291, 571))
        sizePolicy2.setHeightForWidth(self.ListOptionsWidget.sizePolicy().hasHeightForWidth())
        self.ListOptionsWidget.setSizePolicy(sizePolicy2)
        self.verticalLayout_5 = QVBoxLayout(self.ListOptionsWidget)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.MotorOption = QPushButton(self.ListOptionsWidget)
        self.MotorOption.setObjectName(u"MotorOption")
        sizePolicy.setHeightForWidth(self.MotorOption.sizePolicy().hasHeightForWidth())
        self.MotorOption.setSizePolicy(sizePolicy)
        self.MotorOption.setStyleSheet(u"")

        self.verticalLayout_5.addWidget(self.MotorOption)

        self.VisionOption = QPushButton(self.ListOptionsWidget)
        self.VisionOption.setObjectName(u"VisionOption")
        sizePolicy.setHeightForWidth(self.VisionOption.sizePolicy().hasHeightForWidth())
        self.VisionOption.setSizePolicy(sizePolicy)
        self.VisionOption.setStyleSheet(u"background-color: #000000;")

        self.verticalLayout_5.addWidget(self.VisionOption)

        self.ClipperOption = QPushButton(self.ListOptionsWidget)
        self.ClipperOption.setObjectName(u"ClipperOption")
        sizePolicy.setHeightForWidth(self.ClipperOption.sizePolicy().hasHeightForWidth())
        self.ClipperOption.setSizePolicy(sizePolicy)
        self.ClipperOption.setStyleSheet(u"")

        self.verticalLayout_5.addWidget(self.ClipperOption)

        self.ForkliftOption = QPushButton(self.ListOptionsWidget)
        self.ForkliftOption.setObjectName(u"ForkliftOption")
        sizePolicy.setHeightForWidth(self.ForkliftOption.sizePolicy().hasHeightForWidth())
        self.ForkliftOption.setSizePolicy(sizePolicy)
        self.ForkliftOption.setStyleSheet(u"")

        self.verticalLayout_5.addWidget(self.ForkliftOption)

        self.DIDOOption = QPushButton(self.ListOptionsWidget)
        self.DIDOOption.setObjectName(u"DIDOOption")
        sizePolicy.setHeightForWidth(self.DIDOOption.sizePolicy().hasHeightForWidth())
        self.DIDOOption.setSizePolicy(sizePolicy)
        self.DIDOOption.setStyleSheet(u"")

        self.verticalLayout_5.addWidget(self.DIDOOption)

        self.ComponentControlStackedWidget.addWidget(self.ComponentControlMain)
        self.MiddleStackedWidget = QStackedWidget(self.ComponentControlPage)
        self.MiddleStackedWidget.setObjectName(u"MiddleStackedWidget")
        self.MiddleStackedWidget.setGeometry(QRect(0, 0, 701, 701))
        self.VisionAndInfoPage = QWidget()
        self.VisionAndInfoPage.setObjectName(u"VisionAndInfoPage")
        self.CameraWidgetInComponentControl = QWidget(self.VisionAndInfoPage)
        self.CameraWidgetInComponentControl.setObjectName(u"CameraWidgetInComponentControl")
        self.CameraWidgetInComponentControl.setGeometry(QRect(0, 0, 701, 491))
        self.CameraWidgetInComponentControl.setStyleSheet(u"background-color: #000000;")
        self.horizontalLayout_15 = QHBoxLayout(self.CameraWidgetInComponentControl)
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.VisionTextInComponentControl = QLabel(self.CameraWidgetInComponentControl)
        self.VisionTextInComponentControl.setObjectName(u"VisionTextInComponentControl")
        self.VisionTextInComponentControl.setStyleSheet(u"color:white;")
        self.VisionTextInComponentControl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_15.addWidget(self.VisionTextInComponentControl)

        self.Info_2 = QFrame(self.VisionAndInfoPage)
        self.Info_2.setObjectName(u"Info_2")
        self.Info_2.setGeometry(QRect(0, 500, 701, 191))
        self.Info_2.setStyleSheet(u" background-color: #000000; color: white;")
        self.Info_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.Info_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_19 = QHBoxLayout(self.Info_2)
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.horizontalLayout_19.setContentsMargins(-1, 9, -1, 9)
        self.CartesianPoseWidget_2 = QWidget(self.Info_2)
        self.CartesianPoseWidget_2.setObjectName(u"CartesianPoseWidget_2")
        sizePolicy2.setHeightForWidth(self.CartesianPoseWidget_2.sizePolicy().hasHeightForWidth())
        self.CartesianPoseWidget_2.setSizePolicy(sizePolicy2)
        self.CartesianPoseWidget_2.setMinimumSize(QSize(0, 0))
        self.CartesianPoseWidget_2.setStyleSheet(u"QWidget#CartesianPoseWidget_2 {\n"
"	border: 2px solid #FFFFFF;  /* white solid border */\n"
"	border-radius: 6px;\n"
"}")
        self.gridLayout_4 = QGridLayout(self.CartesianPoseWidget_2)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.gridLayout_4.setContentsMargins(-1, 6, -1, -1)
        self.CartesianPoseText_2 = QLabel(self.CartesianPoseWidget_2)
        self.CartesianPoseText_2.setObjectName(u"CartesianPoseText_2")
        font2 = QFont()
        font2.setPointSize(11)
        font2.setBold(False)
        self.CartesianPoseText_2.setFont(font2)
        self.CartesianPoseText_2.setStyleSheet(u"QLabel#CartesianPoseText_2 {\n"
"    border-bottom: 1px solid white;\n"
"}")
        self.CartesianPoseText_2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_4.addWidget(self.CartesianPoseText_2, 0, 0, 1, 2, Qt.AlignmentFlag.AlignTop)

        self.zPosFrame_2 = QFrame(self.CartesianPoseWidget_2)
        self.zPosFrame_2.setObjectName(u"zPosFrame_2")
        self.zPosFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.zPosFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_16 = QVBoxLayout(self.zPosFrame_2)
        self.verticalLayout_16.setObjectName(u"verticalLayout_16")
        self.verticalLayout_16.setContentsMargins(-1, -1, -1, 14)
        self.z_2 = QLabel(self.zPosFrame_2)
        self.z_2.setObjectName(u"z_2")
        self.z_2.setFont(font2)

        self.verticalLayout_16.addWidget(self.z_2)

        self.zPose = QLabel(self.zPosFrame_2)
        self.zPose.setObjectName(u"zPose")

        self.verticalLayout_16.addWidget(self.zPose)


        self.gridLayout_4.addWidget(self.zPosFrame_2, 2, 0, 1, 1)

        self.yPosFrame_2 = QFrame(self.CartesianPoseWidget_2)
        self.yPosFrame_2.setObjectName(u"yPosFrame_2")
        self.yPosFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.yPosFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_14 = QVBoxLayout(self.yPosFrame_2)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.verticalLayout_14.setContentsMargins(-1, -1, -1, 3)
        self.y_2 = QLabel(self.yPosFrame_2)
        self.y_2.setObjectName(u"y_2")
        self.y_2.setFont(font2)

        self.verticalLayout_14.addWidget(self.y_2)

        self.yPos = QLabel(self.yPosFrame_2)
        self.yPos.setObjectName(u"yPos")

        self.verticalLayout_14.addWidget(self.yPos)


        self.gridLayout_4.addWidget(self.yPosFrame_2, 1, 1, 1, 1)

        self.yawPosFrame_2 = QFrame(self.CartesianPoseWidget_2)
        self.yawPosFrame_2.setObjectName(u"yawPosFrame_2")
        self.yawPosFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.yawPosFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_17 = QVBoxLayout(self.yawPosFrame_2)
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.verticalLayout_17.setContentsMargins(-1, -1, -1, 14)
        self.yaw_2 = QLabel(self.yawPosFrame_2)
        self.yaw_2.setObjectName(u"yaw_2")
        self.yaw_2.setFont(font2)

        self.verticalLayout_17.addWidget(self.yaw_2)

        self.yawPos = QLabel(self.yawPosFrame_2)
        self.yawPos.setObjectName(u"yawPos")

        self.verticalLayout_17.addWidget(self.yawPos)


        self.gridLayout_4.addWidget(self.yawPosFrame_2, 2, 1, 1, 1)

        self.xPosFrame_2 = QFrame(self.CartesianPoseWidget_2)
        self.xPosFrame_2.setObjectName(u"xPosFrame_2")
        font3 = QFont()
        font3.setPointSize(9)
        self.xPosFrame_2.setFont(font3)
        self.xPosFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.xPosFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_13 = QVBoxLayout(self.xPosFrame_2)
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.verticalLayout_13.setContentsMargins(9, -1, -1, 3)
        self.x_2 = QLabel(self.xPosFrame_2)
        self.x_2.setObjectName(u"x_2")
        self.x_2.setFont(font2)

        self.verticalLayout_13.addWidget(self.x_2)

        self.xPos = QLabel(self.xPosFrame_2)
        self.xPos.setObjectName(u"xPos")

        self.verticalLayout_13.addWidget(self.xPos)


        self.gridLayout_4.addWidget(self.xPosFrame_2, 1, 0, 1, 1)


        self.horizontalLayout_19.addWidget(self.CartesianPoseWidget_2)

        self.MotorInfoWidget_2 = QWidget(self.Info_2)
        self.MotorInfoWidget_2.setObjectName(u"MotorInfoWidget_2")
        self.MotorInfoWidget_2.setStyleSheet(u"QWidget#MotorInfoWidget_2 {\n"
"	border: 2px solid #FFFFFF;  /* white solid border */\n"
"	border-radius: 6px;\n"
"}")
        self.gridLayout_5 = QGridLayout(self.MotorInfoWidget_2)
        self.gridLayout_5.setSpacing(6)
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.gridLayout_5.setContentsMargins(-1, 6, 9, 9)
        self.MotorInfoText_2 = QLabel(self.MotorInfoWidget_2)
        self.MotorInfoText_2.setObjectName(u"MotorInfoText_2")
        font4 = QFont()
        font4.setPointSize(11)
        self.MotorInfoText_2.setFont(font4)
        self.MotorInfoText_2.setStyleSheet(u"QLabel#MotorInfoText_2 {\n"
"    border-bottom: 1px solid white;\n"
"}")
        self.MotorInfoText_2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_5.addWidget(self.MotorInfoText_2, 0, 0, 1, 1, Qt.AlignmentFlag.AlignTop)

        self.PositionFrame_2 = QFrame(self.MotorInfoWidget_2)
        self.PositionFrame_2.setObjectName(u"PositionFrame_2")
        self.PositionFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.PositionFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_15 = QVBoxLayout(self.PositionFrame_2)
        self.verticalLayout_15.setSpacing(12)
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.verticalLayout_15.setContentsMargins(-1, 9, -1, 9)
        self.Position_2 = QLabel(self.PositionFrame_2)
        self.Position_2.setObjectName(u"Position_2")
        self.Position_2.setFont(font2)

        self.verticalLayout_15.addWidget(self.Position_2)

        self.MotorM1 = QLabel(self.PositionFrame_2)
        self.MotorM1.setObjectName(u"MotorM1")

        self.verticalLayout_15.addWidget(self.MotorM1)

        self.MotorM2 = QLabel(self.PositionFrame_2)
        self.MotorM2.setObjectName(u"MotorM2")

        self.verticalLayout_15.addWidget(self.MotorM2)

        self.MotorM3 = QLabel(self.PositionFrame_2)
        self.MotorM3.setObjectName(u"MotorM3")

        self.verticalLayout_15.addWidget(self.MotorM3)


        self.gridLayout_5.addWidget(self.PositionFrame_2, 1, 0, 1, 1, Qt.AlignmentFlag.AlignTop)


        self.horizontalLayout_19.addWidget(self.MotorInfoWidget_2)

        self.LaserInfoWidget = QWidget(self.Info_2)
        self.LaserInfoWidget.setObjectName(u"LaserInfoWidget")
        self.LaserInfoWidget.setStyleSheet(u"QWidget#LaserInfoWidget {\n"
"	border: 2px solid #FFFFFF;  /* white solid border */\n"
"	border-radius: 6px;\n"
"}")
        self.gridLayout_6 = QGridLayout(self.LaserInfoWidget)
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.gridLayout_6.setContentsMargins(-1, 6, -1, -1)
        self.LaserInfoText_2 = QLabel(self.LaserInfoWidget)
        self.LaserInfoText_2.setObjectName(u"LaserInfoText_2")
        font5 = QFont()
        font5.setPointSize(11)
        font5.setItalic(False)
        self.LaserInfoText_2.setFont(font5)
        self.LaserInfoText_2.setStyleSheet(u"QLabel#LaserInfoText_2 {\n"
"    border-bottom: 1px solid white;\n"
"}")
        self.LaserInfoText_2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_6.addWidget(self.LaserInfoText_2, 0, 0, 1, 1, Qt.AlignmentFlag.AlignTop)

        self.frame_8 = QFrame(self.LaserInfoWidget)
        self.frame_8.setObjectName(u"frame_8")
        self.frame_8.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_8.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_25 = QVBoxLayout(self.frame_8)
        self.verticalLayout_25.setObjectName(u"verticalLayout_25")
        self.verticalLayout_25.setContentsMargins(9, -1, -1, 9)
        self.h1_2 = QLabel(self.frame_8)
        self.h1_2.setObjectName(u"h1_2")
        self.h1_2.setFont(font2)

        self.verticalLayout_25.addWidget(self.h1_2)

        self.HeightCommand = QLabel(self.frame_8)
        self.HeightCommand.setObjectName(u"HeightCommand")

        self.verticalLayout_25.addWidget(self.HeightCommand)


        self.gridLayout_6.addWidget(self.frame_8, 4, 0, 1, 1)

        self.frame_7 = QFrame(self.LaserInfoWidget)
        self.frame_7.setObjectName(u"frame_7")
        self.frame_7.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_7.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_23 = QVBoxLayout(self.frame_7)
        self.verticalLayout_23.setObjectName(u"verticalLayout_23")
        self.verticalLayout_23.setContentsMargins(9, -1, -1, 9)
        self.d2_2 = QLabel(self.frame_7)
        self.d2_2.setObjectName(u"d2_2")
        self.d2_2.setFont(font2)

        self.verticalLayout_23.addWidget(self.d2_2)

        self.currentHeight = QLabel(self.frame_7)
        self.currentHeight.setObjectName(u"currentHeight")

        self.verticalLayout_23.addWidget(self.currentHeight)


        self.gridLayout_6.addWidget(self.frame_7, 1, 0, 1, 1)


        self.horizontalLayout_19.addWidget(self.LaserInfoWidget)

        self.DepthInfoWidget = QWidget(self.Info_2)
        self.DepthInfoWidget.setObjectName(u"DepthInfoWidget")
        self.DepthInfoWidget.setStyleSheet(u"QWidget#DepthInfoWidget {\n"
"	border: 2px solid #FFFFFF;  /* white solid border */\n"
"	border-radius: 6px;\n"
"}")
        self.gridLayout_13 = QGridLayout(self.DepthInfoWidget)
        self.gridLayout_13.setObjectName(u"gridLayout_13")
        self.gridLayout_13.setContentsMargins(-1, 6, -1, -1)
        self.frame_13 = QFrame(self.DepthInfoWidget)
        self.frame_13.setObjectName(u"frame_13")
        self.frame_13.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_13.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_32 = QVBoxLayout(self.frame_13)
        self.verticalLayout_32.setObjectName(u"verticalLayout_32")
        self.verticalLayout_32.setContentsMargins(9, -1, -1, 9)
        self.h1_5 = QLabel(self.frame_13)
        self.h1_5.setObjectName(u"h1_5")
        font6 = QFont()
        font6.setBold(False)
        self.h1_5.setFont(font6)

        self.verticalLayout_32.addWidget(self.h1_5)

        self.RightDepthText = QLabel(self.frame_13)
        self.RightDepthText.setObjectName(u"RightDepthText")

        self.verticalLayout_32.addWidget(self.RightDepthText)


        self.gridLayout_13.addWidget(self.frame_13, 4, 0, 1, 1)

        self.frame_14 = QFrame(self.DepthInfoWidget)
        self.frame_14.setObjectName(u"frame_14")
        self.frame_14.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_14.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_33 = QVBoxLayout(self.frame_14)
        self.verticalLayout_33.setObjectName(u"verticalLayout_33")
        self.verticalLayout_33.setContentsMargins(9, -1, -1, 9)
        self.d2_5 = QLabel(self.frame_14)
        self.d2_5.setObjectName(u"d2_5")
        self.d2_5.setFont(font2)

        self.verticalLayout_33.addWidget(self.d2_5)

        self.LeftDepthText = QLabel(self.frame_14)
        self.LeftDepthText.setObjectName(u"LeftDepthText")

        self.verticalLayout_33.addWidget(self.LeftDepthText)


        self.gridLayout_13.addWidget(self.frame_14, 1, 0, 1, 1)

        self.DepthInfoText = QLabel(self.DepthInfoWidget)
        self.DepthInfoText.setObjectName(u"DepthInfoText")
        self.DepthInfoText.setFont(font4)
        self.DepthInfoText.setStyleSheet(u"QLabel#DepthInfoText {\n"
"    border-bottom: 1px solid white;\n"
"}")
        self.DepthInfoText.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_13.addWidget(self.DepthInfoText, 0, 0, 1, 1, Qt.AlignmentFlag.AlignTop)


        self.horizontalLayout_19.addWidget(self.DepthInfoWidget)

        self.MotorInfoWidget_2.raise_()
        self.LaserInfoWidget.raise_()
        self.CartesianPoseWidget_2.raise_()
        self.DepthInfoWidget.raise_()
        self.MiddleStackedWidget.addWidget(self.VisionAndInfoPage)
        self.DIDOContainerPage = QWidget()
        self.DIDOContainerPage.setObjectName(u"DIDOContainerPage")
        self.DIDOContainerPage.setStyleSheet(u"")
        self.ScrollAreaDIDO = QScrollArea(self.DIDOContainerPage)
        self.ScrollAreaDIDO.setObjectName(u"ScrollAreaDIDO")
        self.ScrollAreaDIDO.setGeometry(QRect(0, 350, 701, 331))
        font7 = QFont()
        font7.setStrikeOut(False)
        font7.setKerning(True)
        self.ScrollAreaDIDO.setFont(font7)
        self.ScrollAreaDIDO.setTabletTracking(True)
        self.ScrollAreaDIDO.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.ScrollAreaDIDO.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.ScrollAreaDIDO.setWidgetResizable(True)
        self.scrollAreaWidgetContents_2 = QWidget()
        self.scrollAreaWidgetContents_2.setObjectName(u"scrollAreaWidgetContents_2")
        self.scrollAreaWidgetContents_2.setGeometry(QRect(0, 0, 685, 648))
        self.horizontalLayout_34 = QHBoxLayout(self.scrollAreaWidgetContents_2)
        self.horizontalLayout_34.setSpacing(9)
        self.horizontalLayout_34.setObjectName(u"horizontalLayout_34")
        self.widget_2 = QWidget(self.scrollAreaWidgetContents_2)
        self.widget_2.setObjectName(u"widget_2")
        self.widget_2.setMaximumSize(QSize(16777215, 16777215))
        self.gridLayout_7 = QGridLayout(self.widget_2)
        self.gridLayout_7.setObjectName(u"gridLayout_7")
        self.gridLayout_7.setHorizontalSpacing(0)
        self.gridLayout_7.setVerticalSpacing(10)
        self.gridLayout_7.setContentsMargins(0, 0, 0, 0)
        self.DO23 = QPushButton(self.widget_2)
        self.DO23.setObjectName(u"DO23")
        sizePolicy2.setHeightForWidth(self.DO23.sizePolicy().hasHeightForWidth())
        self.DO23.setSizePolicy(sizePolicy2)
        self.DO23.setMinimumSize(QSize(95, 70))
        self.DO23.setMaximumSize(QSize(100, 100))
        self.DO23.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO23.setCheckable(True)
        self.DO23.setChecked(False)

        self.gridLayout_7.addWidget(self.DO23, 3, 4, 1, 1)

        self.DO5 = QPushButton(self.widget_2)
        self.DO5.setObjectName(u"DO5")
        sizePolicy2.setHeightForWidth(self.DO5.sizePolicy().hasHeightForWidth())
        self.DO5.setSizePolicy(sizePolicy2)
        self.DO5.setMinimumSize(QSize(95, 70))
        self.DO5.setMaximumSize(QSize(100, 100))
        self.DO5.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO5.setCheckable(True)
        self.DO5.setChecked(False)

        self.gridLayout_7.addWidget(self.DO5, 0, 4, 1, 1)

        self.DO14 = QPushButton(self.widget_2)
        self.DO14.setObjectName(u"DO14")
        sizePolicy2.setHeightForWidth(self.DO14.sizePolicy().hasHeightForWidth())
        self.DO14.setSizePolicy(sizePolicy2)
        self.DO14.setMinimumSize(QSize(95, 70))
        self.DO14.setMaximumSize(QSize(100, 100))
        self.DO14.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO14.setCheckable(True)
        self.DO14.setChecked(False)

        self.gridLayout_7.addWidget(self.DO14, 2, 1, 1, 1)

        self.DO9 = QPushButton(self.widget_2)
        self.DO9.setObjectName(u"DO9")
        sizePolicy2.setHeightForWidth(self.DO9.sizePolicy().hasHeightForWidth())
        self.DO9.setSizePolicy(sizePolicy2)
        self.DO9.setMinimumSize(QSize(95, 70))
        self.DO9.setMaximumSize(QSize(100, 100))
        self.DO9.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO9.setCheckable(True)
        self.DO9.setChecked(False)

        self.gridLayout_7.addWidget(self.DO9, 1, 2, 1, 1)

        self.DO38 = QPushButton(self.widget_2)
        self.DO38.setObjectName(u"DO38")
        sizePolicy2.setHeightForWidth(self.DO38.sizePolicy().hasHeightForWidth())
        self.DO38.setSizePolicy(sizePolicy2)
        self.DO38.setMinimumSize(QSize(95, 70))
        self.DO38.setMaximumSize(QSize(100, 100))
        self.DO38.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO38.setCheckable(True)
        self.DO38.setChecked(False)

        self.gridLayout_7.addWidget(self.DO38, 6, 1, 1, 1)

        self.DO22 = QPushButton(self.widget_2)
        self.DO22.setObjectName(u"DO22")
        sizePolicy2.setHeightForWidth(self.DO22.sizePolicy().hasHeightForWidth())
        self.DO22.setSizePolicy(sizePolicy2)
        self.DO22.setMinimumSize(QSize(95, 70))
        self.DO22.setMaximumSize(QSize(100, 100))
        self.DO22.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO22.setCheckable(True)
        self.DO22.setChecked(False)

        self.gridLayout_7.addWidget(self.DO22, 3, 3, 1, 1)

        self.DO48 = QPushButton(self.widget_2)
        self.DO48.setObjectName(u"DO48")
        sizePolicy2.setHeightForWidth(self.DO48.sizePolicy().hasHeightForWidth())
        self.DO48.setSizePolicy(sizePolicy2)
        self.DO48.setMinimumSize(QSize(95, 70))
        self.DO48.setMaximumSize(QSize(100, 100))
        self.DO48.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO48.setCheckable(True)
        self.DO48.setChecked(False)

        self.gridLayout_7.addWidget(self.DO48, 7, 5, 1, 1)

        self.DO46 = QPushButton(self.widget_2)
        self.DO46.setObjectName(u"DO46")
        sizePolicy2.setHeightForWidth(self.DO46.sizePolicy().hasHeightForWidth())
        self.DO46.setSizePolicy(sizePolicy2)
        self.DO46.setMinimumSize(QSize(95, 70))
        self.DO46.setMaximumSize(QSize(100, 100))
        self.DO46.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO46.setCheckable(True)
        self.DO46.setChecked(False)

        self.gridLayout_7.addWidget(self.DO46, 7, 3, 1, 1)

        self.DO28 = QPushButton(self.widget_2)
        self.DO28.setObjectName(u"DO28")
        sizePolicy2.setHeightForWidth(self.DO28.sizePolicy().hasHeightForWidth())
        self.DO28.setSizePolicy(sizePolicy2)
        self.DO28.setMinimumSize(QSize(95, 70))
        self.DO28.setMaximumSize(QSize(100, 100))
        self.DO28.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO28.setCheckable(True)
        self.DO28.setChecked(False)

        self.gridLayout_7.addWidget(self.DO28, 4, 3, 1, 1)

        self.DO2 = QPushButton(self.widget_2)
        self.DO2.setObjectName(u"DO2")
        sizePolicy2.setHeightForWidth(self.DO2.sizePolicy().hasHeightForWidth())
        self.DO2.setSizePolicy(sizePolicy2)
        self.DO2.setMinimumSize(QSize(95, 70))
        self.DO2.setMaximumSize(QSize(100, 100))
        self.DO2.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO2.setCheckable(True)
        self.DO2.setChecked(False)

        self.gridLayout_7.addWidget(self.DO2, 0, 1, 1, 1)

        self.DO41 = QPushButton(self.widget_2)
        self.DO41.setObjectName(u"DO41")
        sizePolicy2.setHeightForWidth(self.DO41.sizePolicy().hasHeightForWidth())
        self.DO41.setSizePolicy(sizePolicy2)
        self.DO41.setMinimumSize(QSize(95, 70))
        self.DO41.setMaximumSize(QSize(100, 100))
        self.DO41.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO41.setCheckable(True)
        self.DO41.setChecked(False)

        self.gridLayout_7.addWidget(self.DO41, 6, 4, 1, 1)

        self.DO31 = QPushButton(self.widget_2)
        self.DO31.setObjectName(u"DO31")
        sizePolicy2.setHeightForWidth(self.DO31.sizePolicy().hasHeightForWidth())
        self.DO31.setSizePolicy(sizePolicy2)
        self.DO31.setMinimumSize(QSize(95, 70))
        self.DO31.setMaximumSize(QSize(100, 100))
        self.DO31.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO31.setCheckable(True)
        self.DO31.setChecked(False)

        self.gridLayout_7.addWidget(self.DO31, 5, 0, 1, 1)

        self.DO11 = QPushButton(self.widget_2)
        self.DO11.setObjectName(u"DO11")
        sizePolicy2.setHeightForWidth(self.DO11.sizePolicy().hasHeightForWidth())
        self.DO11.setSizePolicy(sizePolicy2)
        self.DO11.setMinimumSize(QSize(95, 70))
        self.DO11.setMaximumSize(QSize(100, 100))
        self.DO11.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO11.setCheckable(True)
        self.DO11.setChecked(False)

        self.gridLayout_7.addWidget(self.DO11, 1, 4, 1, 1)

        self.DO17 = QPushButton(self.widget_2)
        self.DO17.setObjectName(u"DO17")
        sizePolicy2.setHeightForWidth(self.DO17.sizePolicy().hasHeightForWidth())
        self.DO17.setSizePolicy(sizePolicy2)
        self.DO17.setMinimumSize(QSize(95, 70))
        self.DO17.setMaximumSize(QSize(100, 100))
        self.DO17.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO17.setCheckable(True)
        self.DO17.setChecked(False)

        self.gridLayout_7.addWidget(self.DO17, 2, 4, 1, 1)

        self.DO4 = QPushButton(self.widget_2)
        self.DO4.setObjectName(u"DO4")
        sizePolicy2.setHeightForWidth(self.DO4.sizePolicy().hasHeightForWidth())
        self.DO4.setSizePolicy(sizePolicy2)
        self.DO4.setMinimumSize(QSize(95, 70))
        self.DO4.setMaximumSize(QSize(100, 100))
        self.DO4.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO4.setCheckable(True)
        self.DO4.setChecked(False)

        self.gridLayout_7.addWidget(self.DO4, 0, 3, 1, 1)

        self.DO20 = QPushButton(self.widget_2)
        self.DO20.setObjectName(u"DO20")
        sizePolicy2.setHeightForWidth(self.DO20.sizePolicy().hasHeightForWidth())
        self.DO20.setSizePolicy(sizePolicy2)
        self.DO20.setMinimumSize(QSize(95, 70))
        self.DO20.setMaximumSize(QSize(100, 100))
        self.DO20.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO20.setCheckable(True)
        self.DO20.setChecked(False)

        self.gridLayout_7.addWidget(self.DO20, 3, 1, 1, 1)

        self.DO15 = QPushButton(self.widget_2)
        self.DO15.setObjectName(u"DO15")
        sizePolicy2.setHeightForWidth(self.DO15.sizePolicy().hasHeightForWidth())
        self.DO15.setSizePolicy(sizePolicy2)
        self.DO15.setMinimumSize(QSize(95, 70))
        self.DO15.setMaximumSize(QSize(100, 100))
        self.DO15.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO15.setCheckable(True)
        self.DO15.setChecked(False)

        self.gridLayout_7.addWidget(self.DO15, 2, 2, 1, 1)

        self.DO19 = QPushButton(self.widget_2)
        self.DO19.setObjectName(u"DO19")
        sizePolicy2.setHeightForWidth(self.DO19.sizePolicy().hasHeightForWidth())
        self.DO19.setSizePolicy(sizePolicy2)
        self.DO19.setMinimumSize(QSize(95, 70))
        self.DO19.setMaximumSize(QSize(100, 100))
        self.DO19.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO19.setCheckable(True)
        self.DO19.setChecked(False)

        self.gridLayout_7.addWidget(self.DO19, 3, 0, 1, 1)

        self.DO21 = QPushButton(self.widget_2)
        self.DO21.setObjectName(u"DO21")
        sizePolicy2.setHeightForWidth(self.DO21.sizePolicy().hasHeightForWidth())
        self.DO21.setSizePolicy(sizePolicy2)
        self.DO21.setMinimumSize(QSize(95, 70))
        self.DO21.setMaximumSize(QSize(100, 100))
        self.DO21.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO21.setCheckable(True)
        self.DO21.setChecked(False)

        self.gridLayout_7.addWidget(self.DO21, 3, 2, 1, 1)

        self.DO18 = QPushButton(self.widget_2)
        self.DO18.setObjectName(u"DO18")
        sizePolicy2.setHeightForWidth(self.DO18.sizePolicy().hasHeightForWidth())
        self.DO18.setSizePolicy(sizePolicy2)
        self.DO18.setMinimumSize(QSize(95, 70))
        self.DO18.setMaximumSize(QSize(100, 100))
        self.DO18.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO18.setCheckable(True)
        self.DO18.setChecked(False)

        self.gridLayout_7.addWidget(self.DO18, 2, 5, 1, 1)

        self.DO36 = QPushButton(self.widget_2)
        self.DO36.setObjectName(u"DO36")
        sizePolicy2.setHeightForWidth(self.DO36.sizePolicy().hasHeightForWidth())
        self.DO36.setSizePolicy(sizePolicy2)
        self.DO36.setMinimumSize(QSize(95, 70))
        self.DO36.setMaximumSize(QSize(100, 100))
        self.DO36.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO36.setCheckable(True)
        self.DO36.setChecked(False)

        self.gridLayout_7.addWidget(self.DO36, 5, 5, 1, 1)

        self.DO8 = QPushButton(self.widget_2)
        self.DO8.setObjectName(u"DO8")
        sizePolicy2.setHeightForWidth(self.DO8.sizePolicy().hasHeightForWidth())
        self.DO8.setSizePolicy(sizePolicy2)
        self.DO8.setMinimumSize(QSize(95, 70))
        self.DO8.setMaximumSize(QSize(100, 100))
        self.DO8.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO8.setCheckable(True)
        self.DO8.setChecked(False)

        self.gridLayout_7.addWidget(self.DO8, 1, 1, 1, 1)

        self.DO42 = QPushButton(self.widget_2)
        self.DO42.setObjectName(u"DO42")
        sizePolicy2.setHeightForWidth(self.DO42.sizePolicy().hasHeightForWidth())
        self.DO42.setSizePolicy(sizePolicy2)
        self.DO42.setMinimumSize(QSize(95, 70))
        self.DO42.setMaximumSize(QSize(100, 100))
        self.DO42.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO42.setCheckable(True)
        self.DO42.setChecked(False)

        self.gridLayout_7.addWidget(self.DO42, 6, 5, 1, 1)

        self.DO26 = QPushButton(self.widget_2)
        self.DO26.setObjectName(u"DO26")
        sizePolicy2.setHeightForWidth(self.DO26.sizePolicy().hasHeightForWidth())
        self.DO26.setSizePolicy(sizePolicy2)
        self.DO26.setMinimumSize(QSize(95, 70))
        self.DO26.setMaximumSize(QSize(100, 100))
        self.DO26.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO26.setCheckable(True)
        self.DO26.setChecked(False)

        self.gridLayout_7.addWidget(self.DO26, 4, 1, 1, 1)

        self.DO1 = QPushButton(self.widget_2)
        self.DO1.setObjectName(u"DO1")
        sizePolicy2.setHeightForWidth(self.DO1.sizePolicy().hasHeightForWidth())
        self.DO1.setSizePolicy(sizePolicy2)
        self.DO1.setMinimumSize(QSize(95, 70))
        self.DO1.setMaximumSize(QSize(100, 100))
        self.DO1.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO1.setCheckable(True)
        self.DO1.setChecked(False)

        self.gridLayout_7.addWidget(self.DO1, 0, 0, 1, 1)

        self.DO44 = QPushButton(self.widget_2)
        self.DO44.setObjectName(u"DO44")
        sizePolicy2.setHeightForWidth(self.DO44.sizePolicy().hasHeightForWidth())
        self.DO44.setSizePolicy(sizePolicy2)
        self.DO44.setMinimumSize(QSize(95, 70))
        self.DO44.setMaximumSize(QSize(100, 100))
        self.DO44.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO44.setCheckable(True)
        self.DO44.setChecked(False)

        self.gridLayout_7.addWidget(self.DO44, 7, 1, 1, 1)

        self.DO16 = QPushButton(self.widget_2)
        self.DO16.setObjectName(u"DO16")
        sizePolicy2.setHeightForWidth(self.DO16.sizePolicy().hasHeightForWidth())
        self.DO16.setSizePolicy(sizePolicy2)
        self.DO16.setMinimumSize(QSize(95, 70))
        self.DO16.setMaximumSize(QSize(100, 100))
        self.DO16.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO16.setCheckable(True)
        self.DO16.setChecked(False)

        self.gridLayout_7.addWidget(self.DO16, 2, 3, 1, 1)

        self.DO32 = QPushButton(self.widget_2)
        self.DO32.setObjectName(u"DO32")
        sizePolicy2.setHeightForWidth(self.DO32.sizePolicy().hasHeightForWidth())
        self.DO32.setSizePolicy(sizePolicy2)
        self.DO32.setMinimumSize(QSize(95, 70))
        self.DO32.setMaximumSize(QSize(100, 100))
        self.DO32.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO32.setCheckable(True)
        self.DO32.setChecked(False)

        self.gridLayout_7.addWidget(self.DO32, 5, 1, 1, 1)

        self.DO34 = QPushButton(self.widget_2)
        self.DO34.setObjectName(u"DO34")
        sizePolicy2.setHeightForWidth(self.DO34.sizePolicy().hasHeightForWidth())
        self.DO34.setSizePolicy(sizePolicy2)
        self.DO34.setMinimumSize(QSize(95, 70))
        self.DO34.setMaximumSize(QSize(100, 100))
        self.DO34.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO34.setCheckable(True)
        self.DO34.setChecked(False)

        self.gridLayout_7.addWidget(self.DO34, 5, 3, 1, 1)

        self.DO10 = QPushButton(self.widget_2)
        self.DO10.setObjectName(u"DO10")
        sizePolicy2.setHeightForWidth(self.DO10.sizePolicy().hasHeightForWidth())
        self.DO10.setSizePolicy(sizePolicy2)
        self.DO10.setMinimumSize(QSize(95, 70))
        self.DO10.setMaximumSize(QSize(100, 100))
        self.DO10.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO10.setCheckable(True)
        self.DO10.setChecked(False)

        self.gridLayout_7.addWidget(self.DO10, 1, 3, 1, 1)

        self.DO29 = QPushButton(self.widget_2)
        self.DO29.setObjectName(u"DO29")
        sizePolicy2.setHeightForWidth(self.DO29.sizePolicy().hasHeightForWidth())
        self.DO29.setSizePolicy(sizePolicy2)
        self.DO29.setMinimumSize(QSize(95, 70))
        self.DO29.setMaximumSize(QSize(100, 100))
        self.DO29.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO29.setCheckable(True)
        self.DO29.setChecked(False)

        self.gridLayout_7.addWidget(self.DO29, 4, 4, 1, 1)

        self.DO35 = QPushButton(self.widget_2)
        self.DO35.setObjectName(u"DO35")
        sizePolicy2.setHeightForWidth(self.DO35.sizePolicy().hasHeightForWidth())
        self.DO35.setSizePolicy(sizePolicy2)
        self.DO35.setMinimumSize(QSize(95, 70))
        self.DO35.setMaximumSize(QSize(100, 100))
        self.DO35.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO35.setCheckable(True)
        self.DO35.setChecked(False)

        self.gridLayout_7.addWidget(self.DO35, 5, 4, 1, 1)

        self.DO37 = QPushButton(self.widget_2)
        self.DO37.setObjectName(u"DO37")
        sizePolicy2.setHeightForWidth(self.DO37.sizePolicy().hasHeightForWidth())
        self.DO37.setSizePolicy(sizePolicy2)
        self.DO37.setMinimumSize(QSize(95, 70))
        self.DO37.setMaximumSize(QSize(100, 100))
        self.DO37.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO37.setCheckable(True)
        self.DO37.setChecked(False)

        self.gridLayout_7.addWidget(self.DO37, 6, 0, 1, 1)

        self.DO12 = QPushButton(self.widget_2)
        self.DO12.setObjectName(u"DO12")
        sizePolicy2.setHeightForWidth(self.DO12.sizePolicy().hasHeightForWidth())
        self.DO12.setSizePolicy(sizePolicy2)
        self.DO12.setMinimumSize(QSize(95, 70))
        self.DO12.setMaximumSize(QSize(100, 100))
        self.DO12.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO12.setCheckable(True)
        self.DO12.setChecked(False)

        self.gridLayout_7.addWidget(self.DO12, 1, 5, 1, 1)

        self.DO43 = QPushButton(self.widget_2)
        self.DO43.setObjectName(u"DO43")
        sizePolicy2.setHeightForWidth(self.DO43.sizePolicy().hasHeightForWidth())
        self.DO43.setSizePolicy(sizePolicy2)
        self.DO43.setMinimumSize(QSize(95, 70))
        self.DO43.setMaximumSize(QSize(100, 100))
        self.DO43.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO43.setCheckable(True)
        self.DO43.setChecked(False)

        self.gridLayout_7.addWidget(self.DO43, 7, 0, 1, 1)

        self.DO47 = QPushButton(self.widget_2)
        self.DO47.setObjectName(u"DO47")
        sizePolicy2.setHeightForWidth(self.DO47.sizePolicy().hasHeightForWidth())
        self.DO47.setSizePolicy(sizePolicy2)
        self.DO47.setMinimumSize(QSize(95, 70))
        self.DO47.setMaximumSize(QSize(100, 100))
        self.DO47.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO47.setCheckable(True)
        self.DO47.setChecked(False)

        self.gridLayout_7.addWidget(self.DO47, 7, 4, 1, 1)

        self.DO30 = QPushButton(self.widget_2)
        self.DO30.setObjectName(u"DO30")
        sizePolicy2.setHeightForWidth(self.DO30.sizePolicy().hasHeightForWidth())
        self.DO30.setSizePolicy(sizePolicy2)
        self.DO30.setMinimumSize(QSize(95, 70))
        self.DO30.setMaximumSize(QSize(100, 100))
        self.DO30.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO30.setCheckable(True)
        self.DO30.setChecked(False)

        self.gridLayout_7.addWidget(self.DO30, 4, 5, 1, 1)

        self.DO24 = QPushButton(self.widget_2)
        self.DO24.setObjectName(u"DO24")
        sizePolicy2.setHeightForWidth(self.DO24.sizePolicy().hasHeightForWidth())
        self.DO24.setSizePolicy(sizePolicy2)
        self.DO24.setMinimumSize(QSize(95, 70))
        self.DO24.setMaximumSize(QSize(100, 100))
        self.DO24.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO24.setCheckable(True)
        self.DO24.setChecked(False)

        self.gridLayout_7.addWidget(self.DO24, 3, 5, 1, 1)

        self.DO25 = QPushButton(self.widget_2)
        self.DO25.setObjectName(u"DO25")
        sizePolicy2.setHeightForWidth(self.DO25.sizePolicy().hasHeightForWidth())
        self.DO25.setSizePolicy(sizePolicy2)
        self.DO25.setMinimumSize(QSize(95, 70))
        self.DO25.setMaximumSize(QSize(100, 100))
        self.DO25.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO25.setCheckable(True)
        self.DO25.setChecked(False)

        self.gridLayout_7.addWidget(self.DO25, 4, 0, 1, 1)

        self.DO6 = QPushButton(self.widget_2)
        self.DO6.setObjectName(u"DO6")
        sizePolicy2.setHeightForWidth(self.DO6.sizePolicy().hasHeightForWidth())
        self.DO6.setSizePolicy(sizePolicy2)
        self.DO6.setMinimumSize(QSize(95, 70))
        self.DO6.setMaximumSize(QSize(100, 100))
        self.DO6.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO6.setCheckable(True)
        self.DO6.setChecked(False)

        self.gridLayout_7.addWidget(self.DO6, 0, 5, 1, 1)

        self.DO13 = QPushButton(self.widget_2)
        self.DO13.setObjectName(u"DO13")
        sizePolicy2.setHeightForWidth(self.DO13.sizePolicy().hasHeightForWidth())
        self.DO13.setSizePolicy(sizePolicy2)
        self.DO13.setMinimumSize(QSize(95, 70))
        self.DO13.setMaximumSize(QSize(100, 100))
        self.DO13.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO13.setCheckable(True)
        self.DO13.setChecked(False)

        self.gridLayout_7.addWidget(self.DO13, 2, 0, 1, 1)

        self.DO7 = QPushButton(self.widget_2)
        self.DO7.setObjectName(u"DO7")
        sizePolicy2.setHeightForWidth(self.DO7.sizePolicy().hasHeightForWidth())
        self.DO7.setSizePolicy(sizePolicy2)
        self.DO7.setMinimumSize(QSize(95, 70))
        self.DO7.setMaximumSize(QSize(100, 100))
        self.DO7.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO7.setCheckable(True)
        self.DO7.setChecked(False)

        self.gridLayout_7.addWidget(self.DO7, 1, 0, 1, 1)

        self.DO33 = QPushButton(self.widget_2)
        self.DO33.setObjectName(u"DO33")
        sizePolicy2.setHeightForWidth(self.DO33.sizePolicy().hasHeightForWidth())
        self.DO33.setSizePolicy(sizePolicy2)
        self.DO33.setMinimumSize(QSize(95, 70))
        self.DO33.setMaximumSize(QSize(100, 100))
        self.DO33.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO33.setCheckable(True)
        self.DO33.setChecked(False)

        self.gridLayout_7.addWidget(self.DO33, 5, 2, 1, 1)

        self.DO3 = QPushButton(self.widget_2)
        self.DO3.setObjectName(u"DO3")
        sizePolicy2.setHeightForWidth(self.DO3.sizePolicy().hasHeightForWidth())
        self.DO3.setSizePolicy(sizePolicy2)
        self.DO3.setMinimumSize(QSize(95, 70))
        self.DO3.setMaximumSize(QSize(100, 100))
        self.DO3.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO3.setCheckable(True)
        self.DO3.setChecked(False)

        self.gridLayout_7.addWidget(self.DO3, 0, 2, 1, 1)

        self.DO27 = QPushButton(self.widget_2)
        self.DO27.setObjectName(u"DO27")
        sizePolicy2.setHeightForWidth(self.DO27.sizePolicy().hasHeightForWidth())
        self.DO27.setSizePolicy(sizePolicy2)
        self.DO27.setMinimumSize(QSize(95, 70))
        self.DO27.setMaximumSize(QSize(100, 100))
        self.DO27.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO27.setCheckable(True)
        self.DO27.setChecked(False)

        self.gridLayout_7.addWidget(self.DO27, 4, 2, 1, 1)

        self.DO40 = QPushButton(self.widget_2)
        self.DO40.setObjectName(u"DO40")
        sizePolicy2.setHeightForWidth(self.DO40.sizePolicy().hasHeightForWidth())
        self.DO40.setSizePolicy(sizePolicy2)
        self.DO40.setMinimumSize(QSize(95, 70))
        self.DO40.setMaximumSize(QSize(100, 100))
        self.DO40.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO40.setCheckable(True)
        self.DO40.setChecked(False)

        self.gridLayout_7.addWidget(self.DO40, 6, 3, 1, 1)

        self.DO39 = QPushButton(self.widget_2)
        self.DO39.setObjectName(u"DO39")
        sizePolicy2.setHeightForWidth(self.DO39.sizePolicy().hasHeightForWidth())
        self.DO39.setSizePolicy(sizePolicy2)
        self.DO39.setMinimumSize(QSize(95, 70))
        self.DO39.setMaximumSize(QSize(100, 100))
        self.DO39.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO39.setCheckable(True)
        self.DO39.setChecked(False)

        self.gridLayout_7.addWidget(self.DO39, 6, 2, 1, 1)

        self.DO45 = QPushButton(self.widget_2)
        self.DO45.setObjectName(u"DO45")
        sizePolicy2.setHeightForWidth(self.DO45.sizePolicy().hasHeightForWidth())
        self.DO45.setSizePolicy(sizePolicy2)
        self.DO45.setMinimumSize(QSize(95, 70))
        self.DO45.setMaximumSize(QSize(100, 100))
        self.DO45.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DO45.setCheckable(True)
        self.DO45.setChecked(False)

        self.gridLayout_7.addWidget(self.DO45, 7, 2, 1, 1)


        self.horizontalLayout_34.addWidget(self.widget_2)

        self.ScrollAreaDIDO.setWidget(self.scrollAreaWidgetContents_2)
        self.gridLayoutWidget = QWidget(self.DIDOContainerPage)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(20, 20, 661, 301))
        self.gridLayout_8 = QGridLayout(self.gridLayoutWidget)
        self.gridLayout_8.setObjectName(u"gridLayout_8")
        self.gridLayout_8.setContentsMargins(0, 0, 0, 1)
        self.DI8 = QPushButton(self.gridLayoutWidget)
        self.DI8.setObjectName(u"DI8")
        self.DI8.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI8.sizePolicy().hasHeightForWidth())
        self.DI8.setSizePolicy(sizePolicy2)
        self.DI8.setMinimumSize(QSize(95, 70))
        self.DI8.setMaximumSize(QSize(100, 100))
        self.DI8.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI8.setCheckable(True)
        self.DI8.setChecked(False)

        self.gridLayout_8.addWidget(self.DI8, 1, 3, 1, 1)

        self.DI3 = QPushButton(self.gridLayoutWidget)
        self.DI3.setObjectName(u"DI3")
        self.DI3.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI3.sizePolicy().hasHeightForWidth())
        self.DI3.setSizePolicy(sizePolicy2)
        self.DI3.setMinimumSize(QSize(95, 70))
        self.DI3.setMaximumSize(QSize(100, 100))
        self.DI3.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI3.setCheckable(True)
        self.DI3.setChecked(False)

        self.gridLayout_8.addWidget(self.DI3, 0, 2, 1, 1)

        self.DI12 = QPushButton(self.gridLayoutWidget)
        self.DI12.setObjectName(u"DI12")
        self.DI12.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI12.sizePolicy().hasHeightForWidth())
        self.DI12.setSizePolicy(sizePolicy2)
        self.DI12.setMinimumSize(QSize(95, 70))
        self.DI12.setMaximumSize(QSize(100, 100))
        self.DI12.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI12.setCheckable(True)
        self.DI12.setChecked(False)

        self.gridLayout_8.addWidget(self.DI12, 2, 3, 1, 1)

        self.DI7 = QPushButton(self.gridLayoutWidget)
        self.DI7.setObjectName(u"DI7")
        self.DI7.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI7.sizePolicy().hasHeightForWidth())
        self.DI7.setSizePolicy(sizePolicy2)
        self.DI7.setMinimumSize(QSize(95, 70))
        self.DI7.setMaximumSize(QSize(100, 100))
        self.DI7.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI7.setCheckable(True)
        self.DI7.setChecked(False)

        self.gridLayout_8.addWidget(self.DI7, 1, 2, 1, 1)

        self.DI11 = QPushButton(self.gridLayoutWidget)
        self.DI11.setObjectName(u"DI11")
        self.DI11.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI11.sizePolicy().hasHeightForWidth())
        self.DI11.setSizePolicy(sizePolicy2)
        self.DI11.setMinimumSize(QSize(95, 70))
        self.DI11.setMaximumSize(QSize(100, 100))
        self.DI11.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI11.setCheckable(True)
        self.DI11.setChecked(False)

        self.gridLayout_8.addWidget(self.DI11, 2, 2, 1, 1)

        self.DI16 = QPushButton(self.gridLayoutWidget)
        self.DI16.setObjectName(u"DI16")
        self.DI16.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI16.sizePolicy().hasHeightForWidth())
        self.DI16.setSizePolicy(sizePolicy2)
        self.DI16.setMinimumSize(QSize(95, 70))
        self.DI16.setMaximumSize(QSize(100, 100))
        self.DI16.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI16.setCheckable(True)
        self.DI16.setChecked(False)

        self.gridLayout_8.addWidget(self.DI16, 3, 3, 1, 1)

        self.DI9 = QPushButton(self.gridLayoutWidget)
        self.DI9.setObjectName(u"DI9")
        self.DI9.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI9.sizePolicy().hasHeightForWidth())
        self.DI9.setSizePolicy(sizePolicy2)
        self.DI9.setMinimumSize(QSize(95, 70))
        self.DI9.setMaximumSize(QSize(100, 100))
        self.DI9.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI9.setCheckable(True)
        self.DI9.setChecked(False)

        self.gridLayout_8.addWidget(self.DI9, 2, 0, 1, 1)

        self.DI4 = QPushButton(self.gridLayoutWidget)
        self.DI4.setObjectName(u"DI4")
        self.DI4.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI4.sizePolicy().hasHeightForWidth())
        self.DI4.setSizePolicy(sizePolicy2)
        self.DI4.setMinimumSize(QSize(95, 70))
        self.DI4.setMaximumSize(QSize(100, 100))
        self.DI4.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI4.setCheckable(True)
        self.DI4.setChecked(False)

        self.gridLayout_8.addWidget(self.DI4, 0, 3, 1, 1)

        self.DI2 = QPushButton(self.gridLayoutWidget)
        self.DI2.setObjectName(u"DI2")
        self.DI2.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI2.sizePolicy().hasHeightForWidth())
        self.DI2.setSizePolicy(sizePolicy2)
        self.DI2.setMinimumSize(QSize(95, 70))
        self.DI2.setMaximumSize(QSize(100, 100))
        self.DI2.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI2.setCheckable(True)
        self.DI2.setChecked(False)

        self.gridLayout_8.addWidget(self.DI2, 0, 1, 1, 1)

        self.DI14 = QPushButton(self.gridLayoutWidget)
        self.DI14.setObjectName(u"DI14")
        self.DI14.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI14.sizePolicy().hasHeightForWidth())
        self.DI14.setSizePolicy(sizePolicy2)
        self.DI14.setMinimumSize(QSize(95, 70))
        self.DI14.setMaximumSize(QSize(100, 100))
        self.DI14.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI14.setCheckable(True)
        self.DI14.setChecked(False)

        self.gridLayout_8.addWidget(self.DI14, 3, 1, 1, 1)

        self.DI15 = QPushButton(self.gridLayoutWidget)
        self.DI15.setObjectName(u"DI15")
        self.DI15.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI15.sizePolicy().hasHeightForWidth())
        self.DI15.setSizePolicy(sizePolicy2)
        self.DI15.setMinimumSize(QSize(95, 70))
        self.DI15.setMaximumSize(QSize(100, 100))
        self.DI15.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI15.setCheckable(True)
        self.DI15.setChecked(False)

        self.gridLayout_8.addWidget(self.DI15, 3, 2, 1, 1)

        self.DI6 = QPushButton(self.gridLayoutWidget)
        self.DI6.setObjectName(u"DI6")
        self.DI6.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI6.sizePolicy().hasHeightForWidth())
        self.DI6.setSizePolicy(sizePolicy2)
        self.DI6.setMinimumSize(QSize(95, 70))
        self.DI6.setMaximumSize(QSize(100, 100))
        self.DI6.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI6.setCheckable(True)
        self.DI6.setChecked(False)

        self.gridLayout_8.addWidget(self.DI6, 1, 1, 1, 1)

        self.DI1 = QPushButton(self.gridLayoutWidget)
        self.DI1.setObjectName(u"DI1")
        self.DI1.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI1.sizePolicy().hasHeightForWidth())
        self.DI1.setSizePolicy(sizePolicy2)
        self.DI1.setMinimumSize(QSize(95, 70))
        self.DI1.setMaximumSize(QSize(100, 100))
        self.DI1.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI1.setCheckable(True)
        self.DI1.setChecked(False)

        self.gridLayout_8.addWidget(self.DI1, 0, 0, 1, 1)

        self.DI10 = QPushButton(self.gridLayoutWidget)
        self.DI10.setObjectName(u"DI10")
        self.DI10.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI10.sizePolicy().hasHeightForWidth())
        self.DI10.setSizePolicy(sizePolicy2)
        self.DI10.setMinimumSize(QSize(95, 70))
        self.DI10.setMaximumSize(QSize(100, 100))
        self.DI10.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI10.setCheckable(True)
        self.DI10.setChecked(False)

        self.gridLayout_8.addWidget(self.DI10, 2, 1, 1, 1)

        self.DI13 = QPushButton(self.gridLayoutWidget)
        self.DI13.setObjectName(u"DI13")
        self.DI13.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI13.sizePolicy().hasHeightForWidth())
        self.DI13.setSizePolicy(sizePolicy2)
        self.DI13.setMinimumSize(QSize(95, 70))
        self.DI13.setMaximumSize(QSize(100, 100))
        self.DI13.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI13.setCheckable(True)
        self.DI13.setChecked(False)

        self.gridLayout_8.addWidget(self.DI13, 3, 0, 1, 1)

        self.DI5 = QPushButton(self.gridLayoutWidget)
        self.DI5.setObjectName(u"DI5")
        self.DI5.setEnabled(False)
        sizePolicy2.setHeightForWidth(self.DI5.sizePolicy().hasHeightForWidth())
        self.DI5.setSizePolicy(sizePolicy2)
        self.DI5.setMinimumSize(QSize(95, 70))
        self.DI5.setMaximumSize(QSize(100, 100))
        self.DI5.setStyleSheet(u"    border: none;\n"
"    border-radius: 24px;\n"
"")
        self.DI5.setCheckable(True)
        self.DI5.setChecked(False)

        self.gridLayout_8.addWidget(self.DI5, 1, 0, 1, 1)

        self.label_5 = QLabel(self.DIDOContainerPage)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(320, 0, 67, 17))
        font8 = QFont()
        font8.setPointSize(16)
        self.label_5.setFont(font8)
        self.label_5.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_8 = QLabel(self.DIDOContainerPage)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(320, 330, 67, 17))
        self.label_8.setFont(font8)
        self.label_8.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.MiddleStackedWidget.addWidget(self.DIDOContainerPage)
        self.ParentStackedWidgetToChangeMenuOptions.addWidget(self.ComponentControlPage)
        self.ProductionRecordPage = QWidget()
        self.ProductionRecordPage.setObjectName(u"ProductionRecordPage")
        self.frame = QFrame(self.ProductionRecordPage)
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(490, 250, 119, 36))
        self.frame.setStyleSheet(u"background-color: #000000; color:white;")
        self.frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_18 = QVBoxLayout(self.frame)
        self.verticalLayout_18.setObjectName(u"verticalLayout_18")
        self.label = QLabel(self.frame)
        self.label.setObjectName(u"label")

        self.verticalLayout_18.addWidget(self.label)

        self.ParentStackedWidgetToChangeMenuOptions.addWidget(self.ProductionRecordPage)
        self.LogsPage = QWidget()
        self.LogsPage.setObjectName(u"LogsPage")
        self.frame_3 = QFrame(self.LogsPage)
        self.frame_3.setObjectName(u"frame_3")
        self.frame_3.setGeometry(QRect(430, 260, 45, 36))
        self.frame_3.setStyleSheet(u"background-color: #000000; color: white;")
        self.frame_3.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_3.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_19 = QVBoxLayout(self.frame_3)
        self.verticalLayout_19.setObjectName(u"verticalLayout_19")
        self.label_3 = QLabel(self.frame_3)
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout_19.addWidget(self.label_3)

        self.ParentStackedWidgetToChangeMenuOptions.addWidget(self.LogsPage)
        self.SystemSettingsPage = QWidget()
        self.SystemSettingsPage.setObjectName(u"SystemSettingsPage")
        self.ChangeLanguageText = QLabel(self.SystemSettingsPage)
        self.ChangeLanguageText.setObjectName(u"ChangeLanguageText")
        self.ChangeLanguageText.setGeometry(QRect(410, 20, 201, 31))
        self.ChangeLanguageText.setFont(font1)
        self.ChangeLanguageText.setStyleSheet(u"color: white;")
        self.ChangeLanguageText.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.FontSizeText = QLabel(self.SystemSettingsPage)
        self.FontSizeText.setObjectName(u"FontSizeText")
        self.FontSizeText.setGeometry(QRect(410, 210, 201, 31))
        self.FontSizeText.setFont(font1)
        self.FontSizeText.setStyleSheet(u"color: white;")
        self.FontSizeText.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.ThemeText = QLabel(self.SystemSettingsPage)
        self.ThemeText.setObjectName(u"ThemeText")
        self.ThemeText.setGeometry(QRect(410, 400, 201, 31))
        self.ThemeText.setFont(font1)
        self.ThemeText.setStyleSheet(u"color: white;")
        self.ThemeText.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.SaveChangesButton = QPushButton(self.SystemSettingsPage)
        self.SaveChangesButton.setObjectName(u"SaveChangesButton")
        self.SaveChangesButton.setGeometry(QRect(430, 620, 161, 61))
        self.EnglishLanguageButton = QPushButton(self.SystemSettingsPage)
        self.LanguageButtonGroup = QButtonGroup(MainWindow)
        self.LanguageButtonGroup.setObjectName(u"LanguageButtonGroup")
        self.LanguageButtonGroup.addButton(self.EnglishLanguageButton)
        self.EnglishLanguageButton.setObjectName(u"EnglishLanguageButton")
        self.EnglishLanguageButton.setGeometry(QRect(240, 80, 151, 71))
        self.EnglishLanguageButton.setCheckable(True)
        self.EnglishLanguageButton.setChecked(True)
        self.ChineseLanguageButton = QPushButton(self.SystemSettingsPage)
        self.LanguageButtonGroup.addButton(self.ChineseLanguageButton)
        self.ChineseLanguageButton.setObjectName(u"ChineseLanguageButton")
        self.ChineseLanguageButton.setGeometry(QRect(430, 80, 151, 71))
        self.ChineseLanguageButton.setCheckable(True)
        self.ThaiLanguageButton = QPushButton(self.SystemSettingsPage)
        self.LanguageButtonGroup.addButton(self.ThaiLanguageButton)
        self.ThaiLanguageButton.setObjectName(u"ThaiLanguageButton")
        self.ThaiLanguageButton.setGeometry(QRect(620, 80, 151, 71))
        self.ThaiLanguageButton.setCheckable(True)
        self.BigFontSizeButton = QPushButton(self.SystemSettingsPage)
        self.FontSizeButtonGroup = QButtonGroup(MainWindow)
        self.FontSizeButtonGroup.setObjectName(u"FontSizeButtonGroup")
        self.FontSizeButtonGroup.addButton(self.BigFontSizeButton)
        self.BigFontSizeButton.setObjectName(u"BigFontSizeButton")
        self.BigFontSizeButton.setGeometry(QRect(620, 270, 151, 71))
        self.BigFontSizeButton.setCheckable(True)
        self.NormalFontSizeButton = QPushButton(self.SystemSettingsPage)
        self.FontSizeButtonGroup.addButton(self.NormalFontSizeButton)
        self.NormalFontSizeButton.setObjectName(u"NormalFontSizeButton")
        self.NormalFontSizeButton.setGeometry(QRect(430, 270, 151, 71))
        self.NormalFontSizeButton.setCheckable(True)
        self.NormalFontSizeButton.setChecked(True)
        self.SmallFontSizeButton = QPushButton(self.SystemSettingsPage)
        self.FontSizeButtonGroup.addButton(self.SmallFontSizeButton)
        self.SmallFontSizeButton.setObjectName(u"SmallFontSizeButton")
        self.SmallFontSizeButton.setGeometry(QRect(240, 270, 151, 71))
        self.SmallFontSizeButton.setCheckable(True)
        self.DarkThemeButton = QPushButton(self.SystemSettingsPage)
        self.ThemeButtonGroup = QButtonGroup(MainWindow)
        self.ThemeButtonGroup.setObjectName(u"ThemeButtonGroup")
        self.ThemeButtonGroup.addButton(self.DarkThemeButton)
        self.DarkThemeButton.setObjectName(u"DarkThemeButton")
        self.DarkThemeButton.setGeometry(QRect(340, 460, 151, 71))
        self.DarkThemeButton.setCheckable(True)
        self.DarkThemeButton.setChecked(True)
        self.LightThemeButton = QPushButton(self.SystemSettingsPage)
        self.ThemeButtonGroup.addButton(self.LightThemeButton)
        self.LightThemeButton.setObjectName(u"LightThemeButton")
        self.LightThemeButton.setGeometry(QRect(530, 460, 151, 71))
        self.LightThemeButton.setCheckable(True)
        self.ParentStackedWidgetToChangeMenuOptions.addWidget(self.SystemSettingsPage)
        self.WorkOrderWidget = QWidget(self.BackgroundWidget)
        self.WorkOrderWidget.setObjectName(u"WorkOrderWidget")
        self.WorkOrderWidget.setGeometry(QRect(470, 0, 141, 41))
        self.WorkOrderWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_17 = QHBoxLayout(self.WorkOrderWidget)
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.WorkOrderText = QLabel(self.WorkOrderWidget)
        self.WorkOrderText.setObjectName(u"WorkOrderText")
        font9 = QFont()
        font9.setBold(True)
        self.WorkOrderText.setFont(font9)

        self.horizontalLayout_17.addWidget(self.WorkOrderText)

        self.WorkOrderNumberInput = QLabel(self.WorkOrderWidget)
        self.WorkOrderNumberInput.setObjectName(u"WorkOrderNumberInput")

        self.horizontalLayout_17.addWidget(self.WorkOrderNumberInput)

        self.RecipeWidget = QWidget(self.BackgroundWidget)
        self.RecipeWidget.setObjectName(u"RecipeWidget")
        self.RecipeWidget.setGeometry(QRect(450, 30, 171, 41))
        self.RecipeWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_18 = QHBoxLayout(self.RecipeWidget)
        self.horizontalLayout_18.setObjectName(u"horizontalLayout_18")
        self.RecipeText = QLabel(self.RecipeWidget)
        self.RecipeText.setObjectName(u"RecipeText")
        self.RecipeText.setFont(font9)

        self.horizontalLayout_18.addWidget(self.RecipeText)

        self.RecipeNameInput = QLabel(self.RecipeWidget)
        self.RecipeNameInput.setObjectName(u"RecipeNameInput")

        self.horizontalLayout_18.addWidget(self.RecipeNameInput)

        self.QuantityWidget = QWidget(self.BackgroundWidget)
        self.QuantityWidget.setObjectName(u"QuantityWidget")
        self.QuantityWidget.setGeometry(QRect(620, 0, 181, 41))
        self.QuantityWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_20 = QHBoxLayout(self.QuantityWidget)
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.QuantityText = QLabel(self.QuantityWidget)
        self.QuantityText.setObjectName(u"QuantityText")
        self.QuantityText.setFont(font9)

        self.horizontalLayout_20.addWidget(self.QuantityText)

        self.QuantityNumberInput = QLabel(self.QuantityWidget)
        self.QuantityNumberInput.setObjectName(u"QuantityNumberInput")

        self.horizontalLayout_20.addWidget(self.QuantityNumberInput)

        self.WorkerNameWidget = QWidget(self.BackgroundWidget)
        self.WorkerNameWidget.setObjectName(u"WorkerNameWidget")
        self.WorkerNameWidget.setGeometry(QRect(620, 30, 171, 41))
        self.WorkerNameWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_21 = QHBoxLayout(self.WorkerNameWidget)
        self.horizontalLayout_21.setObjectName(u"horizontalLayout_21")
        self.WorkerNameText = QLabel(self.WorkerNameWidget)
        self.WorkerNameText.setObjectName(u"WorkerNameText")
        self.WorkerNameText.setFont(font9)

        self.horizontalLayout_21.addWidget(self.WorkerNameText)

        self.WorkerNameInput = QLabel(self.WorkerNameWidget)
        self.WorkerNameInput.setObjectName(u"WorkerNameInput")

        self.horizontalLayout_21.addWidget(self.WorkerNameInput)

        self.CartHeightWidget = QWidget(self.BackgroundWidget)
        self.CartHeightWidget.setObjectName(u"CartHeightWidget")
        self.CartHeightWidget.setGeometry(QRect(240, 30, 221, 41))
        self.CartHeightWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_23 = QHBoxLayout(self.CartHeightWidget)
        self.horizontalLayout_23.setObjectName(u"horizontalLayout_23")
        self.CartHeightText = QLabel(self.CartHeightWidget)
        self.CartHeightText.setObjectName(u"CartHeightText")
        self.CartHeightText.setFont(font9)

        self.horizontalLayout_23.addWidget(self.CartHeightText)

        self.CartHeightInput = QLabel(self.CartHeightWidget)
        self.CartHeightInput.setObjectName(u"CartHeightInput")

        self.horizontalLayout_23.addWidget(self.CartHeightInput)

        self.CartDepthWidget = QWidget(self.BackgroundWidget)
        self.CartDepthWidget.setObjectName(u"CartDepthWidget")
        self.CartDepthWidget.setGeometry(QRect(240, 0, 221, 41))
        self.CartDepthWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_24 = QHBoxLayout(self.CartDepthWidget)
        self.horizontalLayout_24.setObjectName(u"horizontalLayout_24")
        self.CartDepthText = QLabel(self.CartDepthWidget)
        self.CartDepthText.setObjectName(u"CartDepthText")
        self.CartDepthText.setFont(font9)

        self.horizontalLayout_24.addWidget(self.CartDepthText)

        self.CartDepthInput = QLabel(self.CartDepthWidget)
        self.CartDepthInput.setObjectName(u"CartDepthInput")

        self.horizontalLayout_24.addWidget(self.CartDepthInput)

        self.DateWidget = QWidget(self.BackgroundWidget)
        self.DateWidget.setObjectName(u"DateWidget")
        self.DateWidget.setGeometry(QRect(770, 0, 151, 41))
        self.DateWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_26 = QHBoxLayout(self.DateWidget)
        self.horizontalLayout_26.setObjectName(u"horizontalLayout_26")
        self.DateText = QLabel(self.DateWidget)
        self.DateText.setObjectName(u"DateText")
        self.DateText.setFont(font9)

        self.horizontalLayout_26.addWidget(self.DateText)

        self.DateInput = QLabel(self.DateWidget)
        self.DateInput.setObjectName(u"DateInput")

        self.horizontalLayout_26.addWidget(self.DateInput)

        self.ServoONOFFButton = QPushButton(self.BackgroundWidget)
        self.ServoONOFFButton.setObjectName(u"ServoONOFFButton")
        self.ServoONOFFButton.setGeometry(QRect(960, 0, 91, 61))
        self.ServoONOFFButton.setStyleSheet(u"QPushButton#ServoONOFFButton {\n"
"         border-radius: 8px;\n"
"}")
        self.ServoONOFFButton.setCheckable(True)
        self.AlarmButton = QPushButton(self.BackgroundWidget)
        self.AlarmButton.setObjectName(u"AlarmButton")
        self.AlarmButton.setGeometry(QRect(1060, 0, 91, 61))
        self.AlarmButton.setStyleSheet(u"            border-radius: 8px;\n"
"")
        self.AlarmButton.setIconSize(QSize(28, 28))
        self.ResetButton = QPushButton(self.BackgroundWidget)
        self.ResetButton.setObjectName(u"ResetButton")
        self.ResetButton.setGeometry(QRect(1160, 0, 90, 61))
        self.ResetButton.setStyleSheet(u"            border-radius: 8px;\n"
"")
        self.ParentStackedWidgetToChangeMenuOptions.raise_()
        self.Line.raise_()
        self.SystemSettingsButton.raise_()
        self.SignalLightsWidget.raise_()
        self.MenuButtons.raise_()
        self.WorkOrderWidget.raise_()
        self.RecipeWidget.raise_()
        self.QuantityWidget.raise_()
        self.WorkerNameWidget.raise_()
        self.CartHeightWidget.raise_()
        self.CartDepthWidget.raise_()
        self.DateWidget.raise_()
        self.DeltaLogo.raise_()
        self.ServoONOFFButton.raise_()
        self.AlarmButton.raise_()
        self.ResetButton.raise_()

        self.horizontalLayout_4.addWidget(self.BackgroundWidget)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(0)
        self.ActionButtons.setCurrentIndex(0)
        self.ComponentControlStackedWidget.setCurrentIndex(0)
        self.ChangeComponentControlStackedWidget.setCurrentIndex(0)
        self.MotorStackedWidget.setCurrentIndex(0)
        self.MiddleStackedWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.SystemSettingsButton.setText(QCoreApplication.translate("MainWindow", u"System\n"
"Settings", None))
        self.RedSignal.setText("")
        self.YellowSignal.setText("")
        self.GreenSignal.setText("")
        self.DeltaLogo.setText("")
        self.MainPageButton.setText(QCoreApplication.translate("MainWindow", u"Main Page", None))
        self.ComponentControlButton.setText(QCoreApplication.translate("MainWindow", u"Component\n"
"Control", None))
        self.ProductionRecordButton.setText(QCoreApplication.translate("MainWindow", u"Production\n"
"Record", None))
        self.LogsButton.setText(QCoreApplication.translate("MainWindow", u"Logs", None))
        self.VisionText.setText(QCoreApplication.translate("MainWindow", u"Vision", None))
        self.AutoButton.setText(QCoreApplication.translate("MainWindow", u"Auto", None))
        self.ManualButton.setText(QCoreApplication.translate("MainWindow", u"Manual", None))
        self.INITButton.setText(QCoreApplication.translate("MainWindow", u"INIT", None))
        self.RunButton.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.RoughAlignButton.setText(QCoreApplication.translate("MainWindow", u"Rough Align", None))
        self.PreciseAlignButton.setText(QCoreApplication.translate("MainWindow", u"Precise Align", None))
        self.PickButton.setText(QCoreApplication.translate("MainWindow", u"Pick", None))
        self.AssemblyButton.setText(QCoreApplication.translate("MainWindow", u"Assembly", None))
        self.AutoPauseButton.setText(QCoreApplication.translate("MainWindow", u"Pause", None))
        self.AutoStopButton.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.StartProcessText.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.StartCircle.setText("")
        self.ConnectProcessText.setText(QCoreApplication.translate("MainWindow", u"Connect", None))
        self.ConnectCircle.setText("")
        self.INITProcessText.setText(QCoreApplication.translate("MainWindow", u"INIT", None))
        self.INITCircle.setText("")
        self.IdleProcessText.setText(QCoreApplication.translate("MainWindow", u"Idle", None))
        self.IdleCircle.setText("")
        self.RoughAlignProcessText.setText(QCoreApplication.translate("MainWindow", u"Rough Align", None))
        self.RoughAlignCircle.setText("")
        self.PreciseAlignProcessText.setText(QCoreApplication.translate("MainWindow", u"Precise Align", None))
        self.PreciseAlignCircle.setText("")
        self.PickProcessText.setText(QCoreApplication.translate("MainWindow", u"Pick", None))
        self.PickCircle.setText("")
        self.AssemblyProcessText.setText(QCoreApplication.translate("MainWindow", u"Assembly", None))
        self.AssemblyCircle.setText("")
        self.ChooseMotor.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.ChooseVision.setText(QCoreApplication.translate("MainWindow", u"Vision", None))
        self.ChooseClipper.setText(QCoreApplication.translate("MainWindow", u"Clipper", None))
        self.ChooseForklift.setText(QCoreApplication.translate("MainWindow", u"Forklift", None))
        self.ChooseDIDO.setText(QCoreApplication.translate("MainWindow", u"DI/DO", None))
        self.MotorStartedButton.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.HamburgerMenu.setText("")
        self.ServoON.setText(QCoreApplication.translate("MainWindow", u"Servo ON", None))
        self.HomeMotor.setText(QCoreApplication.translate("MainWindow", u"Home", None))
        self.ServoOFF.setText(QCoreApplication.translate("MainWindow", u"Servo OFF", None))
        self.MotorConfigNextButton.setText(QCoreApplication.translate("MainWindow", u"next", None))
        self.YawPlusCP.setText("")
        self.ControlUpCP.setText("")
        self.ControlLeftCP.setText("")
        self.ControlDownCP.setText("")
        self.ControlRightCP.setText("")
        self.YawMinusCP.setText("")
        self.PauseMotor.setText(QCoreApplication.translate("MainWindow", u"Pause", None))
        self.StopMotor.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.MotorJogNextButton.setText(QCoreApplication.translate("MainWindow", u"next", None))
        self.MotorChooseDistance.setText("")
        self.MotorYAxisNextButton.setText(QCoreApplication.translate("MainWindow", u"next", None))
        self.HomeY.setText(QCoreApplication.translate("MainWindow", u"Home", None))
        self.ReadyY.setText(QCoreApplication.translate("MainWindow", u"Ready", None))
        self.AssemblyY.setText(QCoreApplication.translate("MainWindow", u"Assembly", None))
        self.SendYCommand.setText(QCoreApplication.translate("MainWindow", u"Send Y Command", None))
        self.InputMotorDistance.setText("")
        self.InputMotorSpeed.setText("")
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Distance", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Speed", None))
        self.VisionOne.setText(QCoreApplication.translate("MainWindow", u"Screw", None))
        self.VisionTwo.setText(QCoreApplication.translate("MainWindow", u"LShape", None))
        self.VisionThree.setText(QCoreApplication.translate("MainWindow", u"ICP Fit", None))
        self.VisionSendButton.setText(QCoreApplication.translate("MainWindow", u"Send", None))
        self.yVisionLabel.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.yVision.setText(QCoreApplication.translate("MainWindow", u"Y", None))
        self.xVisionLabel.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.Yaw.setText(QCoreApplication.translate("MainWindow", u"Yaw", None))
        self.xVision.setText(QCoreApplication.translate("MainWindow", u"X", None))
        self.zVision.setText(QCoreApplication.translate("MainWindow", u"Z", None))
        self.YawVisionLabel.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.zVisionLabel.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.OpenClipper.setText(QCoreApplication.translate("MainWindow", u"open", None))
        self.CloseClipper.setText(QCoreApplication.translate("MainWindow", u"close", None))
        self.StopClipper.setText(QCoreApplication.translate("MainWindow", u"stop", None))
        self.ResetClipper.setText(QCoreApplication.translate("MainWindow", u"reset", None))
        self.LiftUp.setText("")
        self.LowerDown.setText("")
        self.FastForktLiftButton.setText(QCoreApplication.translate("MainWindow", u"Fast", None))
        self.SlowLiftButton.setText(QCoreApplication.translate("MainWindow", u"Slow", None))
        self.StopForkliftButton.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Speed", None))
        self.SendForkliftCommand.setText(QCoreApplication.translate("MainWindow", u"Go to distance", None))
        self.MediumLiftButton.setText(QCoreApplication.translate("MainWindow", u"Medium", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Distance (manual)", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"+10 mm", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"-10 mm", None))
        self.InputDistance.setText("")
        self.MotorOption.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.VisionOption.setText(QCoreApplication.translate("MainWindow", u"Vision", None))
        self.ClipperOption.setText(QCoreApplication.translate("MainWindow", u"Clipper", None))
        self.ForkliftOption.setText(QCoreApplication.translate("MainWindow", u"Forklift", None))
        self.DIDOOption.setText(QCoreApplication.translate("MainWindow", u"DI/DO", None))
        self.VisionTextInComponentControl.setText(QCoreApplication.translate("MainWindow", u"Vision", None))
        self.CartesianPoseText_2.setText(QCoreApplication.translate("MainWindow", u"Cartesian Pose", None))
        self.z_2.setText(QCoreApplication.translate("MainWindow", u"z (mm)", None))
        self.zPose.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.y_2.setText(QCoreApplication.translate("MainWindow", u"y (mm)", None))
        self.yPos.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.yaw_2.setText(QCoreApplication.translate("MainWindow", u"yaw (deg)", None))
        self.yawPos.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.x_2.setText(QCoreApplication.translate("MainWindow", u"x (mm)", None))
        self.xPos.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.MotorInfoText_2.setText(QCoreApplication.translate("MainWindow", u"Motor Info", None))
        self.Position_2.setText(QCoreApplication.translate("MainWindow", u"Position (mm)", None))
        self.MotorM1.setText(QCoreApplication.translate("MainWindow", u"m1", None))
        self.MotorM2.setText(QCoreApplication.translate("MainWindow", u"m2", None))
        self.MotorM3.setText(QCoreApplication.translate("MainWindow", u"m3", None))
        self.LaserInfoText_2.setText(QCoreApplication.translate("MainWindow", u"Height Info", None))
        self.h1_2.setText(QCoreApplication.translate("MainWindow", u"Height Cmd (mm)", None))
        self.HeightCommand.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.d2_2.setText(QCoreApplication.translate("MainWindow", u"Curr. Height (mm)", None))
        self.currentHeight.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.h1_5.setText(QCoreApplication.translate("MainWindow", u"Right (mm)", None))
        self.RightDepthText.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.d2_5.setText(QCoreApplication.translate("MainWindow", u"Left (mm)", None))
        self.LeftDepthText.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.DepthInfoText.setText(QCoreApplication.translate("MainWindow", u"Depth Info", None))
        self.DO23.setText(QCoreApplication.translate("MainWindow", u"23", None))
        self.DO5.setText(QCoreApplication.translate("MainWindow", u"5", None))
        self.DO14.setText(QCoreApplication.translate("MainWindow", u"14", None))
        self.DO9.setText(QCoreApplication.translate("MainWindow", u"9", None))
        self.DO38.setText(QCoreApplication.translate("MainWindow", u"38", None))
        self.DO22.setText(QCoreApplication.translate("MainWindow", u"22", None))
        self.DO48.setText(QCoreApplication.translate("MainWindow", u"48", None))
        self.DO46.setText(QCoreApplication.translate("MainWindow", u"46", None))
        self.DO28.setText(QCoreApplication.translate("MainWindow", u"28", None))
        self.DO2.setText(QCoreApplication.translate("MainWindow", u"2", None))
        self.DO41.setText(QCoreApplication.translate("MainWindow", u"41", None))
        self.DO31.setText(QCoreApplication.translate("MainWindow", u"31", None))
        self.DO11.setText(QCoreApplication.translate("MainWindow", u"11", None))
        self.DO17.setText(QCoreApplication.translate("MainWindow", u"17", None))
        self.DO4.setText(QCoreApplication.translate("MainWindow", u"4", None))
        self.DO20.setText(QCoreApplication.translate("MainWindow", u"20", None))
        self.DO15.setText(QCoreApplication.translate("MainWindow", u"15", None))
        self.DO19.setText(QCoreApplication.translate("MainWindow", u"19", None))
        self.DO21.setText(QCoreApplication.translate("MainWindow", u"21", None))
        self.DO18.setText(QCoreApplication.translate("MainWindow", u"18", None))
        self.DO36.setText(QCoreApplication.translate("MainWindow", u"36", None))
        self.DO8.setText(QCoreApplication.translate("MainWindow", u"8", None))
        self.DO42.setText(QCoreApplication.translate("MainWindow", u"42", None))
        self.DO26.setText(QCoreApplication.translate("MainWindow", u"26", None))
        self.DO1.setText(QCoreApplication.translate("MainWindow", u"1", None))
        self.DO44.setText(QCoreApplication.translate("MainWindow", u"44", None))
        self.DO16.setText(QCoreApplication.translate("MainWindow", u"16", None))
        self.DO32.setText(QCoreApplication.translate("MainWindow", u"32", None))
        self.DO34.setText(QCoreApplication.translate("MainWindow", u"34", None))
        self.DO10.setText(QCoreApplication.translate("MainWindow", u"10", None))
        self.DO29.setText(QCoreApplication.translate("MainWindow", u"29", None))
        self.DO35.setText(QCoreApplication.translate("MainWindow", u"35", None))
        self.DO37.setText(QCoreApplication.translate("MainWindow", u"37", None))
        self.DO12.setText(QCoreApplication.translate("MainWindow", u"12", None))
        self.DO43.setText(QCoreApplication.translate("MainWindow", u"43", None))
        self.DO47.setText(QCoreApplication.translate("MainWindow", u"47", None))
        self.DO30.setText(QCoreApplication.translate("MainWindow", u"30", None))
        self.DO24.setText(QCoreApplication.translate("MainWindow", u"24", None))
        self.DO25.setText(QCoreApplication.translate("MainWindow", u"25", None))
        self.DO6.setText(QCoreApplication.translate("MainWindow", u"6", None))
        self.DO13.setText(QCoreApplication.translate("MainWindow", u"13", None))
        self.DO7.setText(QCoreApplication.translate("MainWindow", u"7", None))
        self.DO33.setText(QCoreApplication.translate("MainWindow", u"33", None))
        self.DO3.setText(QCoreApplication.translate("MainWindow", u"3", None))
        self.DO27.setText(QCoreApplication.translate("MainWindow", u"27", None))
        self.DO40.setText(QCoreApplication.translate("MainWindow", u"40", None))
        self.DO39.setText(QCoreApplication.translate("MainWindow", u"39", None))
        self.DO45.setText(QCoreApplication.translate("MainWindow", u"45", None))
        self.DI8.setText(QCoreApplication.translate("MainWindow", u"8", None))
        self.DI3.setText(QCoreApplication.translate("MainWindow", u"3", None))
        self.DI12.setText(QCoreApplication.translate("MainWindow", u"12", None))
        self.DI7.setText(QCoreApplication.translate("MainWindow", u"7", None))
        self.DI11.setText(QCoreApplication.translate("MainWindow", u"11", None))
        self.DI16.setText(QCoreApplication.translate("MainWindow", u"16", None))
        self.DI9.setText(QCoreApplication.translate("MainWindow", u"9", None))
        self.DI4.setText(QCoreApplication.translate("MainWindow", u"4", None))
        self.DI2.setText(QCoreApplication.translate("MainWindow", u"2", None))
        self.DI14.setText(QCoreApplication.translate("MainWindow", u"14", None))
        self.DI15.setText(QCoreApplication.translate("MainWindow", u"15", None))
        self.DI6.setText(QCoreApplication.translate("MainWindow", u"6", None))
        self.DI1.setText(QCoreApplication.translate("MainWindow", u"1", None))
        self.DI10.setText(QCoreApplication.translate("MainWindow", u"10", None))
        self.DI13.setText(QCoreApplication.translate("MainWindow", u"13", None))
        self.DI5.setText(QCoreApplication.translate("MainWindow", u"5", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"DI", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"DO", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Production Record", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Logs", None))
        self.ChangeLanguageText.setText(QCoreApplication.translate("MainWindow", u"Change Language", None))
        self.FontSizeText.setText(QCoreApplication.translate("MainWindow", u"Font Size", None))
        self.ThemeText.setText(QCoreApplication.translate("MainWindow", u"Theme", None))
        self.SaveChangesButton.setText(QCoreApplication.translate("MainWindow", u"Save Changes", None))
        self.EnglishLanguageButton.setText(QCoreApplication.translate("MainWindow", u"English", None))
        self.ChineseLanguageButton.setText(QCoreApplication.translate("MainWindow", u"Chinese", None))
        self.ThaiLanguageButton.setText(QCoreApplication.translate("MainWindow", u"Thai", None))
        self.BigFontSizeButton.setText(QCoreApplication.translate("MainWindow", u"Big", None))
        self.NormalFontSizeButton.setText(QCoreApplication.translate("MainWindow", u"Normal", None))
        self.SmallFontSizeButton.setText(QCoreApplication.translate("MainWindow", u"Small", None))
        self.DarkThemeButton.setText(QCoreApplication.translate("MainWindow", u"Dark", None))
        self.LightThemeButton.setText(QCoreApplication.translate("MainWindow", u"Light", None))
        self.WorkOrderText.setText(QCoreApplication.translate("MainWindow", u"\u5de5\u55ae:", None))
        self.WorkOrderNumberInput.setText(QCoreApplication.translate("MainWindow", u"12345678", None))
        self.RecipeText.setText(QCoreApplication.translate("MainWindow", u"Recipe:", None))
        self.RecipeNameInput.setText(QCoreApplication.translate("MainWindow", u"Battery ASSY", None))
        self.QuantityText.setText(QCoreApplication.translate("MainWindow", u"Quantity:", None))
        self.QuantityNumberInput.setText(QCoreApplication.translate("MainWindow", u"200", None))
        self.WorkerNameText.setText(QCoreApplication.translate("MainWindow", u"\u7d44\u88dd\u4eba\u54e1:", None))
        self.WorkerNameInput.setText(QCoreApplication.translate("MainWindow", u"Carlos", None))
        self.CartHeightText.setText(QCoreApplication.translate("MainWindow", u"Cart Height:", None))
        self.CartHeightInput.setText(QCoreApplication.translate("MainWindow", u"153.3 mm", None))
        self.CartDepthText.setText(QCoreApplication.translate("MainWindow", u"Cart Depth:", None))
        self.CartDepthInput.setText(QCoreApplication.translate("MainWindow", u"200.1 mm", None))
        self.DateText.setText(QCoreApplication.translate("MainWindow", u"Date:", None))
        self.DateInput.setText(QCoreApplication.translate("MainWindow", u"7/25/2025", None))
        self.ServoONOFFButton.setText(QCoreApplication.translate("MainWindow", u"Servo OFF", None))
        self.AlarmButton.setText(QCoreApplication.translate("MainWindow", u"Alarm: 0", None))
        self.ResetButton.setText(QCoreApplication.translate("MainWindow", u"Reset", None))
    # retranslateUi

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
    QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QSizePolicy, QSlider, QStackedWidget, QVBoxLayout,
    QWidget)

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
"")
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
        self.GreenSignal.setStyleSheet(u"background-color: #6FCF53; border-radius: 10px;")

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
        self.ParentStackedWidgetToChangeMenuOptions.setGeometry(QRect(240, 80, 1021, 691))
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

        self.AutoManualButtons = QWidget(self.MainPagePage)
        self.AutoManualButtons.setObjectName(u"AutoManualButtons")
        self.AutoManualButtons.setGeometry(QRect(720, 0, 291, 101))
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
        self.AutoButton.setFont(font)
        self.AutoButton.setStyleSheet(u"")
        self.AutoButton.setCheckable(True)
        self.AutoButton.setChecked(True)

        self.horizontalLayout_5.addWidget(self.AutoButton)

        self.HamburgerMenuMainPage = QPushButton(self.AutoManualButtons)
        self.HamburgerMenuMainPage.setObjectName(u"HamburgerMenuMainPage")
        sizePolicy.setHeightForWidth(self.HamburgerMenuMainPage.sizePolicy().hasHeightForWidth())
        self.HamburgerMenuMainPage.setSizePolicy(sizePolicy)
        self.HamburgerMenuMainPage.setStyleSheet(u"")
        icon5 = QIcon()
        icon5.addFile(u":/icons/white/menu.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.HamburgerMenuMainPage.setIcon(icon5)
        self.HamburgerMenuMainPage.setIconSize(QSize(30, 30))

        self.horizontalLayout_5.addWidget(self.HamburgerMenuMainPage)

        self.ActionButtons = QStackedWidget(self.MainPagePage)
        self.ActionButtons.setObjectName(u"ActionButtons")
        self.ActionButtons.setGeometry(QRect(710, 100, 301, 591))
        self.ActionButtons.setStyleSheet(u"")
        self.ActionButtonsPage = QWidget()
        self.ActionButtonsPage.setObjectName(u"ActionButtonsPage")
        self.verticalLayout_12 = QVBoxLayout(self.ActionButtonsPage)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.InitButton = QPushButton(self.ActionButtonsPage)
        self.InitButton.setObjectName(u"InitButton")
        sizePolicy.setHeightForWidth(self.InitButton.sizePolicy().hasHeightForWidth())
        self.InitButton.setSizePolicy(sizePolicy)
        self.InitButton.setFont(font)
        self.InitButton.setStyleSheet(u"")
        self.InitButton.setCheckable(False)

        self.verticalLayout_12.addWidget(self.InitButton)

        self.RunButton = QPushButton(self.ActionButtonsPage)
        self.RunButton.setObjectName(u"RunButton")
        sizePolicy.setHeightForWidth(self.RunButton.sizePolicy().hasHeightForWidth())
        self.RunButton.setSizePolicy(sizePolicy)
        self.RunButton.setFont(font)
        self.RunButton.setStyleSheet(u"")

        self.verticalLayout_12.addWidget(self.RunButton)

        self.StopButton = QPushButton(self.ActionButtonsPage)
        self.StopButton.setObjectName(u"StopButton")
        sizePolicy.setHeightForWidth(self.StopButton.sizePolicy().hasHeightForWidth())
        self.StopButton.setSizePolicy(sizePolicy)
        self.StopButton.setFont(font)
        self.StopButton.setStyleSheet(u"")

        self.verticalLayout_12.addWidget(self.StopButton)

        self.AutoResetButton = QPushButton(self.ActionButtonsPage)
        self.AutoResetButton.setObjectName(u"AutoResetButton")
        sizePolicy.setHeightForWidth(self.AutoResetButton.sizePolicy().hasHeightForWidth())
        self.AutoResetButton.setSizePolicy(sizePolicy)
        self.AutoResetButton.setFont(font)
        self.AutoResetButton.setStyleSheet(u"")

        self.verticalLayout_12.addWidget(self.AutoResetButton)

        self.ActionButtons.addWidget(self.ActionButtonsPage)
        self.RoughAlignPage = QWidget()
        self.RoughAlignPage.setObjectName(u"RoughAlignPage")
        self.RoughAlignPage.setStyleSheet(u"")
        self.label_9 = QLabel(self.RoughAlignPage)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setGeometry(QRect(120, 140, 67, 17))
        self.ActionButtons.addWidget(self.RoughAlignPage)
        self.PreciseAlignPage = QWidget()
        self.PreciseAlignPage.setObjectName(u"PreciseAlignPage")
        self.label_8 = QLabel(self.PreciseAlignPage)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(130, 140, 67, 17))
        self.ActionButtons.addWidget(self.PreciseAlignPage)
        self.PickPage = QWidget()
        self.PickPage.setObjectName(u"PickPage")
        self.label_10 = QLabel(self.PickPage)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setGeometry(QRect(120, 140, 67, 17))
        self.ActionButtons.addWidget(self.PickPage)
        self.page = QWidget()
        self.page.setObjectName(u"page")
        self.label_11 = QLabel(self.page)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setGeometry(QRect(120, 130, 67, 17))
        self.ActionButtons.addWidget(self.page)
        self.ProcessAndInfoStackedWidget = QStackedWidget(self.MainPagePage)
        self.ProcessAndInfoStackedWidget.setObjectName(u"ProcessAndInfoStackedWidget")
        self.ProcessAndInfoStackedWidget.setGeometry(QRect(0, 500, 701, 191))
        self.ProcessAndInfoStackedWidget.setStyleSheet(u" background-color: #000000; color: white;")
        self.AutoProcessPage = QWidget()
        self.AutoProcessPage.setObjectName(u"AutoProcessPage")
        self.AutoProcessPage.setStyleSheet(u"")
        self.horizontalLayout_6 = QHBoxLayout(self.AutoProcessPage)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.Timeline = QFrame(self.AutoProcessPage)
        self.Timeline.setObjectName(u"Timeline")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.Timeline.sizePolicy().hasHeightForWidth())
        self.Timeline.setSizePolicy(sizePolicy1)
        self.Timeline.setStyleSheet(u"")
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
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.StartCircle.sizePolicy().hasHeightForWidth())
        self.StartCircle.setSizePolicy(sizePolicy2)
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
        sizePolicy2.setHeightForWidth(self.ConnectCircle.sizePolicy().hasHeightForWidth())
        self.ConnectCircle.setSizePolicy(sizePolicy2)
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
        sizePolicy2.setHeightForWidth(self.INITCircle.sizePolicy().hasHeightForWidth())
        self.INITCircle.setSizePolicy(sizePolicy2)
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
        sizePolicy2.setHeightForWidth(self.IdleCircle.sizePolicy().hasHeightForWidth())
        self.IdleCircle.setSizePolicy(sizePolicy2)
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
        sizePolicy2.setHeightForWidth(self.RoughAlignCircle.sizePolicy().hasHeightForWidth())
        self.RoughAlignCircle.setSizePolicy(sizePolicy2)
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
        sizePolicy2.setHeightForWidth(self.PreciseAlignCircle.sizePolicy().hasHeightForWidth())
        self.PreciseAlignCircle.setSizePolicy(sizePolicy2)
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
        sizePolicy2.setHeightForWidth(self.PickCircle.sizePolicy().hasHeightForWidth())
        self.PickCircle.setSizePolicy(sizePolicy2)
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
        sizePolicy2.setHeightForWidth(self.AssemblyCircle.sizePolicy().hasHeightForWidth())
        self.AssemblyCircle.setSizePolicy(sizePolicy2)
        self.AssemblyCircle.setPixmap(QPixmap(u":/icons/white/timeline-circle.svg"))
        self.AssemblyCircle.setScaledContents(True)
        self.AssemblyCircle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_4.addWidget(self.AssemblyCircle, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.horizontalLayout_3.addWidget(self.AssemblyWidget, 0, Qt.AlignmentFlag.AlignVCenter)


        self.horizontalLayout_6.addWidget(self.Timeline)

        self.ProcessAndInfoStackedWidget.addWidget(self.AutoProcessPage)
        self.InformationPage = QWidget()
        self.InformationPage.setObjectName(u"InformationPage")
        self.InformationPage.setStyleSheet(u"")
        self.horizontalLayout_7 = QHBoxLayout(self.InformationPage)
        self.horizontalLayout_7.setSpacing(6)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.Info = QFrame(self.InformationPage)
        self.Info.setObjectName(u"Info")
        self.Info.setFrameShape(QFrame.Shape.StyledPanel)
        self.Info.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_10 = QHBoxLayout(self.Info)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.horizontalLayout_10.setContentsMargins(-1, 0, -1, 9)
        self.CartesianPoseWidget = QWidget(self.Info)
        self.CartesianPoseWidget.setObjectName(u"CartesianPoseWidget")
        sizePolicy1.setHeightForWidth(self.CartesianPoseWidget.sizePolicy().hasHeightForWidth())
        self.CartesianPoseWidget.setSizePolicy(sizePolicy1)
        self.CartesianPoseWidget.setMinimumSize(QSize(0, 0))
        self.gridLayout = QGridLayout(self.CartesianPoseWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(-1, 6, -1, -1)
        self.yPosFrame = QFrame(self.CartesianPoseWidget)
        self.yPosFrame.setObjectName(u"yPosFrame")
        self.yPosFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.yPosFrame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_9 = QHBoxLayout(self.yPosFrame)
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.horizontalLayout_9.setContentsMargins(-1, -1, -1, 3)
        self.y = QLabel(self.yPosFrame)
        self.y.setObjectName(u"y")
        font2 = QFont()
        font2.setPointSize(9)
        self.y.setFont(font2)

        self.horizontalLayout_9.addWidget(self.y, 0, Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.yText = QLabel(self.yPosFrame)
        self.yText.setObjectName(u"yText")

        self.horizontalLayout_9.addWidget(self.yText, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.gridLayout.addWidget(self.yPosFrame, 1, 1, 1, 1)

        self.zPosFrame = QFrame(self.CartesianPoseWidget)
        self.zPosFrame.setObjectName(u"zPosFrame")
        self.zPosFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.zPosFrame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_11 = QHBoxLayout(self.zPosFrame)
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.horizontalLayout_11.setContentsMargins(-1, -1, -1, 14)
        self.z = QLabel(self.zPosFrame)
        self.z.setObjectName(u"z")
        self.z.setFont(font2)

        self.horizontalLayout_11.addWidget(self.z, 0, Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.zText = QLabel(self.zPosFrame)
        self.zText.setObjectName(u"zText")

        self.horizontalLayout_11.addWidget(self.zText, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.gridLayout.addWidget(self.zPosFrame, 2, 0, 1, 1)

        self.yawPosFrame = QFrame(self.CartesianPoseWidget)
        self.yawPosFrame.setObjectName(u"yawPosFrame")
        self.yawPosFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.yawPosFrame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_12 = QHBoxLayout(self.yawPosFrame)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.horizontalLayout_12.setContentsMargins(-1, -1, -1, 14)
        self.yaw = QLabel(self.yawPosFrame)
        self.yaw.setObjectName(u"yaw")
        self.yaw.setFont(font2)

        self.horizontalLayout_12.addWidget(self.yaw, 0, Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.yawText = QLabel(self.yawPosFrame)
        self.yawText.setObjectName(u"yawText")

        self.horizontalLayout_12.addWidget(self.yawText, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.gridLayout.addWidget(self.yawPosFrame, 2, 1, 1, 1)

        self.CartesianPoseText = QLabel(self.CartesianPoseWidget)
        self.CartesianPoseText.setObjectName(u"CartesianPoseText")
        font3 = QFont()
        font3.setPointSize(10)
        self.CartesianPoseText.setFont(font3)
        self.CartesianPoseText.setStyleSheet(u"border: 2px solid #FFFFFF;  /* white solid border */\n"
"border-radius: 6px;")

        self.gridLayout.addWidget(self.CartesianPoseText, 0, 0, 1, 2, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignTop)

        self.xPosFrame = QFrame(self.CartesianPoseWidget)
        self.xPosFrame.setObjectName(u"xPosFrame")
        self.xPosFrame.setFont(font2)
        self.xPosFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.xPosFrame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_8 = QHBoxLayout(self.xPosFrame)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(9, -1, -1, 3)
        self.x = QLabel(self.xPosFrame)
        self.x.setObjectName(u"x")
        self.x.setFont(font2)

        self.horizontalLayout_8.addWidget(self.x, 0, Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.xText = QLabel(self.xPosFrame)
        self.xText.setObjectName(u"xText")

        self.horizontalLayout_8.addWidget(self.xText, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.gridLayout.addWidget(self.xPosFrame, 1, 0, 1, 1)


        self.horizontalLayout_10.addWidget(self.CartesianPoseWidget)

        self.MotorInfoWidget = QWidget(self.Info)
        self.MotorInfoWidget.setObjectName(u"MotorInfoWidget")
        self.MotorInfoWidget.setStyleSheet(u"")
        self.gridLayout_2 = QGridLayout(self.MotorInfoWidget)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.gridLayout_2.setContentsMargins(-1, 6, -1, -1)
        self.PositionFrame = QFrame(self.MotorInfoWidget)
        self.PositionFrame.setObjectName(u"PositionFrame")
        self.PositionFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.PositionFrame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_13 = QHBoxLayout(self.PositionFrame)
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.Position = QLabel(self.PositionFrame)
        self.Position.setObjectName(u"Position")

        self.horizontalLayout_13.addWidget(self.Position)

        self.PositionText = QLabel(self.PositionFrame)
        self.PositionText.setObjectName(u"PositionText")

        self.horizontalLayout_13.addWidget(self.PositionText)


        self.gridLayout_2.addWidget(self.PositionFrame, 2, 0, 1, 1)

        self.MotorInfoText = QLabel(self.MotorInfoWidget)
        self.MotorInfoText.setObjectName(u"MotorInfoText")
        self.MotorInfoText.setFont(font3)
        self.MotorInfoText.setStyleSheet(u"border: 2px solid #FFFFFF;  /* white solid border */\n"
"border-radius: 6px;")

        self.gridLayout_2.addWidget(self.MotorInfoText, 0, 0, 1, 1, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignTop)

        self.CurrentFrame = QFrame(self.MotorInfoWidget)
        self.CurrentFrame.setObjectName(u"CurrentFrame")
        self.CurrentFrame.setFrameShape(QFrame.Shape.StyledPanel)
        self.CurrentFrame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_14 = QHBoxLayout(self.CurrentFrame)
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.Current = QLabel(self.CurrentFrame)
        self.Current.setObjectName(u"Current")

        self.horizontalLayout_14.addWidget(self.Current)

        self.CurrentText = QLabel(self.CurrentFrame)
        self.CurrentText.setObjectName(u"CurrentText")

        self.horizontalLayout_14.addWidget(self.CurrentText)


        self.gridLayout_2.addWidget(self.CurrentFrame, 3, 0, 1, 1)


        self.horizontalLayout_10.addWidget(self.MotorInfoWidget)

        self.RecordData = QWidget(self.Info)
        self.RecordData.setObjectName(u"RecordData")
        self.gridLayout_3 = QGridLayout(self.RecordData)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.gridLayout_3.setContentsMargins(-1, 6, -1, -1)
        self.frame_11 = QFrame(self.RecordData)
        self.frame_11.setObjectName(u"frame_11")
        self.frame_11.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_11.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_13 = QVBoxLayout(self.frame_11)
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.verticalLayout_13.setContentsMargins(9, 9, -1, 0)
        self.d1 = QLabel(self.frame_11)
        self.d1.setObjectName(u"d1")

        self.verticalLayout_13.addWidget(self.d1)


        self.gridLayout_3.addWidget(self.frame_11, 1, 0, 1, 1)

        self.LaserInfoText = QLabel(self.RecordData)
        self.LaserInfoText.setObjectName(u"LaserInfoText")
        self.LaserInfoText.setFont(font3)
        self.LaserInfoText.setStyleSheet(u"border: 2px solid #FFFFFF;  /* white solid border */\n"
"border-radius: 6px;")

        self.gridLayout_3.addWidget(self.LaserInfoText, 0, 0, 1, 1, Qt.AlignmentFlag.AlignHCenter)

        self.frame_6 = QFrame(self.RecordData)
        self.frame_6.setObjectName(u"frame_6")
        self.frame_6.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_6.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_22 = QVBoxLayout(self.frame_6)
        self.verticalLayout_22.setObjectName(u"verticalLayout_22")
        self.verticalLayout_22.setContentsMargins(9, -1, -1, 0)
        self.d2 = QLabel(self.frame_6)
        self.d2.setObjectName(u"d2")

        self.verticalLayout_22.addWidget(self.d2)


        self.gridLayout_3.addWidget(self.frame_6, 3, 0, 1, 1)

        self.frame_5 = QFrame(self.RecordData)
        self.frame_5.setObjectName(u"frame_5")
        self.frame_5.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_5.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_14 = QVBoxLayout(self.frame_5)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.verticalLayout_14.setContentsMargins(9, -1, -1, 0)
        self.h1 = QLabel(self.frame_5)
        self.h1.setObjectName(u"h1")

        self.verticalLayout_14.addWidget(self.h1)


        self.gridLayout_3.addWidget(self.frame_5, 4, 0, 1, 1)


        self.horizontalLayout_10.addWidget(self.RecordData)

        self.RecordDataButtonContainer = QWidget(self.Info)
        self.RecordDataButtonContainer.setObjectName(u"RecordDataButtonContainer")
        sizePolicy1.setHeightForWidth(self.RecordDataButtonContainer.sizePolicy().hasHeightForWidth())
        self.RecordDataButtonContainer.setSizePolicy(sizePolicy1)
        self.verticalLayout_16 = QVBoxLayout(self.RecordDataButtonContainer)
        self.verticalLayout_16.setObjectName(u"verticalLayout_16")
        self.verticalLayout_16.setContentsMargins(9, 9, -1, -1)
        self.RecordDataButton = QPushButton(self.RecordDataButtonContainer)
        self.RecordDataButton.setObjectName(u"RecordDataButton")
        sizePolicy1.setHeightForWidth(self.RecordDataButton.sizePolicy().hasHeightForWidth())
        self.RecordDataButton.setSizePolicy(sizePolicy1)
        self.RecordDataButton.setFont(font)
        self.RecordDataButton.setStyleSheet(u"QPushButton {\n"
"    background-color: qlineargradient(\n"
"        x1:0, y1:0, x2:0, y2:1,\n"
"        stop:0 #1a1a1a, stop:1 #000000\n"
"    );\n"
"    color: white;\n"
"    border: 1px solid #555;\n"
"    border-radius: 4px;\n"
"    padding: 16px 24px;\n"
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
"	background-color: #0B76A0;\n"
"    border: 1px solid #555;\n"
"    border-radius: 4px;\n"
"    padding: 16px 24px;\n"
"}\n"
"")

        self.verticalLayout_16.addWidget(self.RecordDataButton, 0, Qt.AlignmentFlag.AlignVCenter)


        self.horizontalLayout_10.addWidget(self.RecordDataButtonContainer)


        self.horizontalLayout_7.addWidget(self.Info)

        self.ProcessAndInfoStackedWidget.addWidget(self.InformationPage)
        self.HamburgerMenuInMainPageOptions = QWidget(self.MainPagePage)
        self.HamburgerMenuInMainPageOptions.setObjectName(u"HamburgerMenuInMainPageOptions")
        self.HamburgerMenuInMainPageOptions.setGeometry(QRect(740, 110, 251, 481))
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.HamburgerMenuInMainPageOptions.sizePolicy().hasHeightForWidth())
        self.HamburgerMenuInMainPageOptions.setSizePolicy(sizePolicy3)
        self.gridLayout_7 = QGridLayout(self.HamburgerMenuInMainPageOptions)
        self.gridLayout_7.setObjectName(u"gridLayout_7")
        self.AutoButtonOption = QPushButton(self.HamburgerMenuInMainPageOptions)
        self.AutoButtonOption.setObjectName(u"AutoButtonOption")
        sizePolicy2.setHeightForWidth(self.AutoButtonOption.sizePolicy().hasHeightForWidth())
        self.AutoButtonOption.setSizePolicy(sizePolicy2)
        self.AutoButtonOption.setMinimumSize(QSize(200, 80))
        self.AutoButtonOption.setFont(font)
        self.AutoButtonOption.setStyleSheet(u"")
        self.AutoButtonOption.setCheckable(False)

        self.gridLayout_7.addWidget(self.AutoButtonOption, 0, 0, 1, 1)

        self.ManualButton = QPushButton(self.HamburgerMenuInMainPageOptions)
        self.ManualButton.setObjectName(u"ManualButton")
        self.ManualButton.setEnabled(True)
        sizePolicy2.setHeightForWidth(self.ManualButton.sizePolicy().hasHeightForWidth())
        self.ManualButton.setSizePolicy(sizePolicy2)
        self.ManualButton.setMinimumSize(QSize(200, 80))
        self.ManualButton.setFont(font)
        self.ManualButton.setStyleSheet(u"")
        self.ManualButton.setCheckable(False)

        self.gridLayout_7.addWidget(self.ManualButton, 1, 0, 1, 1)

        self.ManualOptionsWidget = QWidget(self.HamburgerMenuInMainPageOptions)
        self.ManualOptionsWidget.setObjectName(u"ManualOptionsWidget")
        sizePolicy.setHeightForWidth(self.ManualOptionsWidget.sizePolicy().hasHeightForWidth())
        self.ManualOptionsWidget.setSizePolicy(sizePolicy)
        self.ManualOptionsWidget.setStyleSheet(u"padding: 20px;")
        self.verticalLayout_24 = QVBoxLayout(self.ManualOptionsWidget)
        self.verticalLayout_24.setObjectName(u"verticalLayout_24")
        self.verticalLayout_24.setContentsMargins(9, -1, -1, -1)
        self.RoughAlignButton = QPushButton(self.ManualOptionsWidget)
        self.RoughAlignButton.setObjectName(u"RoughAlignButton")
        sizePolicy1.setHeightForWidth(self.RoughAlignButton.sizePolicy().hasHeightForWidth())
        self.RoughAlignButton.setSizePolicy(sizePolicy1)
        self.RoughAlignButton.setFont(font)
        self.RoughAlignButton.setStyleSheet(u"")
        self.RoughAlignButton.setCheckable(False)

        self.verticalLayout_24.addWidget(self.RoughAlignButton)

        self.PreciseAlignButton = QPushButton(self.ManualOptionsWidget)
        self.PreciseAlignButton.setObjectName(u"PreciseAlignButton")
        sizePolicy1.setHeightForWidth(self.PreciseAlignButton.sizePolicy().hasHeightForWidth())
        self.PreciseAlignButton.setSizePolicy(sizePolicy1)
        self.PreciseAlignButton.setFont(font)
        self.PreciseAlignButton.setStyleSheet(u"")
        self.PreciseAlignButton.setCheckable(False)

        self.verticalLayout_24.addWidget(self.PreciseAlignButton)

        self.PickButton = QPushButton(self.ManualOptionsWidget)
        self.PickButton.setObjectName(u"PickButton")
        sizePolicy1.setHeightForWidth(self.PickButton.sizePolicy().hasHeightForWidth())
        self.PickButton.setSizePolicy(sizePolicy1)
        self.PickButton.setFont(font)
        self.PickButton.setStyleSheet(u"")
        self.PickButton.setCheckable(False)

        self.verticalLayout_24.addWidget(self.PickButton)

        self.AssemblyButton = QPushButton(self.ManualOptionsWidget)
        self.AssemblyButton.setObjectName(u"AssemblyButton")
        sizePolicy1.setHeightForWidth(self.AssemblyButton.sizePolicy().hasHeightForWidth())
        self.AssemblyButton.setSizePolicy(sizePolicy1)
        self.AssemblyButton.setFont(font)
        self.AssemblyButton.setStyleSheet(u"")
        self.AssemblyButton.setCheckable(False)

        self.verticalLayout_24.addWidget(self.AssemblyButton)


        self.gridLayout_7.addWidget(self.ManualOptionsWidget, 2, 0, 1, 1)

        self.ParentStackedWidgetToChangeMenuOptions.addWidget(self.MainPagePage)
        self.ComponentControlPage = QWidget()
        self.ComponentControlPage.setObjectName(u"ComponentControlPage")
        self.CameraWidgetInComponentControl = QWidget(self.ComponentControlPage)
        self.CameraWidgetInComponentControl.setObjectName(u"CameraWidgetInComponentControl")
        self.CameraWidgetInComponentControl.setGeometry(QRect(0, 10, 701, 481))
        self.CameraWidgetInComponentControl.setStyleSheet(u"background-color: #000000;")
        self.horizontalLayout_15 = QHBoxLayout(self.CameraWidgetInComponentControl)
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.VisionTextInComponentControl = QLabel(self.CameraWidgetInComponentControl)
        self.VisionTextInComponentControl.setObjectName(u"VisionTextInComponentControl")
        self.VisionTextInComponentControl.setStyleSheet(u"color:white;")
        self.VisionTextInComponentControl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_15.addWidget(self.VisionTextInComponentControl, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)

        self.ComponentControlOptionsWidget = QWidget(self.ComponentControlPage)
        self.ComponentControlOptionsWidget.setObjectName(u"ComponentControlOptionsWidget")
        self.ComponentControlOptionsWidget.setGeometry(QRect(720, 0, 291, 101))
        self.horizontalLayout_16 = QHBoxLayout(self.ComponentControlOptionsWidget)
        self.horizontalLayout_16.setSpacing(25)
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.MotorStartedButton = QPushButton(self.ComponentControlOptionsWidget)
        self.MotorStartedButton.setObjectName(u"MotorStartedButton")
        sizePolicy1.setHeightForWidth(self.MotorStartedButton.sizePolicy().hasHeightForWidth())
        self.MotorStartedButton.setSizePolicy(sizePolicy1)
        self.MotorStartedButton.setStyleSheet(u"")
        self.MotorStartedButton.setCheckable(True)
        self.MotorStartedButton.setChecked(True)

        self.horizontalLayout_16.addWidget(self.MotorStartedButton)

        self.HamburgerMenu = QPushButton(self.ComponentControlOptionsWidget)
        self.HamburgerMenu.setObjectName(u"HamburgerMenu")
        sizePolicy.setHeightForWidth(self.HamburgerMenu.sizePolicy().hasHeightForWidth())
        self.HamburgerMenu.setSizePolicy(sizePolicy)
        self.HamburgerMenu.setStyleSheet(u"")
        self.HamburgerMenu.setIcon(icon5)
        self.HamburgerMenu.setIconSize(QSize(30, 30))

        self.horizontalLayout_16.addWidget(self.HamburgerMenu)

        self.ChangeComponentControlStackedWidget = QStackedWidget(self.ComponentControlPage)
        self.ChangeComponentControlStackedWidget.setObjectName(u"ChangeComponentControlStackedWidget")
        self.ChangeComponentControlStackedWidget.setGeometry(QRect(710, 100, 311, 591))
        self.ChangeComponentControlStackedWidget.setStyleSheet(u"")
        self.MotorPage = QWidget()
        self.MotorPage.setObjectName(u"MotorPage")
        self.ControlUpCP = QPushButton(self.MotorPage)
        self.ControlUpCP.setObjectName(u"ControlUpCP")
        self.ControlUpCP.setGeometry(QRect(120, 40, 101, 91))
        sizePolicy4 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Maximum)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.ControlUpCP.sizePolicy().hasHeightForWidth())
        self.ControlUpCP.setSizePolicy(sizePolicy4)
        self.ControlUpCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon6 = QIcon()
        icon6.addFile(u":/controlArrows/cartesian/up.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ControlUpCP.setIcon(icon6)
        self.ControlUpCP.setIconSize(QSize(80, 80))
        self.ControlLeftCP = QPushButton(self.MotorPage)
        self.ControlLeftCP.setObjectName(u"ControlLeftCP")
        self.ControlLeftCP.setGeometry(QRect(20, 130, 101, 91))
        self.ControlLeftCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon7 = QIcon()
        icon7.addFile(u":/controlArrows/cartesian/left.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ControlLeftCP.setIcon(icon7)
        self.ControlLeftCP.setIconSize(QSize(80, 80))
        self.ControlDownCP = QPushButton(self.MotorPage)
        self.ControlDownCP.setObjectName(u"ControlDownCP")
        self.ControlDownCP.setGeometry(QRect(120, 220, 101, 91))
        self.ControlDownCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon8 = QIcon()
        icon8.addFile(u":/controlArrows/cartesian/down.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ControlDownCP.setIcon(icon8)
        self.ControlDownCP.setIconSize(QSize(80, 80))
        self.ControlRightCP = QPushButton(self.MotorPage)
        self.ControlRightCP.setObjectName(u"ControlRightCP")
        self.ControlRightCP.setGeometry(QRect(220, 130, 101, 91))
        self.ControlRightCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon9 = QIcon()
        icon9.addFile(u":/controlArrows/cartesian/right.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.ControlRightCP.setIcon(icon9)
        self.ControlRightCP.setIconSize(QSize(80, 80))
        self.YawPlusCP = QPushButton(self.MotorPage)
        self.YawPlusCP.setObjectName(u"YawPlusCP")
        self.YawPlusCP.setGeometry(QRect(220, 20, 101, 91))
        sizePolicy4.setHeightForWidth(self.YawPlusCP.sizePolicy().hasHeightForWidth())
        self.YawPlusCP.setSizePolicy(sizePolicy4)
        self.YawPlusCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon10 = QIcon()
        icon10.addFile(u":/controlArrows/cartesian/yawPlus.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.YawPlusCP.setIcon(icon10)
        self.YawPlusCP.setIconSize(QSize(80, 80))
        self.YawMinusCP = QPushButton(self.MotorPage)
        self.YawMinusCP.setObjectName(u"YawMinusCP")
        self.YawMinusCP.setGeometry(QRect(10, 240, 101, 91))
        sizePolicy4.setHeightForWidth(self.YawMinusCP.sizePolicy().hasHeightForWidth())
        self.YawMinusCP.setSizePolicy(sizePolicy4)
        self.YawMinusCP.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon11 = QIcon()
        icon11.addFile(u":/controlArrows/cartesian/yawMinus.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.YawMinusCP.setIcon(icon11)
        self.YawMinusCP.setIconSize(QSize(80, 80))
        self.YawMinusCP.setCheckable(False)
        self.ClipperAndResetWidget_3 = QWidget(self.MotorPage)
        self.ClipperAndResetWidget_3.setObjectName(u"ClipperAndResetWidget_3")
        self.ClipperAndResetWidget_3.setGeometry(QRect(0, 440, 311, 151))
        self.verticalLayout_10 = QVBoxLayout(self.ClipperAndResetWidget_3)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.ClipperButtonOnOff = QPushButton(self.ClipperAndResetWidget_3)
        self.ClipperButtonOnOff.setObjectName(u"ClipperButtonOnOff")
        self.ClipperButtonOnOff.setEnabled(True)
        sizePolicy.setHeightForWidth(self.ClipperButtonOnOff.sizePolicy().hasHeightForWidth())
        self.ClipperButtonOnOff.setSizePolicy(sizePolicy)
        self.ClipperButtonOnOff.setFont(font)
        self.ClipperButtonOnOff.setStyleSheet(u"QPushButton {\n"
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
        self.ClipperButtonOnOff.setCheckable(True)

        self.verticalLayout_10.addWidget(self.ClipperButtonOnOff)

        self.MotorResetButton = QPushButton(self.ClipperAndResetWidget_3)
        self.MotorResetButton.setObjectName(u"MotorResetButton")
        sizePolicy.setHeightForWidth(self.MotorResetButton.sizePolicy().hasHeightForWidth())
        self.MotorResetButton.setSizePolicy(sizePolicy)
        self.MotorResetButton.setFont(font)
        self.MotorResetButton.setStyleSheet(u"QPushButton {\n"
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

        self.verticalLayout_10.addWidget(self.MotorResetButton)

        self.ChangeComponentControlStackedWidget.addWidget(self.MotorPage)
        self.VisionPage = QWidget()
        self.VisionPage.setObjectName(u"VisionPage")
        self.label_5 = QLabel(self.VisionPage)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(120, 270, 67, 17))
        self.ChangeComponentControlStackedWidget.addWidget(self.VisionPage)
        self.ClipperPage = QWidget()
        self.ClipperPage.setObjectName(u"ClipperPage")
        self.label_6 = QLabel(self.ClipperPage)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(130, 320, 67, 17))
        self.ChangeComponentControlStackedWidget.addWidget(self.ClipperPage)
        self.ForkLiftPage = QWidget()
        self.ForkLiftPage.setObjectName(u"ForkLiftPage")
        self.LiftUp = QPushButton(self.ForkLiftPage)
        self.LiftUp.setObjectName(u"LiftUp")
        self.LiftUp.setGeometry(QRect(20, 480, 101, 91))
        sizePolicy4.setHeightForWidth(self.LiftUp.sizePolicy().hasHeightForWidth())
        self.LiftUp.setSizePolicy(sizePolicy4)
        self.LiftUp.setStyleSheet(u"background-color: transparent;\n"
"border: none;")
        icon12 = QIcon()
        icon12.addFile(u":/controlArrows/lift.svg", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.LiftUp.setIcon(icon12)
        self.LiftUp.setIconSize(QSize(80, 80))
        self.LiftUp.setCheckable(False)
        self.LowerDown = QPushButton(self.ForkLiftPage)
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
        self.SliderLift = QSlider(self.ForkLiftPage)
        self.SliderLift.setObjectName(u"SliderLift")
        self.SliderLift.setGeometry(QRect(60, 400, 201, 61))
        self.SliderLift.setMinimum(80)
        self.SliderLift.setMaximum(200)
        self.SliderLift.setValue(80)
        self.SliderLift.setSliderPosition(80)
        self.SliderLift.setOrientation(Qt.Orientation.Horizontal)
        self.FastForktLiftButton = QPushButton(self.ForkLiftPage)
        self.buttonGroup = QButtonGroup(MainWindow)
        self.buttonGroup.setObjectName(u"buttonGroup")
        self.buttonGroup.addButton(self.FastForktLiftButton)
        self.FastForktLiftButton.setObjectName(u"FastForktLiftButton")
        self.FastForktLiftButton.setGeometry(QRect(30, 300, 111, 71))
        self.FastForktLiftButton.setCheckable(True)
        self.SlowLiftButton = QPushButton(self.ForkLiftPage)
        self.buttonGroup.addButton(self.SlowLiftButton)
        self.SlowLiftButton.setObjectName(u"SlowLiftButton")
        self.SlowLiftButton.setGeometry(QRect(170, 300, 111, 71))
        self.SlowLiftButton.setCheckable(True)
        self.RunForkLiftButton = QPushButton(self.ForkLiftPage)
        self.buttonGroup_2 = QButtonGroup(MainWindow)
        self.buttonGroup_2.setObjectName(u"buttonGroup_2")
        self.buttonGroup_2.addButton(self.RunForkLiftButton)
        self.RunForkLiftButton.setObjectName(u"RunForkLiftButton")
        self.RunForkLiftButton.setGeometry(QRect(20, 130, 111, 71))
        self.RunForkLiftButton.setCheckable(True)
        self.StopForkLiftButton = QPushButton(self.ForkLiftPage)
        self.buttonGroup_2.addButton(self.StopForkLiftButton)
        self.StopForkLiftButton.setObjectName(u"StopForkLiftButton")
        self.StopForkLiftButton.setGeometry(QRect(170, 130, 111, 71))
        self.StopForkLiftButton.setCheckable(True)
        self.ChangeComponentControlStackedWidget.addWidget(self.ForkLiftPage)
        self.ListOptionsWidget = QWidget(self.ComponentControlPage)
        self.ListOptionsWidget.setObjectName(u"ListOptionsWidget")
        self.ListOptionsWidget.setGeometry(QRect(740, 110, 251, 301))
        sizePolicy1.setHeightForWidth(self.ListOptionsWidget.sizePolicy().hasHeightForWidth())
        self.ListOptionsWidget.setSizePolicy(sizePolicy1)
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

        self.ForkLiftOption = QPushButton(self.ListOptionsWidget)
        self.ForkLiftOption.setObjectName(u"ForkLiftOption")
        sizePolicy.setHeightForWidth(self.ForkLiftOption.sizePolicy().hasHeightForWidth())
        self.ForkLiftOption.setSizePolicy(sizePolicy)
        self.ForkLiftOption.setStyleSheet(u"")

        self.verticalLayout_5.addWidget(self.ForkLiftOption)

        self.Info_2 = QFrame(self.ComponentControlPage)
        self.Info_2.setObjectName(u"Info_2")
        self.Info_2.setGeometry(QRect(0, 500, 701, 191))
        self.Info_2.setStyleSheet(u" background-color: #000000; color: white;")
        self.Info_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.Info_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_19 = QHBoxLayout(self.Info_2)
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.horizontalLayout_19.setContentsMargins(-1, 0, -1, 9)
        self.CartesianPoseWidget_2 = QWidget(self.Info_2)
        self.CartesianPoseWidget_2.setObjectName(u"CartesianPoseWidget_2")
        sizePolicy1.setHeightForWidth(self.CartesianPoseWidget_2.sizePolicy().hasHeightForWidth())
        self.CartesianPoseWidget_2.setSizePolicy(sizePolicy1)
        self.CartesianPoseWidget_2.setMinimumSize(QSize(0, 0))
        self.gridLayout_4 = QGridLayout(self.CartesianPoseWidget_2)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.gridLayout_4.setContentsMargins(-1, 6, -1, -1)
        self.yPosFrame_2 = QFrame(self.CartesianPoseWidget_2)
        self.yPosFrame_2.setObjectName(u"yPosFrame_2")
        self.yPosFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.yPosFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_22 = QHBoxLayout(self.yPosFrame_2)
        self.horizontalLayout_22.setObjectName(u"horizontalLayout_22")
        self.horizontalLayout_22.setContentsMargins(-1, -1, -1, 3)
        self.y_2 = QLabel(self.yPosFrame_2)
        self.y_2.setObjectName(u"y_2")
        self.y_2.setFont(font2)

        self.horizontalLayout_22.addWidget(self.y_2, 0, Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.yText_2 = QLabel(self.yPosFrame_2)
        self.yText_2.setObjectName(u"yText_2")

        self.horizontalLayout_22.addWidget(self.yText_2, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.gridLayout_4.addWidget(self.yPosFrame_2, 1, 1, 1, 1)

        self.zPosFrame_2 = QFrame(self.CartesianPoseWidget_2)
        self.zPosFrame_2.setObjectName(u"zPosFrame_2")
        self.zPosFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.zPosFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_27 = QHBoxLayout(self.zPosFrame_2)
        self.horizontalLayout_27.setObjectName(u"horizontalLayout_27")
        self.horizontalLayout_27.setContentsMargins(-1, -1, -1, 14)
        self.z_2 = QLabel(self.zPosFrame_2)
        self.z_2.setObjectName(u"z_2")
        self.z_2.setFont(font2)

        self.horizontalLayout_27.addWidget(self.z_2, 0, Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.zText_2 = QLabel(self.zPosFrame_2)
        self.zText_2.setObjectName(u"zText_2")

        self.horizontalLayout_27.addWidget(self.zText_2, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.gridLayout_4.addWidget(self.zPosFrame_2, 2, 0, 1, 1)

        self.yawPosFrame_2 = QFrame(self.CartesianPoseWidget_2)
        self.yawPosFrame_2.setObjectName(u"yawPosFrame_2")
        self.yawPosFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.yawPosFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_28 = QHBoxLayout(self.yawPosFrame_2)
        self.horizontalLayout_28.setObjectName(u"horizontalLayout_28")
        self.horizontalLayout_28.setContentsMargins(-1, -1, -1, 14)
        self.yaw_2 = QLabel(self.yawPosFrame_2)
        self.yaw_2.setObjectName(u"yaw_2")
        self.yaw_2.setFont(font2)

        self.horizontalLayout_28.addWidget(self.yaw_2, 0, Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.yawText_2 = QLabel(self.yawPosFrame_2)
        self.yawText_2.setObjectName(u"yawText_2")

        self.horizontalLayout_28.addWidget(self.yawText_2, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.gridLayout_4.addWidget(self.yawPosFrame_2, 2, 1, 1, 1)

        self.CartesianPoseText_2 = QLabel(self.CartesianPoseWidget_2)
        self.CartesianPoseText_2.setObjectName(u"CartesianPoseText_2")
        self.CartesianPoseText_2.setFont(font3)
        self.CartesianPoseText_2.setStyleSheet(u"border: 2px solid #FFFFFF;  /* white solid border */\n"
"border-radius: 6px;")

        self.gridLayout_4.addWidget(self.CartesianPoseText_2, 0, 0, 1, 2, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignTop)

        self.xPosFrame_2 = QFrame(self.CartesianPoseWidget_2)
        self.xPosFrame_2.setObjectName(u"xPosFrame_2")
        self.xPosFrame_2.setFont(font2)
        self.xPosFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.xPosFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_29 = QHBoxLayout(self.xPosFrame_2)
        self.horizontalLayout_29.setObjectName(u"horizontalLayout_29")
        self.horizontalLayout_29.setContentsMargins(9, -1, -1, 3)
        self.x_2 = QLabel(self.xPosFrame_2)
        self.x_2.setObjectName(u"x_2")
        self.x_2.setFont(font2)

        self.horizontalLayout_29.addWidget(self.x_2, 0, Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.xText_2 = QLabel(self.xPosFrame_2)
        self.xText_2.setObjectName(u"xText_2")

        self.horizontalLayout_29.addWidget(self.xText_2, 0, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignVCenter)


        self.gridLayout_4.addWidget(self.xPosFrame_2, 1, 0, 1, 1)


        self.horizontalLayout_19.addWidget(self.CartesianPoseWidget_2)

        self.MotorInfoWidget_2 = QWidget(self.Info_2)
        self.MotorInfoWidget_2.setObjectName(u"MotorInfoWidget_2")
        self.MotorInfoWidget_2.setStyleSheet(u"")
        self.gridLayout_5 = QGridLayout(self.MotorInfoWidget_2)
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.gridLayout_5.setContentsMargins(-1, 6, -1, -1)
        self.PositionFrame_2 = QFrame(self.MotorInfoWidget_2)
        self.PositionFrame_2.setObjectName(u"PositionFrame_2")
        self.PositionFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.PositionFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_30 = QHBoxLayout(self.PositionFrame_2)
        self.horizontalLayout_30.setObjectName(u"horizontalLayout_30")
        self.Position_2 = QLabel(self.PositionFrame_2)
        self.Position_2.setObjectName(u"Position_2")

        self.horizontalLayout_30.addWidget(self.Position_2)

        self.PositionText_2 = QLabel(self.PositionFrame_2)
        self.PositionText_2.setObjectName(u"PositionText_2")

        self.horizontalLayout_30.addWidget(self.PositionText_2)


        self.gridLayout_5.addWidget(self.PositionFrame_2, 2, 0, 1, 1)

        self.MotorInfoText_2 = QLabel(self.MotorInfoWidget_2)
        self.MotorInfoText_2.setObjectName(u"MotorInfoText_2")
        self.MotorInfoText_2.setFont(font3)
        self.MotorInfoText_2.setStyleSheet(u"border: 2px solid #FFFFFF;  /* white solid border */\n"
"border-radius: 6px;")

        self.gridLayout_5.addWidget(self.MotorInfoText_2, 0, 0, 1, 1, Qt.AlignmentFlag.AlignHCenter|Qt.AlignmentFlag.AlignTop)

        self.CurrentFrame_2 = QFrame(self.MotorInfoWidget_2)
        self.CurrentFrame_2.setObjectName(u"CurrentFrame_2")
        self.CurrentFrame_2.setFrameShape(QFrame.Shape.StyledPanel)
        self.CurrentFrame_2.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_31 = QHBoxLayout(self.CurrentFrame_2)
        self.horizontalLayout_31.setObjectName(u"horizontalLayout_31")
        self.Current_2 = QLabel(self.CurrentFrame_2)
        self.Current_2.setObjectName(u"Current_2")

        self.horizontalLayout_31.addWidget(self.Current_2)

        self.CurrentText_2 = QLabel(self.CurrentFrame_2)
        self.CurrentText_2.setObjectName(u"CurrentText_2")

        self.horizontalLayout_31.addWidget(self.CurrentText_2)


        self.gridLayout_5.addWidget(self.CurrentFrame_2, 3, 0, 1, 1)


        self.horizontalLayout_19.addWidget(self.MotorInfoWidget_2)

        self.RecordData_2 = QWidget(self.Info_2)
        self.RecordData_2.setObjectName(u"RecordData_2")
        self.gridLayout_6 = QGridLayout(self.RecordData_2)
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.gridLayout_6.setContentsMargins(-1, 6, -1, -1)
        self.frame_12 = QFrame(self.RecordData_2)
        self.frame_12.setObjectName(u"frame_12")
        self.frame_12.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_12.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_15 = QVBoxLayout(self.frame_12)
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.verticalLayout_15.setContentsMargins(9, 9, -1, 0)
        self.d1_2 = QLabel(self.frame_12)
        self.d1_2.setObjectName(u"d1_2")

        self.verticalLayout_15.addWidget(self.d1_2)


        self.gridLayout_6.addWidget(self.frame_12, 1, 0, 1, 1)

        self.LaserInfoText_2 = QLabel(self.RecordData_2)
        self.LaserInfoText_2.setObjectName(u"LaserInfoText_2")
        self.LaserInfoText_2.setFont(font3)
        self.LaserInfoText_2.setStyleSheet(u"border: 2px solid #FFFFFF;  /* white solid border */\n"
"border-radius: 6px;")

        self.gridLayout_6.addWidget(self.LaserInfoText_2, 0, 0, 1, 1, Qt.AlignmentFlag.AlignHCenter)

        self.frame_7 = QFrame(self.RecordData_2)
        self.frame_7.setObjectName(u"frame_7")
        self.frame_7.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_7.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_23 = QVBoxLayout(self.frame_7)
        self.verticalLayout_23.setObjectName(u"verticalLayout_23")
        self.verticalLayout_23.setContentsMargins(9, -1, -1, 0)
        self.d2_2 = QLabel(self.frame_7)
        self.d2_2.setObjectName(u"d2_2")

        self.verticalLayout_23.addWidget(self.d2_2)


        self.gridLayout_6.addWidget(self.frame_7, 3, 0, 1, 1)

        self.frame_8 = QFrame(self.RecordData_2)
        self.frame_8.setObjectName(u"frame_8")
        self.frame_8.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_8.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_17 = QVBoxLayout(self.frame_8)
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.verticalLayout_17.setContentsMargins(9, -1, -1, 0)
        self.h1_2 = QLabel(self.frame_8)
        self.h1_2.setObjectName(u"h1_2")

        self.verticalLayout_17.addWidget(self.h1_2)


        self.gridLayout_6.addWidget(self.frame_8, 4, 0, 1, 1)


        self.horizontalLayout_19.addWidget(self.RecordData_2)

        self.RecordDataButtonContainer_2 = QWidget(self.Info_2)
        self.RecordDataButtonContainer_2.setObjectName(u"RecordDataButtonContainer_2")
        sizePolicy1.setHeightForWidth(self.RecordDataButtonContainer_2.sizePolicy().hasHeightForWidth())
        self.RecordDataButtonContainer_2.setSizePolicy(sizePolicy1)
        self.verticalLayout_21 = QVBoxLayout(self.RecordDataButtonContainer_2)
        self.verticalLayout_21.setObjectName(u"verticalLayout_21")
        self.verticalLayout_21.setContentsMargins(9, 9, -1, -1)
        self.RecordDataButton_2 = QPushButton(self.RecordDataButtonContainer_2)
        self.RecordDataButton_2.setObjectName(u"RecordDataButton_2")
        sizePolicy1.setHeightForWidth(self.RecordDataButton_2.sizePolicy().hasHeightForWidth())
        self.RecordDataButton_2.setSizePolicy(sizePolicy1)
        self.RecordDataButton_2.setFont(font)
        self.RecordDataButton_2.setStyleSheet(u"QPushButton {\n"
"    padding: 16px 24px;\n"
"}\n"
"\n"
"")

        self.verticalLayout_21.addWidget(self.RecordDataButton_2, 0, Qt.AlignmentFlag.AlignVCenter)


        self.horizontalLayout_19.addWidget(self.RecordDataButtonContainer_2)

        self.ParentStackedWidgetToChangeMenuOptions.addWidget(self.ComponentControlPage)
        self.ChangeComponentControlStackedWidget.raise_()
        self.CameraWidgetInComponentControl.raise_()
        self.ComponentControlOptionsWidget.raise_()
        self.Info_2.raise_()
        self.ListOptionsWidget.raise_()
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
        self.WorkOrderWidget.setGeometry(QRect(530, 0, 141, 41))
        self.WorkOrderWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_17 = QHBoxLayout(self.WorkOrderWidget)
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.WorkOrderText = QLabel(self.WorkOrderWidget)
        self.WorkOrderText.setObjectName(u"WorkOrderText")
        font4 = QFont()
        font4.setBold(True)
        self.WorkOrderText.setFont(font4)

        self.horizontalLayout_17.addWidget(self.WorkOrderText)

        self.WorkOrderNumberInput = QLabel(self.WorkOrderWidget)
        self.WorkOrderNumberInput.setObjectName(u"WorkOrderNumberInput")

        self.horizontalLayout_17.addWidget(self.WorkOrderNumberInput)

        self.RecipeWidget = QWidget(self.BackgroundWidget)
        self.RecipeWidget.setObjectName(u"RecipeWidget")
        self.RecipeWidget.setGeometry(QRect(530, 30, 171, 41))
        self.RecipeWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_18 = QHBoxLayout(self.RecipeWidget)
        self.horizontalLayout_18.setObjectName(u"horizontalLayout_18")
        self.RecipeText = QLabel(self.RecipeWidget)
        self.RecipeText.setObjectName(u"RecipeText")
        self.RecipeText.setFont(font4)

        self.horizontalLayout_18.addWidget(self.RecipeText)

        self.RecipeNameInput = QLabel(self.RecipeWidget)
        self.RecipeNameInput.setObjectName(u"RecipeNameInput")

        self.horizontalLayout_18.addWidget(self.RecipeNameInput)

        self.QuantityWidget = QWidget(self.BackgroundWidget)
        self.QuantityWidget.setObjectName(u"QuantityWidget")
        self.QuantityWidget.setGeometry(QRect(800, 0, 181, 41))
        self.QuantityWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_20 = QHBoxLayout(self.QuantityWidget)
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.QuantityText = QLabel(self.QuantityWidget)
        self.QuantityText.setObjectName(u"QuantityText")
        self.QuantityText.setFont(font4)

        self.horizontalLayout_20.addWidget(self.QuantityText)

        self.QuantityNumberInput = QLabel(self.QuantityWidget)
        self.QuantityNumberInput.setObjectName(u"QuantityNumberInput")

        self.horizontalLayout_20.addWidget(self.QuantityNumberInput)

        self.WorkerNameWidget = QWidget(self.BackgroundWidget)
        self.WorkerNameWidget.setObjectName(u"WorkerNameWidget")
        self.WorkerNameWidget.setGeometry(QRect(800, 30, 171, 41))
        self.WorkerNameWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_21 = QHBoxLayout(self.WorkerNameWidget)
        self.horizontalLayout_21.setObjectName(u"horizontalLayout_21")
        self.WorkerNameText = QLabel(self.WorkerNameWidget)
        self.WorkerNameText.setObjectName(u"WorkerNameText")
        self.WorkerNameText.setFont(font4)

        self.horizontalLayout_21.addWidget(self.WorkerNameText)

        self.WorkerNameInput = QLabel(self.WorkerNameWidget)
        self.WorkerNameInput.setObjectName(u"WorkerNameInput")

        self.horizontalLayout_21.addWidget(self.WorkerNameInput)

        self.CartHeightWidget = QWidget(self.BackgroundWidget)
        self.CartHeightWidget.setObjectName(u"CartHeightWidget")
        self.CartHeightWidget.setGeometry(QRect(250, 30, 221, 41))
        self.CartHeightWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_23 = QHBoxLayout(self.CartHeightWidget)
        self.horizontalLayout_23.setObjectName(u"horizontalLayout_23")
        self.CartHeightText = QLabel(self.CartHeightWidget)
        self.CartHeightText.setObjectName(u"CartHeightText")
        self.CartHeightText.setFont(font4)

        self.horizontalLayout_23.addWidget(self.CartHeightText)

        self.CartHeightInput = QLabel(self.CartHeightWidget)
        self.CartHeightInput.setObjectName(u"CartHeightInput")

        self.horizontalLayout_23.addWidget(self.CartHeightInput)

        self.CartDepthWidget = QWidget(self.BackgroundWidget)
        self.CartDepthWidget.setObjectName(u"CartDepthWidget")
        self.CartDepthWidget.setGeometry(QRect(250, 0, 221, 41))
        self.CartDepthWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_24 = QHBoxLayout(self.CartDepthWidget)
        self.horizontalLayout_24.setObjectName(u"horizontalLayout_24")
        self.CartDepthText = QLabel(self.CartDepthWidget)
        self.CartDepthText.setObjectName(u"CartDepthText")
        self.CartDepthText.setFont(font4)

        self.horizontalLayout_24.addWidget(self.CartDepthText)

        self.CartDepthInput = QLabel(self.CartDepthWidget)
        self.CartDepthInput.setObjectName(u"CartDepthInput")

        self.horizontalLayout_24.addWidget(self.CartDepthInput)

        self.DateWidget = QWidget(self.BackgroundWidget)
        self.DateWidget.setObjectName(u"DateWidget")
        self.DateWidget.setGeometry(QRect(1050, 10, 151, 41))
        self.DateWidget.setStyleSheet(u"color: white;")
        self.horizontalLayout_26 = QHBoxLayout(self.DateWidget)
        self.horizontalLayout_26.setObjectName(u"horizontalLayout_26")
        self.DateText = QLabel(self.DateWidget)
        self.DateText.setObjectName(u"DateText")
        self.DateText.setFont(font4)

        self.horizontalLayout_26.addWidget(self.DateText)

        self.DateInput = QLabel(self.DateWidget)
        self.DateInput.setObjectName(u"DateInput")

        self.horizontalLayout_26.addWidget(self.DateInput)

        self.ParentStackedWidgetToChangeMenuOptions.raise_()
        self.Line.raise_()
        self.SystemSettingsButton.raise_()
        self.SignalLightsWidget.raise_()
        self.DeltaLogo.raise_()
        self.MenuButtons.raise_()
        self.WorkOrderWidget.raise_()
        self.RecipeWidget.raise_()
        self.QuantityWidget.raise_()
        self.WorkerNameWidget.raise_()
        self.CartHeightWidget.raise_()
        self.CartDepthWidget.raise_()
        self.DateWidget.raise_()

        self.horizontalLayout_4.addWidget(self.BackgroundWidget)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(0)
        self.ActionButtons.setCurrentIndex(0)
        self.ProcessAndInfoStackedWidget.setCurrentIndex(0)
        self.ChangeComponentControlStackedWidget.setCurrentIndex(0)


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
        self.HamburgerMenuMainPage.setText("")
        self.InitButton.setText(QCoreApplication.translate("MainWindow", u"INIT", None))
        self.RunButton.setText(QCoreApplication.translate("MainWindow", u"RUN", None))
        self.StopButton.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.AutoResetButton.setText(QCoreApplication.translate("MainWindow", u"RESET", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Rough Align Page", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"PreciseAlignPage", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Pick Page", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"Assembly Page", None))
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
        self.y.setText(QCoreApplication.translate("MainWindow", u"y:", None))
        self.yText.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.z.setText(QCoreApplication.translate("MainWindow", u"z:", None))
        self.zText.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.yaw.setText(QCoreApplication.translate("MainWindow", u"yaw:", None))
        self.yawText.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.CartesianPoseText.setText(QCoreApplication.translate("MainWindow", u"Cartesian Pose", None))
        self.x.setText(QCoreApplication.translate("MainWindow", u"x:", None))
        self.xText.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.Position.setText(QCoreApplication.translate("MainWindow", u"Position:", None))
        self.PositionText.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.MotorInfoText.setText(QCoreApplication.translate("MainWindow", u"Motor Info", None))
        self.Current.setText(QCoreApplication.translate("MainWindow", u"Current: ", None))
        self.CurrentText.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.d1.setText(QCoreApplication.translate("MainWindow", u"D1: ", None))
        self.LaserInfoText.setText(QCoreApplication.translate("MainWindow", u"Laser Info", None))
        self.d2.setText(QCoreApplication.translate("MainWindow", u"D2: ", None))
        self.h1.setText(QCoreApplication.translate("MainWindow", u"H1: ", None))
        self.RecordDataButton.setText(QCoreApplication.translate("MainWindow", u"Record Data", None))
        self.AutoButtonOption.setText(QCoreApplication.translate("MainWindow", u"Auto", None))
        self.ManualButton.setText(QCoreApplication.translate("MainWindow", u"Manual", None))
        self.RoughAlignButton.setText(QCoreApplication.translate("MainWindow", u"Rough Align", None))
        self.PreciseAlignButton.setText(QCoreApplication.translate("MainWindow", u"Precise Align", None))
        self.PickButton.setText(QCoreApplication.translate("MainWindow", u"Pick", None))
        self.AssemblyButton.setText(QCoreApplication.translate("MainWindow", u"Assembly", None))
        self.VisionTextInComponentControl.setText(QCoreApplication.translate("MainWindow", u"Vision", None))
        self.MotorStartedButton.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.HamburgerMenu.setText("")
        self.ControlUpCP.setText("")
        self.ControlLeftCP.setText("")
        self.ControlDownCP.setText("")
        self.ControlRightCP.setText("")
        self.YawPlusCP.setText("")
        self.YawMinusCP.setText("")
        self.ClipperButtonOnOff.setText(QCoreApplication.translate("MainWindow", u"Clipper  Off", None))
        self.MotorResetButton.setText(QCoreApplication.translate("MainWindow", u"RESET", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Viison Page", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Clipper Page", None))
        self.LiftUp.setText("")
        self.LowerDown.setText("")
        self.FastForktLiftButton.setText(QCoreApplication.translate("MainWindow", u"Fast", None))
        self.SlowLiftButton.setText(QCoreApplication.translate("MainWindow", u"Slow", None))
        self.RunForkLiftButton.setText(QCoreApplication.translate("MainWindow", u"Run", None))
        self.StopForkLiftButton.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.MotorOption.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.VisionOption.setText(QCoreApplication.translate("MainWindow", u"Vision", None))
        self.ClipperOption.setText(QCoreApplication.translate("MainWindow", u"Clipper", None))
        self.ForkLiftOption.setText(QCoreApplication.translate("MainWindow", u"Fork Lift", None))
        self.y_2.setText(QCoreApplication.translate("MainWindow", u"y:", None))
        self.yText_2.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.z_2.setText(QCoreApplication.translate("MainWindow", u"z:", None))
        self.zText_2.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.yaw_2.setText(QCoreApplication.translate("MainWindow", u"yaw:", None))
        self.yawText_2.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.CartesianPoseText_2.setText(QCoreApplication.translate("MainWindow", u"Cartesian Pose", None))
        self.x_2.setText(QCoreApplication.translate("MainWindow", u"x:", None))
        self.xText_2.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.Position_2.setText(QCoreApplication.translate("MainWindow", u"Position:", None))
        self.PositionText_2.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.MotorInfoText_2.setText(QCoreApplication.translate("MainWindow", u"Motor Info", None))
        self.Current_2.setText(QCoreApplication.translate("MainWindow", u"Current: ", None))
        self.CurrentText_2.setText(QCoreApplication.translate("MainWindow", u"pos", None))
        self.d1_2.setText(QCoreApplication.translate("MainWindow", u"D1: ", None))
        self.LaserInfoText_2.setText(QCoreApplication.translate("MainWindow", u"Laser Info", None))
        self.d2_2.setText(QCoreApplication.translate("MainWindow", u"D2: ", None))
        self.h1_2.setText(QCoreApplication.translate("MainWindow", u"H1: ", None))
        self.RecordDataButton_2.setText(QCoreApplication.translate("MainWindow", u"Record Data", None))
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
    # retranslateUi

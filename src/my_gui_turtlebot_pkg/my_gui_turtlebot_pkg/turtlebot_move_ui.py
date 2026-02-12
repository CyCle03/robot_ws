# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'turtlebot_move.ui'
##
## Created by: Qt User Interface Compiler version 6.10.2
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
from PySide6.QtWidgets import (QApplication, QListWidget, QListWidgetItem, QMainWindow,
    QMenuBar, QPushButton, QSizePolicy, QStatusBar,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.listWidget = QListWidget(self.centralwidget)
        self.listWidget.setObjectName(u"listWidget")
        self.listWidget.setGeometry(QRect(440, 80, 311, 411))
        self.btn_left = QPushButton(self.centralwidget)
        self.btn_left.setObjectName(u"btn_left")
        self.btn_left.setGeometry(QRect(20, 190, 101, 81))
        self.btn_go = QPushButton(self.centralwidget)
        self.btn_go.setObjectName(u"btn_go")
        self.btn_go.setGeometry(QRect(140, 90, 101, 81))
        self.btn_right = QPushButton(self.centralwidget)
        self.btn_right.setObjectName(u"btn_right")
        self.btn_right.setGeometry(QRect(260, 190, 101, 81))
        self.btn_stop = QPushButton(self.centralwidget)
        self.btn_stop.setObjectName(u"btn_stop")
        self.btn_stop.setGeometry(QRect(140, 190, 101, 81))
        self.btn_back = QPushButton(self.centralwidget)
        self.btn_back.setObjectName(u"btn_back")
        self.btn_back.setGeometry(QRect(140, 290, 101, 81))
        self.btn_patrol_tri = QPushButton(self.centralwidget)
        self.btn_patrol_tri.setObjectName(u"btn_patrol_tri")
        self.btn_patrol_tri.setGeometry(QRect(20, 430, 101, 81))
        self.btn_patrol_sqr = QPushButton(self.centralwidget)
        self.btn_patrol_sqr.setObjectName(u"btn_patrol_sqr")
        self.btn_patrol_sqr.setGeometry(QRect(140, 430, 101, 81))
        self.btn_patrol_stop = QPushButton(self.centralwidget)
        self.btn_patrol_stop.setObjectName(u"btn_patrol_stop")
        self.btn_patrol_stop.setGeometry(QRect(260, 430, 101, 81))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 800, 27))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.btn_left.setText(QCoreApplication.translate("MainWindow", u"Left", None))
        self.btn_go.setText(QCoreApplication.translate("MainWindow", u"Go", None))
        self.btn_right.setText(QCoreApplication.translate("MainWindow", u"Right", None))
        self.btn_stop.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.btn_back.setText(QCoreApplication.translate("MainWindow", u"Back", None))
        self.btn_patrol_tri.setText(QCoreApplication.translate("MainWindow", u"patrol(tri)", None))
        self.btn_patrol_sqr.setText(QCoreApplication.translate("MainWindow", u"patrol(sqr)", None))
        self.btn_patrol_stop.setText(QCoreApplication.translate("MainWindow", u"patrol stop", None))
    # retranslateUi


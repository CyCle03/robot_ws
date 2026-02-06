# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'msg_qt.ui'
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
from PySide6.QtWidgets import (QApplication, QLabel, QListWidget, QListWidgetItem,
    QMainWindow, QMenuBar, QPushButton, QSizePolicy,
    QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.btn_subscriber = QPushButton(self.centralwidget)
        self.btn_subscriber.setObjectName(u"btn_subscriber")
        self.btn_subscriber.setGeometry(QRect(110, 70, 96, 27))
        self.btn_sub_cancel = QPushButton(self.centralwidget)
        self.btn_sub_cancel.setObjectName(u"btn_sub_cancel")
        self.btn_sub_cancel.setGeometry(QRect(230, 70, 96, 27))
        self.btn_publisher = QPushButton(self.centralwidget)
        self.btn_publisher.setObjectName(u"btn_publisher")
        self.btn_publisher.setGeometry(QRect(110, 120, 96, 27))
        self.btn_pub_cancel = QPushButton(self.centralwidget)
        self.btn_pub_cancel.setObjectName(u"btn_pub_cancel")
        self.btn_pub_cancel.setGeometry(QRect(230, 120, 96, 27))
        self.lbl_sub = QLabel(self.centralwidget)
        self.lbl_sub.setObjectName(u"lbl_sub")
        self.lbl_sub.setGeometry(QRect(10, 70, 81, 19))
        self.lbl_pub = QLabel(self.centralwidget)
        self.lbl_pub.setObjectName(u"lbl_pub")
        self.lbl_pub.setGeometry(QRect(20, 120, 69, 19))
        self.msg_list = QListWidget(self.centralwidget)
        self.msg_list.setObjectName(u"msg_list")
        self.msg_list.setGeometry(QRect(360, 60, 371, 471))
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
        self.btn_subscriber.setText(QCoreApplication.translate("MainWindow", u"Subscribe", None))
        self.btn_sub_cancel.setText(QCoreApplication.translate("MainWindow", u"Cancel", None))
        self.btn_publisher.setText(QCoreApplication.translate("MainWindow", u"Publish", None))
        self.btn_pub_cancel.setText(QCoreApplication.translate("MainWindow", u"Cancel", None))
        self.lbl_sub.setText(QCoreApplication.translate("MainWindow", u"subscriber", None))
        self.lbl_pub.setText(QCoreApplication.translate("MainWindow", u"publisher", None))
    # retranslateUi


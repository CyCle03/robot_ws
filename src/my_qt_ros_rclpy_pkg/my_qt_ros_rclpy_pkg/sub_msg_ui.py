# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'sub_msg.ui'
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
from PySide6.QtWidgets import (QApplication, QListWidget, QListWidgetItem, QPushButton,
    QSizePolicy, QWidget)

class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(995, 829)
        self.btn_start = QPushButton(Form)
        self.btn_start.setObjectName(u"btn_start")
        self.btn_start.setGeometry(QRect(60, 160, 171, 71))
        font = QFont()
        font.setPointSize(15)
        self.btn_start.setFont(font)
        self.btn_cancel = QPushButton(Form)
        self.btn_cancel.setObjectName(u"btn_cancel")
        self.btn_cancel.setGeometry(QRect(60, 260, 171, 71))
        self.btn_cancel.setFont(font)
        self.listWidget = QListWidget(Form)
        self.listWidget.setObjectName(u"listWidget")
        self.listWidget.setGeometry(QRect(300, 140, 591, 621))

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.btn_start.setText(QCoreApplication.translate("Form", u"start", None))
        self.btn_cancel.setText(QCoreApplication.translate("Form", u"cancel", None))
    # retranslateUi


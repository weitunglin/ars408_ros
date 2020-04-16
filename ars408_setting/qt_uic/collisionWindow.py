# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'collisionWindow.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(593, 250)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.checkBox0 = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox0.setGeometry(QtCore.QRect(30, 30, 121, 21))
        self.checkBox0.setObjectName("checkBox0")
        self.checkBox1 = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox1.setGeometry(QtCore.QRect(230, 30, 101, 21))
        self.checkBox1.setObjectName("checkBox1")
        self.checkBox3 = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox3.setGeometry(QtCore.QRect(30, 130, 131, 21))
        self.checkBox3.setObjectName("checkBox3")
        self.checkBox2 = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox2.setGeometry(QtCore.QRect(430, 30, 121, 31))
        self.checkBox2.setObjectName("checkBox2")
        self.label0 = QtWidgets.QLabel(self.centralwidget)
        self.label0.setGeometry(QtCore.QRect(180, 130, 140, 25))
        self.label0.setObjectName("label0")
        self.maxSlider0 = QtWidgets.QSlider(self.centralwidget)
        self.maxSlider0.setGeometry(QtCore.QRect(300, 130, 141, 25))
        self.maxSlider0.setMaximum(255)
        self.maxSlider0.setProperty("value", 255)
        self.maxSlider0.setOrientation(QtCore.Qt.Horizontal)
        self.maxSlider0.setObjectName("maxSlider0")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(270, 200, 191, 25))
        self.lineEdit.setObjectName("lineEdit")
        self.sendButton = QtWidgets.QPushButton(self.centralwidget)
        self.sendButton.setGeometry(QtCore.QRect(480, 200, 89, 25))
        self.sendButton.setObjectName("sendButton")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.checkBox0.setText(_translate("MainWindow", "WarningReset"))
        self.checkBox1.setText(_translate("MainWindow", "Activation"))
        self.checkBox3.setText(_translate("MainWindow", "MinTime_valid"))
        self.checkBox2.setText(_translate("MainWindow", "ClearRegions "))
        self.label0.setText(_translate("MainWindow", "0 ~ 25.5"))
        self.lineEdit.setText(_translate("MainWindow", "can0"))
        self.lineEdit.setPlaceholderText(_translate("MainWindow", "can name"))
        self.sendButton.setText(_translate("MainWindow", "PushButton"))


# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'speedWindow.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(548, 364)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.labelD = QtWidgets.QLabel(self.centralwidget)
        self.labelD.setGeometry(QtCore.QRect(50, 50, 140, 25))
        self.labelD.setObjectName("labelD")
        self.comboBox0 = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox0.setGeometry(QtCore.QRect(180, 50, 91, 31))
        self.comboBox0.setObjectName("comboBox0")
        self.comboBox0.addItem("")
        self.comboBox0.addItem("")
        self.comboBox0.addItem("")
        self.labelS = QtWidgets.QLabel(self.centralwidget)
        self.labelS.setGeometry(QtCore.QRect(50, 150, 140, 25))
        self.labelS.setObjectName("labelS")
        self.label0 = QtWidgets.QLabel(self.centralwidget)
        self.label0.setGeometry(QtCore.QRect(200, 150, 140, 25))
        self.label0.setObjectName("label0")
        self.maxSlider0 = QtWidgets.QSlider(self.centralwidget)
        self.maxSlider0.setGeometry(QtCore.QRect(330, 150, 141, 25))
        self.maxSlider0.setMaximum(8191)
        self.maxSlider0.setProperty("value", 8191)
        self.maxSlider0.setOrientation(QtCore.Qt.Horizontal)
        self.maxSlider0.setObjectName("maxSlider0")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(200, 310, 191, 25))
        self.lineEdit.setObjectName("lineEdit")
        self.sendButton = QtWidgets.QPushButton(self.centralwidget)
        self.sendButton.setGeometry(QtCore.QRect(410, 310, 89, 25))
        self.sendButton.setObjectName("sendButton")
        self.maxSlider1 = QtWidgets.QSlider(self.centralwidget)
        self.maxSlider1.setGeometry(QtCore.QRect(330, 230, 141, 25))
        self.maxSlider1.setMaximum(65535)
        self.maxSlider1.setProperty("value", 65535)
        self.maxSlider1.setOrientation(QtCore.Qt.Horizontal)
        self.maxSlider1.setObjectName("maxSlider1")
        self.label1 = QtWidgets.QLabel(self.centralwidget)
        self.label1.setGeometry(QtCore.QRect(200, 230, 140, 25))
        self.label1.setObjectName("label1")
        self.labelY = QtWidgets.QLabel(self.centralwidget)
        self.labelY.setGeometry(QtCore.QRect(50, 230, 140, 25))
        self.labelY.setObjectName("labelY")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.labelD.setText(_translate("MainWindow", "SpeedDirection"))
        self.comboBox0.setItemText(0, _translate("MainWindow", "standstill"))
        self.comboBox0.setItemText(1, _translate("MainWindow", "forward"))
        self.comboBox0.setItemText(2, _translate("MainWindow", "backward"))
        self.labelS.setText(_translate("MainWindow", "Speed"))
        self.label0.setText(_translate("MainWindow", "163.8"))
        self.lineEdit.setText(_translate("MainWindow", "can0"))
        self.lineEdit.setPlaceholderText(_translate("MainWindow", "can name"))
        self.sendButton.setText(_translate("MainWindow", "PushButton"))
        self.label1.setText(_translate("MainWindow", "327.68"))
        self.labelY.setText(_translate("MainWindow", "YawRate"))


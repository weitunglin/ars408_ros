# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'qt_designer/myWindow.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(580, 729)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.outputType_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.outputType_groupBox.setGeometry(QtCore.QRect(10, 390, 200, 100))
        self.outputType_groupBox.setTitle("")
        self.outputType_groupBox.setFlat(False)
        self.outputType_groupBox.setObjectName("outputType_groupBox")
        self.outputType_radioBtn2 = QtWidgets.QRadioButton(self.outputType_groupBox)
        self.outputType_radioBtn2.setGeometry(QtCore.QRect(30, 50, 150, 23))
        self.outputType_radioBtn2.setChecked(True)
        self.outputType_radioBtn2.setObjectName("outputType_radioBtn2")
        self.outputType_radioBtn3 = QtWidgets.QRadioButton(self.outputType_groupBox)
        self.outputType_radioBtn3.setGeometry(QtCore.QRect(30, 70, 150, 23))
        self.outputType_radioBtn3.setObjectName("outputType_radioBtn3")
        self.outputType_radioBtn1 = QtWidgets.QRadioButton(self.outputType_groupBox)
        self.outputType_radioBtn1.setGeometry(QtCore.QRect(30, 30, 150, 23))
        self.outputType_radioBtn1.setChecked(False)
        self.outputType_radioBtn1.setObjectName("outputType_radioBtn1")
        self.checkBox_4 = QtWidgets.QCheckBox(self.outputType_groupBox)
        self.checkBox_4.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.checkBox_4.setObjectName("checkBox_4")
        self.distance_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.distance_groupBox.setGeometry(QtCore.QRect(220, 10, 350, 70))
        self.distance_groupBox.setTitle("")
        self.distance_groupBox.setObjectName("distance_groupBox")
        self.checkBox_7 = QtWidgets.QCheckBox(self.distance_groupBox)
        self.checkBox_7.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.checkBox_7.setObjectName("checkBox_7")
        self.distance_spinBox = QtWidgets.QSpinBox(self.distance_groupBox)
        self.distance_spinBox.setGeometry(QtCore.QRect(30, 30, 70, 30))
        self.distance_spinBox.setMinimum(10)
        self.distance_spinBox.setMaximum(2046)
        self.distance_spinBox.setSingleStep(2)
        self.distance_spinBox.setProperty("value", 2046)
        self.distance_spinBox.setObjectName("distance_spinBox")
        self.distance_slider = QtWidgets.QSlider(self.distance_groupBox)
        self.distance_slider.setGeometry(QtCore.QRect(110, 30, 221, 30))
        self.distance_slider.setMinimum(10)
        self.distance_slider.setMaximum(2046)
        self.distance_slider.setSingleStep(2)
        self.distance_slider.setProperty("value", 2046)
        self.distance_slider.setOrientation(QtCore.Qt.Horizontal)
        self.distance_slider.setObjectName("distance_slider")
        self.sensorID_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.sensorID_groupBox.setGeometry(QtCore.QRect(10, 630, 201, 70))
        self.sensorID_groupBox.setTitle("")
        self.sensorID_groupBox.setObjectName("sensorID_groupBox")
        self.checkBox_6 = QtWidgets.QCheckBox(self.sensorID_groupBox)
        self.checkBox_6.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.checkBox_6.setObjectName("checkBox_6")
        self.sensorID_spinBox = QtWidgets.QSpinBox(self.sensorID_groupBox)
        self.sensorID_spinBox.setGeometry(QtCore.QRect(30, 30, 70, 30))
        self.sensorID_spinBox.setMinimum(0)
        self.sensorID_spinBox.setMaximum(7)
        self.sensorID_spinBox.setSingleStep(1)
        self.sensorID_spinBox.setProperty("value", 0)
        self.sensorID_spinBox.setObjectName("sensorID_spinBox")
        self.storeNVM_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.storeNVM_groupBox.setGeometry(QtCore.QRect(10, 10, 200, 80))
        self.storeNVM_groupBox.setTitle("")
        self.storeNVM_groupBox.setObjectName("storeNVM_groupBox")
        self.checkBox_0 = QtWidgets.QCheckBox(self.storeNVM_groupBox)
        self.checkBox_0.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.checkBox_0.setObjectName("checkBox_0")
        self.storeNVM_radioBtn2 = QtWidgets.QRadioButton(self.storeNVM_groupBox)
        self.storeNVM_radioBtn2.setGeometry(QtCore.QRect(30, 50, 150, 23))
        self.storeNVM_radioBtn2.setChecked(False)
        self.storeNVM_radioBtn2.setObjectName("storeNVM_radioBtn2")
        self.storeNVM_radioBtn1 = QtWidgets.QRadioButton(self.storeNVM_groupBox)
        self.storeNVM_radioBtn1.setGeometry(QtCore.QRect(30, 30, 150, 23))
        self.storeNVM_radioBtn1.setChecked(True)
        self.storeNVM_radioBtn1.setObjectName("storeNVM_radioBtn1")
        self.quality_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.quality_groupBox.setGeometry(QtCore.QRect(10, 300, 200, 80))
        self.quality_groupBox.setTitle("")
        self.quality_groupBox.setFlat(False)
        self.quality_groupBox.setObjectName("quality_groupBox")
        self.quality_radioBtn1 = QtWidgets.QRadioButton(self.quality_groupBox)
        self.quality_radioBtn1.setGeometry(QtCore.QRect(30, 30, 150, 23))
        self.quality_radioBtn1.setObjectName("quality_radioBtn1")
        self.checkBox_3 = QtWidgets.QCheckBox(self.quality_groupBox)
        self.checkBox_3.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.checkBox_3.setObjectName("checkBox_3")
        self.quality_radioBtn2 = QtWidgets.QRadioButton(self.quality_groupBox)
        self.quality_radioBtn2.setGeometry(QtCore.QRect(30, 50, 150, 23))
        self.quality_radioBtn2.setChecked(True)
        self.quality_radioBtn2.setObjectName("quality_radioBtn2")
        self.sortIndex_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.sortIndex_groupBox.setGeometry(QtCore.QRect(10, 100, 200, 100))
        self.sortIndex_groupBox.setTitle("")
        self.sortIndex_groupBox.setFlat(False)
        self.sortIndex_groupBox.setObjectName("sortIndex_groupBox")
        self.sortIndex_radioBtn2 = QtWidgets.QRadioButton(self.sortIndex_groupBox)
        self.sortIndex_radioBtn2.setGeometry(QtCore.QRect(30, 50, 150, 23))
        self.sortIndex_radioBtn2.setChecked(True)
        self.sortIndex_radioBtn2.setObjectName("sortIndex_radioBtn2")
        self.sortIndex_radioBtn3 = QtWidgets.QRadioButton(self.sortIndex_groupBox)
        self.sortIndex_radioBtn3.setGeometry(QtCore.QRect(30, 70, 150, 23))
        self.sortIndex_radioBtn3.setObjectName("sortIndex_radioBtn3")
        self.sortIndex_radioBtn1 = QtWidgets.QRadioButton(self.sortIndex_groupBox)
        self.sortIndex_radioBtn1.setGeometry(QtCore.QRect(30, 30, 150, 23))
        self.sortIndex_radioBtn1.setObjectName("sortIndex_radioBtn1")
        self.checkBox_1 = QtWidgets.QCheckBox(self.sortIndex_groupBox)
        self.checkBox_1.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.checkBox_1.setObjectName("checkBox_1")
        self.extInfo_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.extInfo_groupBox.setGeometry(QtCore.QRect(10, 210, 200, 80))
        self.extInfo_groupBox.setTitle("")
        self.extInfo_groupBox.setFlat(False)
        self.extInfo_groupBox.setObjectName("extInfo_groupBox")
        self.extInfo_radioBtn1 = QtWidgets.QRadioButton(self.extInfo_groupBox)
        self.extInfo_radioBtn1.setGeometry(QtCore.QRect(30, 30, 150, 23))
        self.extInfo_radioBtn1.setObjectName("extInfo_radioBtn1")
        self.extInfo_radioBtn2 = QtWidgets.QRadioButton(self.extInfo_groupBox)
        self.extInfo_radioBtn2.setGeometry(QtCore.QRect(30, 50, 150, 23))
        self.extInfo_radioBtn2.setChecked(True)
        self.extInfo_radioBtn2.setObjectName("extInfo_radioBtn2")
        self.checkBox_2 = QtWidgets.QCheckBox(self.extInfo_groupBox)
        self.checkBox_2.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.checkBox_2.setObjectName("checkBox_2")
        self.radarPower_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.radarPower_groupBox.setGeometry(QtCore.QRect(10, 500, 200, 121))
        self.radarPower_groupBox.setTitle("")
        self.radarPower_groupBox.setFlat(False)
        self.radarPower_groupBox.setObjectName("radarPower_groupBox")
        self.radarPower_radioBtn2 = QtWidgets.QRadioButton(self.radarPower_groupBox)
        self.radarPower_radioBtn2.setGeometry(QtCore.QRect(30, 50, 150, 23))
        self.radarPower_radioBtn2.setChecked(False)
        self.radarPower_radioBtn2.setObjectName("radarPower_radioBtn2")
        self.radarPower_radioBtn3 = QtWidgets.QRadioButton(self.radarPower_groupBox)
        self.radarPower_radioBtn3.setGeometry(QtCore.QRect(30, 70, 150, 23))
        self.radarPower_radioBtn3.setObjectName("radarPower_radioBtn3")
        self.radarPower_radioBtn1 = QtWidgets.QRadioButton(self.radarPower_groupBox)
        self.radarPower_radioBtn1.setGeometry(QtCore.QRect(30, 30, 150, 23))
        self.radarPower_radioBtn1.setChecked(True)
        self.radarPower_radioBtn1.setObjectName("radarPower_radioBtn1")
        self.checkBox_5 = QtWidgets.QCheckBox(self.radarPower_groupBox)
        self.checkBox_5.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.checkBox_5.setObjectName("checkBox_5")
        self.radarPower_radioBtn4 = QtWidgets.QRadioButton(self.radarPower_groupBox)
        self.radarPower_radioBtn4.setGeometry(QtCore.QRect(30, 90, 150, 23))
        self.radarPower_radioBtn4.setObjectName("radarPower_radioBtn4")
        self.rcsThreshold_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.rcsThreshold_groupBox.setGeometry(QtCore.QRect(220, 100, 350, 80))
        self.rcsThreshold_groupBox.setTitle("")
        self.rcsThreshold_groupBox.setObjectName("rcsThreshold_groupBox")
        self.rcsThreshold_checkBox = QtWidgets.QCheckBox(self.rcsThreshold_groupBox)
        self.rcsThreshold_checkBox.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.rcsThreshold_checkBox.setObjectName("rcsThreshold_checkBox")
        self.rcsThreshold_radioBtn2 = QtWidgets.QRadioButton(self.rcsThreshold_groupBox)
        self.rcsThreshold_radioBtn2.setGeometry(QtCore.QRect(30, 50, 200, 23))
        self.rcsThreshold_radioBtn2.setChecked(False)
        self.rcsThreshold_radioBtn2.setObjectName("rcsThreshold_radioBtn2")
        self.rcsThreshold_radioBtn1 = QtWidgets.QRadioButton(self.rcsThreshold_groupBox)
        self.rcsThreshold_radioBtn1.setGeometry(QtCore.QRect(30, 30, 200, 23))
        self.rcsThreshold_radioBtn1.setChecked(True)
        self.rcsThreshold_radioBtn1.setObjectName("rcsThreshold_radioBtn1")
        self.invalidClusters_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.invalidClusters_groupBox.setGeometry(QtCore.QRect(220, 190, 350, 201))
        self.invalidClusters_groupBox.setTitle("")
        self.invalidClusters_groupBox.setObjectName("invalidClusters_groupBox")
        self.invalidClusters_checkBox = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.invalidClusters_checkBox.setObjectName("invalidClusters_checkBox")
        self.invalidClusters_checkBox0 = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox0.setGeometry(QtCore.QRect(30, 30, 240, 23))
        self.invalidClusters_checkBox0.setObjectName("invalidClusters_checkBox0")
        self.invalidClusters_checkBox1 = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox1.setGeometry(QtCore.QRect(30, 50, 240, 23))
        self.invalidClusters_checkBox1.setObjectName("invalidClusters_checkBox1")
        self.invalidClusters_checkBox2 = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox2.setGeometry(QtCore.QRect(30, 70, 240, 23))
        self.invalidClusters_checkBox2.setObjectName("invalidClusters_checkBox2")
        self.invalidClusters_checkBox3 = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox3.setGeometry(QtCore.QRect(30, 90, 240, 23))
        self.invalidClusters_checkBox3.setObjectName("invalidClusters_checkBox3")
        self.invalidClusters_checkBox4 = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox4.setGeometry(QtCore.QRect(30, 110, 240, 23))
        self.invalidClusters_checkBox4.setObjectName("invalidClusters_checkBox4")
        self.invalidClusters_checkBox5 = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox5.setGeometry(QtCore.QRect(30, 130, 240, 23))
        self.invalidClusters_checkBox5.setObjectName("invalidClusters_checkBox5")
        self.invalidClusters_checkBox7 = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox7.setGeometry(QtCore.QRect(30, 170, 240, 23))
        self.invalidClusters_checkBox7.setObjectName("invalidClusters_checkBox7")
        self.invalidClusters_checkBox6 = QtWidgets.QCheckBox(self.invalidClusters_groupBox)
        self.invalidClusters_checkBox6.setGeometry(QtCore.QRect(30, 150, 240, 23))
        self.invalidClusters_checkBox6.setObjectName("invalidClusters_checkBox6")
        self.sendButton = QtWidgets.QPushButton(self.centralwidget)
        self.sendButton.setGeometry(QtCore.QRect(450, 670, 120, 30))
        self.sendButton.setObjectName("sendButton")
        self.canName_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.canName_lineEdit.setGeometry(QtCore.QRect(220, 630, 350, 30))
        self.canName_lineEdit.setObjectName("canName_lineEdit")
        self.textEdit = QtWidgets.QTextEdit(self.centralwidget)
        self.textEdit.setGeometry(QtCore.QRect(220, 490, 350, 131))
        self.textEdit.setReadOnly(True)
        self.textEdit.setPlaceholderText("")
        self.textEdit.setObjectName("textEdit")
        self.rcsThreshold_groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.rcsThreshold_groupBox_2.setGeometry(QtCore.QRect(220, 400, 350, 80))
        self.rcsThreshold_groupBox_2.setTitle("")
        self.rcsThreshold_groupBox_2.setObjectName("rcsThreshold_groupBox_2")
        self.ctrlRelay_checkBox = QtWidgets.QCheckBox(self.rcsThreshold_groupBox_2)
        self.ctrlRelay_checkBox.setGeometry(QtCore.QRect(10, 0, 150, 20))
        self.ctrlRelay_checkBox.setObjectName("ctrlRelay_checkBox")
        self.ctrlRelay_radioBtn2 = QtWidgets.QRadioButton(self.rcsThreshold_groupBox_2)
        self.ctrlRelay_radioBtn2.setGeometry(QtCore.QRect(30, 50, 200, 23))
        self.ctrlRelay_radioBtn2.setChecked(False)
        self.ctrlRelay_radioBtn2.setObjectName("ctrlRelay_radioBtn2")
        self.ctrlRelay_radioBtn1 = QtWidgets.QRadioButton(self.rcsThreshold_groupBox_2)
        self.ctrlRelay_radioBtn1.setGeometry(QtCore.QRect(30, 30, 200, 23))
        self.ctrlRelay_radioBtn1.setChecked(True)
        self.ctrlRelay_radioBtn1.setObjectName("ctrlRelay_radioBtn1")
        self.setButton = QtWidgets.QPushButton(self.centralwidget)
        self.setButton.setGeometry(QtCore.QRect(220, 670, 120, 30))
        self.setButton.setObjectName("setButton")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Set ARS408 message #200"))
        self.outputType_radioBtn2.setText(_translate("MainWindow", "send objects"))
        self.outputType_radioBtn3.setText(_translate("MainWindow", "send clusters"))
        self.outputType_radioBtn1.setText(_translate("MainWindow", "none"))
        self.checkBox_4.setText(_translate("MainWindow", "Send Output Type"))
        self.checkBox_7.setText(_translate("MainWindow", "Max Distance"))
        self.checkBox_6.setText(_translate("MainWindow", "Sensor ID"))
        self.checkBox_0.setText(_translate("MainWindow", "Store In NVM"))
        self.storeNVM_radioBtn2.setText(_translate("MainWindow", "active"))
        self.storeNVM_radioBtn1.setText(_translate("MainWindow", "inactive"))
        self.quality_radioBtn1.setText(_translate("MainWindow", "inactive"))
        self.checkBox_3.setText(_translate("MainWindow", "Send Quality"))
        self.quality_radioBtn2.setText(_translate("MainWindow", "active"))
        self.sortIndex_radioBtn2.setText(_translate("MainWindow", "sorted by range"))
        self.sortIndex_radioBtn3.setText(_translate("MainWindow", "sorted by RCS"))
        self.sortIndex_radioBtn1.setText(_translate("MainWindow", "no sorting"))
        self.checkBox_1.setText(_translate("MainWindow", "Sort Index"))
        self.extInfo_radioBtn1.setText(_translate("MainWindow", "inactive"))
        self.extInfo_radioBtn2.setText(_translate("MainWindow", "active"))
        self.checkBox_2.setText(_translate("MainWindow", "Send Ext Info"))
        self.radarPower_radioBtn2.setText(_translate("MainWindow", "-3dB Tx gain"))
        self.radarPower_radioBtn3.setText(_translate("MainWindow", "-6dB Tx gain"))
        self.radarPower_radioBtn1.setText(_translate("MainWindow", "Standard"))
        self.checkBox_5.setText(_translate("MainWindow", "Radar Power"))
        self.radarPower_radioBtn4.setText(_translate("MainWindow", "-9dB Tx gain"))
        self.rcsThreshold_checkBox.setText(_translate("MainWindow", "RCS Threshold"))
        self.rcsThreshold_radioBtn2.setText(_translate("MainWindow", "High sensitivity"))
        self.rcsThreshold_radioBtn1.setText(_translate("MainWindow", "Standard"))
        self.invalidClusters_checkBox.setText(_translate("MainWindow", "Invalid Clusters"))
        self.invalidClusters_checkBox0.setText(_translate("MainWindow", "Disable invalid cluster"))
        self.invalidClusters_checkBox1.setText(_translate("MainWindow", "Enable all invalid clusters"))
        self.invalidClusters_checkBox2.setText(_translate("MainWindow", "Enable low RCS dynamic"))
        self.invalidClusters_checkBox3.setText(_translate("MainWindow", "Enable low RCS static"))
        self.invalidClusters_checkBox4.setText(_translate("MainWindow", "Enable invalid range rate"))
        self.invalidClusters_checkBox5.setText(_translate("MainWindow", "Enable range <1m"))
        self.invalidClusters_checkBox7.setText(_translate("MainWindow", "Enable wrapped stationary"))
        self.invalidClusters_checkBox6.setText(_translate("MainWindow", "Enable ego mirror"))
        self.sendButton.setText(_translate("MainWindow", "Send"))
        self.canName_lineEdit.setText(_translate("MainWindow", "can0"))
        self.canName_lineEdit.setPlaceholderText(_translate("MainWindow", "Can Name"))
        self.textEdit.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.ctrlRelay_checkBox.setText(_translate("MainWindow", "Ctrl Relay"))
        self.ctrlRelay_radioBtn2.setText(_translate("MainWindow", "active"))
        self.ctrlRelay_radioBtn1.setText(_translate("MainWindow", "inactive"))
        self.setButton.setText(_translate("MainWindow", "Set up Radar"))


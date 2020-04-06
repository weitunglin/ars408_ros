import os, sys
from PyQt5 import QtWidgets, QtGui, QtCore
from qt_uic.myWindow import Ui_MainWindow

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # weight setting
        self.ui.distance_slider.valueChanged.connect(self.distanceWeight_valueChanged)
        self.ui.distance_spinBox.valueChanged.connect(self.distanceWeight_valueChanged)
        self.ui.invalidClusters_checkBox0.stateChanged.connect(self.invalidClusters_checkBox_Changed)
        self.ui.sendButton.clicked.connect(self.sendBtn_clicked)

    def sendBtn_clicked(self):
        sendcode = [0, 0, 0, 0, 0, 0, 0, 0]

        # store NVM
        if self.ui.checkBox_0.isChecked():
            sendcode[0] += 0b10000000
            if self.ui.storeNVM_radioBtn2.isChecked():
                sendcode[5] += 0b10000000

        # sort index
        if self.ui.checkBox_1.isChecked():
            sendcode[0] += 0b01000000
            if self.ui.sortIndex_radioBtn2.isChecked():
                sendcode[5] += 0b00100000
            if self.ui.sortIndex_radioBtn3.isChecked():
                sendcode[5] += 0b01000000
        
        # send ext info
        if self.ui.checkBox_2.isChecked():
            sendcode[0] += 0b00100000
            if self.ui.extInfo_radioBtn2.isChecked():
                sendcode[5] += 0b00001000

        # send quality
        if self.ui.checkBox_3.isChecked():
            sendcode[0] += 0b00010000
            if self.ui.quality_radioBtn2.isChecked():
                sendcode[5] += 0b00000100

        # output type
        if self.ui.checkBox_4.isChecked():
            sendcode[0] += 0b00001000
            if self.ui.outputType_radioBtn2.isChecked():
                sendcode[4] += 0b00001000
            if self.ui.outputType_radioBtn3.isChecked():
                sendcode[4] += 0b00010000

        # radar power
        if self.ui.checkBox_5.isChecked():
            sendcode[0] += 0b00000100
            if self.ui.radarPower_radioBtn2.isChecked():
                sendcode[4] += 0b00100000
            if self.ui.outputType_radioBtn3.isChecked():
                sendcode[4] += 0b01000000
            if self.ui.outputType_radioBtn3.isChecked():
                sendcode[4] += 0b01100000

        # sensor ID
        if self.ui.checkBox_6.isChecked():
            sendcode[0] += 0b00000010
            sendcode[4] += self.ui.sensorID_spinBox.value()

        # distance
        if self.ui.checkBox_7.isChecked():
            sendcode[0] += 0b00000001
            sendcode[1] = ((self.ui.distance_spinBox.value() - 10) // 2) >> 2
            sendcode[2] = (((self.ui.distance_spinBox.value() - 10) // 2) & 0b11) << 6

        # RCS threshold
        if self.ui.rcsThreshold_checkBox.isChecked():
            sendcode[6] += 0b00000001
            if self.ui.rcsThreshold_radioBtn2.isChecked():
                sendcode[6] += 0b00000100

        # invalid clusters
        if self.ui.invalidClusters_checkBox.isChecked():
            sendcode[6] += 0b00010000
            if self.ui.invalidClusters_checkBox0.isChecked():
                sendcode[7] = 0x00
            else:
                sendcode[7] += 0x01 if self.ui.invalidClusters_checkBox1.isChecked() else 0
                sendcode[7] += 0x02 if self.ui.invalidClusters_checkBox2.isChecked() else 0
                sendcode[7] += 0x04 if self.ui.invalidClusters_checkBox3.isChecked() else 0
                sendcode[7] += 0x08 if self.ui.invalidClusters_checkBox4.isChecked() else 0
                sendcode[7] += 0x10 if self.ui.invalidClusters_checkBox5.isChecked() else 0
                sendcode[7] += 0x20 if self.ui.invalidClusters_checkBox6.isChecked() else 0
                sendcode[7] += 0x40 if self.ui.invalidClusters_checkBox7.isChecked() else 0

        # Ctrl Relay
        if self.ui.ctrlRelay_checkBox.isChecked():
            sendcode[5] += 0b00000001
            if self.ui.ctrlRelay_radioBtn2.isChecked():
                sendcode[5] += 0b00000010

        # Code Process
        sendcodeStr = ""
        for i in sendcode:
            sendcodeStr += "{0:02x}".format(i)

        sendText = "cansend " + self.ui.lineEdit.text() + " 200#" + sendcodeStr
        self.ui.textEdit.insertPlainText(sendText + "\n")
        self.ui.textEdit.moveCursor(QtGui.QTextCursor.End)

        os.popen(sendText)

        # self.ui.textEdit.moveCursor(QtGui.QTextCursor.End)

    def distanceWeight_valueChanged(self, i):
        if i % 2 is not 0:
            self.ui.distance_slider.setValue(i + 1)
            self.ui.distance_spinBox.setValue(i + 1)
        else:
            self.ui.distance_slider.setValue(i)
            self.ui.distance_spinBox.setValue(i)

    def invalidClusters_checkBox_Changed(self):
        if self.ui.invalidClusters_checkBox0.isChecked():
            self.ui.invalidClusters_checkBox1.setCheckState(False)
            self.ui.invalidClusters_checkBox2.setCheckState(False)
            self.ui.invalidClusters_checkBox3.setCheckState(False)
            self.ui.invalidClusters_checkBox4.setCheckState(False)
            self.ui.invalidClusters_checkBox5.setCheckState(False)
            self.ui.invalidClusters_checkBox6.setCheckState(False)
            self.ui.invalidClusters_checkBox7.setCheckState(False)
            self.ui.invalidClusters_checkBox1.setDisabled(True)
            self.ui.invalidClusters_checkBox2.setDisabled(True)
            self.ui.invalidClusters_checkBox3.setDisabled(True)
            self.ui.invalidClusters_checkBox4.setDisabled(True)
            self.ui.invalidClusters_checkBox5.setDisabled(True)
            self.ui.invalidClusters_checkBox6.setDisabled(True)
            self.ui.invalidClusters_checkBox7.setDisabled(True)
        else:
            self.ui.invalidClusters_checkBox1.setEnabled(True)
            self.ui.invalidClusters_checkBox2.setEnabled(True)
            self.ui.invalidClusters_checkBox3.setEnabled(True)
            self.ui.invalidClusters_checkBox4.setEnabled(True)
            self.ui.invalidClusters_checkBox5.setEnabled(True)
            self.ui.invalidClusters_checkBox6.setEnabled(True)
            self.ui.invalidClusters_checkBox7.setEnabled(True)


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
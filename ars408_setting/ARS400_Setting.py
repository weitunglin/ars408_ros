import os, sys, subprocess
from functools import partial
from PyQt5 import QtWidgets, QtGui, QtCore
from qt_uic.collisionWindow import Ui_MainWindow

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.checkboxs = [
            self.ui.checkBox0,
            self.ui.checkBox1,
            self.ui.checkBox2,
            self.ui.checkBox3,
        ]

        self.labels = [
            self.ui.label0,
        ]

        self.maxSliders = [
            self.ui.maxSlider0,
        ]

        self.res = [
            0.1,
        ]

        self.min = [
            0,
        ]
        
        self.maxSliders[0].valueChanged.connect(partial(self.slider_maxValueChanged, 0))
        self.ui.sendButton.clicked.connect(self.sendbutton_clicked)

    def slider_maxValueChanged(self, num, i):
        text = self.labels[num].text()
        minString = text[0:text.find("~") - 1]
        max = round(self.maxSliders[num].value() * self.res[num] + self.min[num],3)
        text = minString + " ~ " + str(max)
        self.labels[num].setText(text)


    def sendbutton_clicked(self):
        sendcode = [0, 0]

        #CollDetCfg_WarningReset
        if self.checkboxs[0].isChecked():
            sendcode[0] += 0b00000001

        #CollDetCfg_Activation
        if self.checkboxs[1].isChecked():
            sendcode[0] += 0b00000010

        #CollDetCfg_ClearRegions
        if self.checkboxs[2].isChecked():
            sendcode[0] += 0b10000000

        #CollDetCfg_MinTime_valid
        if self.checkboxs[3].isChecked():
            sendcode[0] += 0b00001000
            sendcode[1] = self.maxSliders[0].value()

        # Code Process
        sendcodeStr = ""
        for i in sendcode:
            sendcodeStr += "{0:02x}".format(i)
        sendText = "cansend " + self.ui.lineEdit.text() + " 400#" + sendcodeStr
        os.popen(sendText)

        #print(sendcodeStr)


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
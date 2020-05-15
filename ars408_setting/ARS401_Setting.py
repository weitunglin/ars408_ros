#! /usr/bin/env python3

import os, sys, subprocess
from functools import partial
from PyQt5 import QtWidgets, QtGui, QtCore
from qt_uic.collisionWindow2 import Ui_MainWindow

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
            self.ui.checkBox4,
            self.ui.checkBox5,
            self.ui.checkBox6,
            self.ui.checkBox7,
            self.ui.checkBox8,
            self.ui.checkBox9,
        ]

        self.labels = [
            self.ui.label0,
            self.ui.label1,
            self.ui.label2,
            self.ui.label3,
        ]

        self.maxSliders = [
            self.ui.maxSlider0,
            self.ui.maxSlider1,
            self.ui.maxSlider2,
            self.ui.maxSlider3,
        ]

        self.res = [
            0.2,
            0.2,
            0.2,
            0.2,
        ]

        self.min = [
            -500,
            -204.6,
            -500,
            -204.6,
        ]


        for i in range(0, 4):
            self.maxSliders[i].valueChanged.connect(partial(self.slider_maxValueChanged, i))

        self.ui.sendButton.clicked.connect(self.sendbutton_clicked)

    def slider_maxValueChanged(self, num, i):
        text = self.labels[num].text()
        max = round(self.maxSliders[num].value() * self.res[num] + self.min[num],3)
        self.labels[num].setText(str(max))

    def sendbutton_clicked(self):
        for i in range(0, 8):
            if self.checkboxs[i].isChecked():
                sendcode = [0, 0, 0, 0, 0, 0, 0, 0]
                sendcode[1] += i  # store index

                # Coordinates_valid
                if self.checkboxs[8].isChecked():
                    sendcode[0] += 0b00000010

                # store Activation
                if self.checkboxs[9].isChecked():
                    sendcode[0] += 0b00000100
                    # store Coordinates
                    sendcode[2] += self.maxSliders[0].value() >> 5
                    sendcode[3] += ( self.maxSliders[0].value() % 32 ) << 3

                    sendcode[3] += self.maxSliders[1].value() >> 8
                    sendcode[4] += self.maxSliders[1].value() % 256

                    sendcode[5] += self.maxSliders[2].value() >> 5
                    sendcode[6] += ( self.maxSliders[2].value() % 32 ) << 3

                    sendcode[6] += self.maxSliders[3].value() >> 8
                    sendcode[7] += self.maxSliders[3].value() % 256

                # Code Process
                sendcodeStr = ""
                for i in sendcode:
                    sendcodeStr += "{0:02x}".format(i)
                sendText = "cansend " + self.ui.lineEdit.text() + " 401#" + sendcodeStr
                os.popen(sendText)

                #print(sendcodeStr)



if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
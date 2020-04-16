import os, sys, subprocess
from functools import partial
from PyQt5 import QtWidgets, QtGui, QtCore
from qt_uic.speedWindow import Ui_MainWindow

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.comboboxs = [
            self.ui.comboBox0,
        ]

        self.labels = [
            self.ui.label0,
            self.ui.label1,
        ]

        self.maxSliders = [
            self.ui.maxSlider0,
            self.ui.maxSlider1,
        ]

        self.res = [
            0.02,
            0.01,
        ]

        self.min = [
            0,
            -327.68,
        ]
        
        for i in range(0, 2):
            self.maxSliders[i].valueChanged.connect(partial(self.slider_maxValueChanged, i))
        self.ui.sendButton.clicked.connect(self.sendbutton_clicked)

    def slider_maxValueChanged(self, num, i):
        max = round(self.maxSliders[num].value() * self.res[num] + self.min[num],3)
        self.labels[num].setText(str(max))

    def sendbutton_clicked(self):
        sendcode = [0, 0]  #300

        # store SpeedDirection
        if self.comboboxs[0].currentIndex() == 1:
            sendcode[0] += 0b01000000
        elif self.comboboxs[0].currentIndex() == 2:
            sendcode[0] += 0b10000000

        # store Speed
        sendcode[0] += self.maxSliders[0].value() >> 8
        sendcode[1] += self.maxSliders[0].value() % 256

        # Code Process
        sendcodeStr = ""
        for i in sendcode:
            sendcodeStr += "{0:02x}".format(i)
        sendText = "cansend " + self.ui.lineEdit.text() + " 300#" + sendcodeStr
        os.popen(sendText)

        sendcode2 = [0, 0]  #301

        # store Speed
        sendcode2[0] += self.maxSliders[1].value() >> 8
        sendcode2[1] += self.maxSliders[1].value() % 256

        # Code Process
        sendcodeStr2 = ""
        for i in sendcode2:
            sendcodeStr2 += "{0:02x}".format(i)
        sendText2 = "cansend " + self.ui.lineEdit.text() + " 301#" + sendcodeStr2
        os.popen(sendText2)

        print(sendcodeStr)
        print(sendcodeStr2)



if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
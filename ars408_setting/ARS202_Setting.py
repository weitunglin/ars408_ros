#! /usr/bin/env python3

import os, sys, subprocess
from functools import partial
from PyQt5 import QtWidgets, QtGui, QtCore
from qt_uic.filterWindow import Ui_MainWindow

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
            self.ui.checkBoxA,
            self.ui.checkBoxB,
            self.ui.checkBoxC,
            self.ui.checkBoxD,
            self.ui.checkBoxE,
            self.ui.checkBoxF,
        ]

        self.comboboxs = [
            self.ui.comboBox0,
            self.ui.comboBox1,
            self.ui.comboBox2,
            self.ui.comboBox3,
            self.ui.comboBox4,
            self.ui.comboBox5,
            self.ui.comboBox6,
            self.ui.comboBox7,
            self.ui.comboBox8,
            self.ui.comboBox9,
            self.ui.comboBoxA,
            self.ui.comboBoxB,
            self.ui.comboBoxC,
            self.ui.comboBoxD,
            self.ui.comboBoxE,
            self.ui.comboBoxF,
        ]

        self.labels = [
            self.ui.label0,
            self.ui.label1,
            self.ui.label2,
            self.ui.label3,
            self.ui.label4,
            self.ui.label5,
            self.ui.label6,
            self.ui.label7,
            self.ui.label8,
            self.ui.label9,
            self.ui.labelA,
            self.ui.labelB,
            self.ui.labelC,
            self.ui.labelD,
            self.ui.labelE,
            self.ui.labelF,
        ]

        self.minSliders = [
            None,
            self.ui.minSlider1,
            self.ui.minSlider2,
            self.ui.minSlider3,
            self.ui.minSlider4,
            self.ui.minSlider5,
            self.ui.minSlider6,
            self.ui.minSlider7,
            self.ui.minSlider8,
            self.ui.minSlider9,
            self.ui.minSliderA,
            self.ui.minSliderB,
            self.ui.minSliderC,
            self.ui.minSliderD,
            self.ui.minSliderE,
            None,
        ]

        self.maxSliders = [
            self.ui.maxSlider0,
            self.ui.maxSlider1,
            self.ui.maxSlider2,
            self.ui.maxSlider3,
            self.ui.maxSlider4,
            self.ui.maxSlider5,
            self.ui.maxSlider6,
            self.ui.maxSlider7,
            self.ui.maxSlider8,
            self.ui.maxSlider9,
            self.ui.maxSliderA,
            self.ui.maxSliderB,
            self.ui.maxSliderC,
            self.ui.maxSliderD,
            self.ui.maxSliderE,
            self.ui.maxSliderF,
        ]

        self.res = [
            1,
            0.1,
            0.025,
            0.0315,
            0.0315,
            0.025,
            0.1,
            0.025,
            1,
            0.2,
            0.2,
            0.0315,
            0.0315,
            0.0315,
            0.0315,
            1,
        ]

        self.min = [
            0,
            0,
            -50,
            0,
            0,
            -50,
            0,
            0,
            0,
            -409.5,
            -500,
            0,
            0,
            0,
            0,
            0,
        ]


        for i in range(0, 16):
            self.maxSliders[i].valueChanged.connect(partial(self.slider_maxValueChanged, i))

            if (i is not 0) and (i is not 15):
                self.minSliders[i].valueChanged.connect(partial(self.slider_minValueChanged, i))

        self.ui.sendButton.clicked.connect(self.sendbutton_clicked)

    def slider_maxValueChanged(self, num, i):
        text = self.labels[num].text()
        minString = text[0:text.find("~") - 1]
        if (num is not 0) and (num is not 15) and self.minSliders[num].value() >= self.maxSliders[num].value():
            self.maxSliders[num].setValue(self.minSliders[num].value())
        max = round(self.maxSliders[num].value() * self.res[num] + self.min[num],3)
        text = minString + " ~ " + str(max)
        self.labels[num].setText(text)

    def slider_minValueChanged(self, num, i):
        text = self.labels[num].text()
        maxString = text[text.find("~") + 2:]
        if self.minSliders[num].value() >= self.maxSliders[num].value():
            self.minSliders[num].setValue(self.maxSliders[num].value())
        min = round(self.minSliders[num].value() * self.res[num] + self.min[num],3)
        text = str(min) + " ~ " + maxString
        self.labels[num].setText(text)


    def sendbutton_clicked(self):
        for i in range(0, 16):
            if self.checkboxs[i].isChecked():
                sendcode = [0, 0, 0, 0, 0]
                sendcode[0] += 0b00000010  # Valid
                sendcode[0] += i << 3  # store index

                # Object or Cluster
                if self.ui.radioButton_object.isChecked():
                    sendcode[0] += 0b10000000

                # store active
                if self.comboboxs[i].currentIndex() == 1:
                    sendcode[0] += 0b00000100

                # store min
                if (i is not 0) and (i is not 15):
                    sendcode[1] += self.minSliders[i].value() >> 8
                    sendcode[2] += self.minSliders[i].value() % 256

                # store max
                sendcode[3] += self.maxSliders[i].value() >> 8
                sendcode[4] += self.maxSliders[i].value() % 256


                # Code Process
                sendcodeStr = ""
                for i in sendcode:
                    sendcodeStr += "{0:02x}".format(i)
                sendText = "cansend " + self.ui.lineEdit.text() + " 202#" + sendcodeStr
                os.popen(sendText)
                print(sendText)


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
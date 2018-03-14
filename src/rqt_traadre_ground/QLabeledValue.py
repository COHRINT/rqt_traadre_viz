#!/usr/bin/python2

'''
Class to have a QLabel with a sunken QLabel next to it so that I can have labelled data values

'''
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class QLabeledValue(QWidget):
    def __init__(self, labelText):
        super(QLabeledValue, self).__init__()
        self.label = QLabel(labelText)
        self.label.setAlignment(Qt.AlignRight)

        self.value = QLabel()
        self.value.setFrameShape(QFrame.Panel)
        self.value.setFrameShadow(QFrame.Sunken)
        self.value.setLineWidth(1)
       
        self.layout = QHBoxLayout()

        self.layout.addWidget(self.label)
        self.layout.addWidget(self.value)
        self.setLayout(self.layout)

    def updateValue(self, value):
        self.value.setText(str(value))

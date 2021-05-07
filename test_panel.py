# Using the designer, it makes an interconnected web of layouts much simpler to do.
# in virtual environment 
# pyqt5-tools designer
# pyuic5 -x your_ui.ui -o your_output.py

from PyQt5 import QtCore, QtWidgets 
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QMessageBox

import time
import serial
import struct
from math import cos, sin, atan2, degrees

import gpiozero  # may need this for D.OUT
from gpiozero import Button
import RPi.GPIO as GPIO
# may want to change buttons under list to a label to indicate state


class Ui_Goomba(object):
    # colors
    background = "background-color:rgba(52, 73, 94,1.0)"  #2C3E50
    light_brown = "color:rgba(156,118,54,156);"  #9C7636
    brown = "color:rgba(79,66,43,79)"  #4F422B
    light_blue = "color:rgba(70,113,156,156)"  #46719C
    gray_blue = "color:rgba(101,128,156,156)"  #65809C
    red = "color:rgba(201,74,50,201)"  #C94A32
    text = "color:rgb(236, 240, 241)"
    green = "color:rgba(107,179,89,70)"  #6BB359

    pwr_idle = "QPushButton {background-color:rgb(26, 188, 156); color:rgb(236, 240, 241)}"
    run = "QPushButton {background-color:rgb(46, 204, 113); color:rgb(236, 240, 241)}"
    pause = "QPushButton {background-color:rgb(231, 76, 60); color:rgb(236, 240, 241)}"
    pwr_icon = 'power_icon.png'
    radar_icon = "radar.png"

    last = False
    pwr_state = ""
    encL_now = 0
    encR_now = 0
    encL_prev = 0
    encR_prev = 0
    ang_prev = 0
    ang_now = 0
    
    def __init__(self, nRF):  # should allow us to pass arguements into class
        super().__init__()
        self.nRF = nRF

    def setupUi(self, Goomba):
        Goomba.setObjectName("Goomba")
        Goomba.resize(739, 550)
        Goomba.setMinimumSize(QtCore.QSize(600, 450))
        Goomba.setMaximumSize(QtCore.QSize(1920, 1080))
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        font.setItalic(True)
        Goomba.setWindowIcon(QIcon("goomba_icon.png"))
        Goomba.setFont(font)
        Goomba.setWindowTitle("Goomba_GUI")
        Goomba.setStyleSheet("QWidget {background-color:rgba(52, 73, 94,1.0)}\n"
        "QLabel {\n"
        "color:rgb(236, 240, 241);\n"
        "}")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(Goomba)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_2 = QtWidgets.QLabel(Goomba)
        self.label_2.setMinimumSize(QtCore.QSize(0, 0))
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_4.addWidget(self.label_2, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignBottom)
        self.line_17 = QtWidgets.QFrame(Goomba)
        self.line_17.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_17.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_17.setObjectName("line_17")
        self.verticalLayout_4.addWidget(self.line_17)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.line_7 = QtWidgets.QFrame(Goomba)
        self.line_7.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_7.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_7.setObjectName("line_7")
        self.gridLayout.addWidget(self.line_7, 1, 2, 1, 1)
        self.line_10 = QtWidgets.QFrame(Goomba)
        self.line_10.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_10.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_10.setObjectName("line_10")
        self.gridLayout.addWidget(self.line_10, 1, 1, 1, 1)
        self.line_9 = QtWidgets.QFrame(Goomba)
        self.line_9.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_9.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_9.setObjectName("line_9")
        self.gridLayout.addWidget(self.line_9, 1, 4, 1, 1)
        self.line_11 = QtWidgets.QFrame(Goomba)
        self.line_11.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_11.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_11.setObjectName("line_11")
        self.gridLayout.addWidget(self.line_11, 0, 3, 1, 1)
        self.line_12 = QtWidgets.QFrame(Goomba)
        self.line_12.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_12.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_12.setObjectName("line_12")
        self.gridLayout.addWidget(self.line_12, 2, 3, 1, 1)
        self.line_8 = QtWidgets.QFrame(Goomba)
        self.line_8.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_8.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_8.setObjectName("line_8")
        self.gridLayout.addWidget(self.line_8, 1, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 0, 0, 1, 1, QtCore.Qt.AlignHCenter)
        self.label_8 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.gridLayout.addWidget(self.label_8, 2, 4, 1, 1, QtCore.Qt.AlignHCenter)
        self.label_7 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 2, 2, 1, 1, QtCore.Qt.AlignHCenter)
        self.label_6 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 2, 0, 1, 1, QtCore.Qt.AlignHCenter)
        self.label_5 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 0, 4, 1, 1, QtCore.Qt.AlignHCenter)
        self.label_4 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 0, 2, 1, 1, QtCore.Qt.AlignHCenter)
        self.line_13 = QtWidgets.QFrame(Goomba)
        self.line_13.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_13.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_13.setObjectName("line_13")
        self.gridLayout.addWidget(self.line_13, 0, 1, 1, 1)
        self.line_14 = QtWidgets.QFrame(Goomba)
        self.line_14.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_14.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_14.setObjectName("line_14")
        self.gridLayout.addWidget(self.line_14, 2, 1, 1, 1)
        self.line_15 = QtWidgets.QFrame(Goomba)
        self.line_15.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_15.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_15.setObjectName("line_15")
        self.gridLayout.addWidget(self.line_15, 1, 3, 1, 1)
        self.horizontalLayout.addLayout(self.gridLayout)
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        self.verticalLayout.addLayout(self.verticalLayout_4)
        self.line_2 = QtWidgets.QFrame(Goomba)
        font = QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.line_2.setFont(font)
        self.line_2.setLineWidth(10)
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.verticalLayout.addWidget(self.line_2)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_5.addItem(spacerItem1)
        self.label_9 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.verticalLayout_5.addWidget(self.label_9, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignBottom)
        self.line_5 = QtWidgets.QFrame(Goomba)
        self.line_5.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_5.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_5.setObjectName("line_5")
        self.verticalLayout_5.addWidget(self.line_5)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.line_6 = QtWidgets.QFrame(Goomba)
        self.line_6.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_6.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_6.setObjectName("line_6")
        self.gridLayout_2.addWidget(self.line_6, 0, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(Goomba)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_10.sizePolicy().hasHeightForWidth())
        self.label_10.setSizePolicy(sizePolicy)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_10.setFont(font)
        self.label_10.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_10.setAutoFillBackground(False)
        self.label_10.setStyleSheet("")
        self.label_10.setScaledContents(False)
        self.label_10.setObjectName("label_10")
        self.gridLayout_2.addWidget(self.label_10, 0, 0, 1, 1, QtCore.Qt.AlignHCenter)
        self.label_11 = QtWidgets.QLabel(Goomba)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_11.sizePolicy().hasHeightForWidth())
        self.label_11.setSizePolicy(sizePolicy)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_11.setFont(font)
        self.label_11.setAutoFillBackground(False)
        self.label_11.setStyleSheet("background-color:rgba(201,74,50,201)")
        self.label_11.setObjectName("label_11")
        self.gridLayout_2.addWidget(self.label_11, 0, 2, 1, 1, QtCore.Qt.AlignHCenter)
        self.verticalLayout_5.addLayout(self.gridLayout_2)
        self.verticalLayout.addLayout(self.verticalLayout_5)
        self.line_3 = QtWidgets.QFrame(Goomba)
        self.line_3.setLineWidth(10)
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.verticalLayout.addWidget(self.line_3)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem2)
        self.label_18 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_18.setFont(font)
        self.label_18.setObjectName("label_18")
        self.verticalLayout.addWidget(self.label_18, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignBottom)
        self.line_19 = QtWidgets.QFrame(Goomba)
        self.line_19.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_19.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_19.setObjectName("line_19")
        self.verticalLayout.addWidget(self.line_19)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_17 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_17.setFont(font)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_5.addWidget(self.label_17, 0, QtCore.Qt.AlignRight)
        self.label_16 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_16.setFont(font)
        self.label_16.setObjectName("label_16")
        self.horizontalLayout_5.addWidget(self.label_16)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.line_18 = QtWidgets.QFrame(Goomba)
        self.line_18.setLineWidth(12)
        self.line_18.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_18.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_18.setObjectName("line_18")
        self.verticalLayout.addWidget(self.line_18)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem3)
        self.label = QtWidgets.QLabel(Goomba)
        self.label.setMaximumSize(QtCore.QSize(16777215, 40))
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label, 0, QtCore.Qt.AlignHCenter)
        self.line_16 = QtWidgets.QFrame(Goomba)
        self.line_16.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_16.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_16.setObjectName("line_16")
        self.verticalLayout.addWidget(self.line_16)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_12 = QtWidgets.QLabel(Goomba)
        self.label_12.setMaximumSize(QtCore.QSize(16777215, 40))
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_12.setFont(font)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_4.addWidget(self.label_12, 0, QtCore.Qt.AlignRight)
        self.label_15 = QtWidgets.QLabel(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(12)
        self.label_15.setFont(font)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_4.addWidget(self.label_15)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        spacerItem4 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem4)
        self.horizontalLayout_3.addLayout(self.verticalLayout)
        self.line = QtWidgets.QFrame(Goomba)
        self.line.setLineWidth(10)
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.horizontalLayout_3.addWidget(self.line)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_13 = QtWidgets.QLabel(Goomba)
        self.label_13.setMaximumSize(QtCore.QSize(16777215, 25))
        font = QFont()
        font.setFamily("Nirmala UI")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_13.setFont(font)
        self.label_13.setObjectName("label_13")
        self.verticalLayout_6.addWidget(self.label_13, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignBottom)
        self.listWidget = QtWidgets.QListWidget(Goomba)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.listWidget.sizePolicy().hasHeightForWidth())
        self.listWidget.setSizePolicy(sizePolicy)
        self.listWidget.setMinimumSize(QtCore.QSize(200, 200))
        self.listWidget.setMaximumSize(QtCore.QSize(0, 0))
        font = QFont()
        font.setPointSize(12)
        self.listWidget.setFont(font)
        self.listWidget.setStyleSheet("color:rgb(236, 240, 241);\n"
"background-color:rgba(70,113,156,156)")
        self.listWidget.setObjectName("listWidget")
        item = QtWidgets.QListWidgetItem()
        self.listWidget.addItem(item)
        item = QtWidgets.QListWidgetItem()
        self.listWidget.addItem(item)
        item = QtWidgets.QListWidgetItem()
        self.listWidget.addItem(item)
        item = QtWidgets.QListWidgetItem()
        self.listWidget.addItem(item)
        item = QtWidgets.QListWidgetItem()
        self.listWidget.addItem(item)
        self.verticalLayout_6.addWidget(self.listWidget, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        self.buttonBox = QtWidgets.QDialogButtonBox(Goomba)
        font = QFont()
        font.setFamily("Nirmala UI")
        self.buttonBox.setFont(font)
        self.buttonBox.setStyleSheet("color:rgb(236, 240, 241)")
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout_6.addWidget(self.buttonBox, 0, QtCore.Qt.AlignHCenter)
        self.line_4 = QtWidgets.QFrame(Goomba)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setLineWidth(10)
        self.line_4.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_4.setObjectName("line_4")
        self.verticalLayout_6.addWidget(self.line_4)
        self.horizontalLayout_2.addLayout(self.verticalLayout_6)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem5)
        self.pwr = QtWidgets.QPushButton(Goomba)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.pwr.sizePolicy().hasHeightForWidth())
        self.pwr.setSizePolicy(sizePolicy)
        self.pwr.setMinimumSize(QtCore.QSize(75, 75))
        self.pwr.setMaximumSize(QtCore.QSize(75, 75))
        self.pwr.setSizeIncrement(QtCore.QSize(2, 2))
        self.pwr.setBaseSize(QtCore.QSize(50, 50))
        font = QFont()
        font.setFamily("Nirmala UI")
        self.pwr.setFont(font)
        self.pwr.setStyleSheet(self.pwr_idle)
        self.pwr.setWhatsThis("")
        self.pwr.setIcon(QIcon(self.pwr_icon))
        self.pwr.setIconSize(QtCore.QSize(50, 50))
        self.pwr.setObjectName("pwr")
        self.verticalLayout_2.addWidget(self.pwr, 0, QtCore.Qt.AlignHCenter)
        self.label_14 = QtWidgets.QLabel(Goomba)
        self.label_14.setMaximumSize(QtCore.QSize(16777215, 30))
        font = QFont()
        font.setFamily("Nirmala UI")
        self.label_14.setFont(font)
        self.label_14.setObjectName("label_14")
        self.verticalLayout_2.addWidget(self.label_14, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        spacerItem6 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem6)
        self.horizontalLayout_3.addLayout(self.verticalLayout_2)

        self.retranslateUi(Goomba)
        QtCore.QMetaObject.connectSlotsByName(Goomba)

        self.reader = ReadThread(self.nRF)
        self.reader.start()
        self.reader.update_progress.connect(self.sensor2label)
        self.sender = SendThread(self.nRF)

        self.pwr.clicked.connect(self.toggle)
        self.buttonBox.clicked.connect(self.list_btn_clicked)

    def list_btn_clicked(self):
        item = self.listWidget.currentItem()
        state = str(item.text())
        self.label_13.setText("State: " + state)
        self.sender.state = state  # workaround bcuz can't pass arguements to start() 
        self.sender.start()
        if state == "Idle":
            self.pwr_state = False
            self.last = False
            self.label_14.setText("Paused")
            pwr_style = self.pause
            self.pwr.setIcon(QIcon(self.pwr_icon))
        elif (state == "Forward") or (state == "Turn: Left") or (state == "Turn: Right"):
            self.pwr_state = True
            self.last = True
            self.label_14.setText("Running")
            pwr_style = self.run
            self.pwr.setIcon(QIcon(self.pwr_icon))
        elif state == "Locate":
            self.label_14.setText("Ignition")
            pwr_style = self.pwr_idle
            self.pwr.setIcon(QIcon(self.pwr_icon))
        self.pwr.setStyleSheet(pwr_style)
        self.sender.finished.connect(self.send_finish)

    def toggle(self):  # on pwr button click this is called
        if self.last is False:
            self.label_14.setText("Running")
            pwr_style = self.run  # TODO integrate with sender class
            self.pwr_state = True  # gives go signal to bot
        elif self.last is True:
            self.label_14.setText("Paused")
            pwr_style = self.pause
            self.pwr_state = False
        self.last = not self.last
        self.pwr.setStyleSheet(pwr_style)
        self.pwr.setIcon(QIcon(self.pwr_icon))
        if self.pwr_state:
            self.sender.state = "Ignite"  # workaround bcuz can't pass arguements to start() 
            self.sender.start()
        else:
            self.sender.state = "Idle"
            self.sender.start()
        self.sender.finished.connect(self.send_finish)

    def sensor2label(self, vals):  # called when we receive a signal from other process
        start = 999
        end = 666
        LOCATE = 0
        FORWARD = 1
        TURN = 2
        TEST = 3
        TIMER = 1
        if int(vals[0]) == start:
            # start assigning text
            mang = self.magnet_angle(vals[1], vals[2], vals[3])
            self.label_12.setText(str(mang))

            self.encL_now, self.encR_now = [vals[4], vals[5]]
            encL_change = self.encL_now - self.encL_prev
            encR_change = self.encR_now - self.encR_prev
            self.encL_prev = self. encL_now
            self.encR_prev = self. encR_now
            if int(vals[14]) == TURN: 
                #bot is turning, angle according to encoders. may want to add additional label for this
                self.label_12.setText(str(self.turn_angle(encL_change, encR_change)))
            elif int(vals[14]) == FORWARD:
                #bot is forward
                self.label_17.setText(str(self.distance(encL_change, encR_change)))
            elif int(vals[14]) == LOCATE:
                #bot is forward
                self.label_17.setText(str(self.distance(encL_change, encR_change)))
            if int(vals[15]) == TIMER:
                self.label_14.setText("Running")
            if int(vals[15]) == TEST:
                self.label_14.setText("UART Test successful!")

            x, y, z = (vals[7], vals[8], vals[9])
            if x > (9.81 * 0.75):  # TODO eventually store and sort data based on the accleromter data
                vals.append("left too high")
            elif x < -(9.81 * 0.75):
                vals.append("right too high")
            elif y < (9.81 * 0.75):
                vals.append("front too high")
            elif y < -(9.81 * 0.75):
                vals.append("back too high")
            
            sonars = [vals[10], vals[11], vals[12]]
            self.label_6.setText(str(sonars[0]))
            self.label_7.setText(str(sonars[1]))
            self.label_8.setText(str(sonars[2]))

            self.label_10.setText(str(vals[13]) + " cm")
            if (vals[13] > 20):
                self.label_11.setText("True")
                self.label_11.setStyleSheet(self.red)
            elif vals[13] < 0:
                self.label_11.setText("Bad reading")
                self.label_11.setStyleSheet(self.light_brown)
                self.error_alert("Bad reading")
            else:
                self.label_11.setText("False")
                self.label_11.setStyleSheet(self.red)
            
        else:
            # go through list until i find start and end values
            try:
                e = vals.index(float(end))  # find where it ends
            except ValueError:
                e = "No signal"
            self.label_9.setText("Start data out of sync. \nCheck UART connections.\n" + str(e))
            time.sleep(0.1)
            #raise Exception("Start of data found at "+str(e)+" index. Make a function to auto-repair.")

    def vector_2_degrees(self, x, y):
        angle = degrees(atan2(y, x))
        if angle < 0:
            angle += 360
        return angle

    def magnet_angle(self, packets):
        magnet_x, magnet_y, _ = packets
        return self.vector_2_degrees(magnet_x, magnet_y)

    def distance(self, enc_change0, enc_change1):
        enc_change = (enc_change0 + enc_change1) / 2
        dist = enc_change * constant
        return dist
    
    def turn_angle(self, enc_change0, enc_change1, prior_ang=0):  # cross reference w/ magnetometer?
        enc_change = (enc_change0 + enc_change1) / 2
        ang = enc_change * constant
        ang_rad = ang * (3.141592 / 180) + prior_ang
        return ang_rad
    
    def send_finish(self):
        self.buttonBox.setStyleSheet(self.green)

    def error_alert(self, nut):
        # do stuff on signal that process is done
        QMessageBox.information(self, "Uh oh, error:\n" + nut)

    def retranslateUi(self, Goomba):
        _translate = QtCore.QCoreApplication.translate
        self.label_2.setText(_translate("Goomba", "Sonars"))
        self.label_3.setText(_translate("Goomba", "Left"))
        self.label_8.setText(_translate("Goomba", "123"))
        self.label_7.setText(_translate("Goomba", "123"))
        self.label_6.setText(_translate("Goomba", "234"))
        self.label_5.setText(_translate("Goomba", "Right"))
        self.label_4.setText(_translate("Goomba", "Front"))
        self.label_9.setText(_translate("Goomba", "Cliff Detection"))
        self.label_10.setText(_translate("Goomba", "123"))
        self.label_11.setText(_translate("Goomba", "False"))
        self.label.setText(_translate("Goomba", "Angle"))
        self.label_12.setText(_translate("Goomba", "123"))
        self.label_15.setText(_translate("Goomba", "Degrees"))
        self.label_13.setText(_translate("Goomba", "State"))
        self.label_14.setText(_translate("Goomba", "Ignition"))
        self.label_18.setText(_translate("Goomba", "Distance"))
        self.label_17.setText(_translate("Goomba", "123"))
        self.label_16.setText(_translate("Goomba", "Meters"))
        __sortingEnabled = self.listWidget.isSortingEnabled()
        self.listWidget.setSortingEnabled(False)
        item = self.listWidget.item(0)
        item.setText(_translate("Goomba", "Idle"))
        item = self.listWidget.item(1)
        item.setText(_translate("Goomba", "Locate"))
        item = self.listWidget.item(2)
        item.setText(_translate("Goomba", "Forward"))
        item = self.listWidget.item(3)
        item.setText(_translate("Goomba", "Turn: Left"))
        item = self.listWidget.item(4)
        item.setText(_translate("Goomba", "Turn: Right"))
        self.listWidget.setSortingEnabled(__sortingEnabled)
        self.pwr.setText(_translate("Goomba", ""))


# for reference of data sent
origins = [(0, 12.0, 2.3), (3, 254.1, 0), (5, 6, 7), (8, 9, 10), (1, 2, 3)]
class ReadThread(QThread):  # try to see if i can run multiple threads at once if one has a loop going
    update_progress = pyqtSignal(list)  # need to define what kind of signal we want to send
    worker_complete = pyqtSignal(list)

    def __init__(self, nRF):  # should allow us to pass arguements into class
        super().__init__()
        self.nRF = nRF

    def run(self):
        while True:
            data = []
            for i in range(18):  # range 17 for starting and stopping numbs?
                received, er = self.read_uart()
                try:
                    data.append(received[0])
                except Exception:
                    data.append(0)
            if len(data) > 2:  # if i get more than the start and end bit, transmit data
                self.update_progress.emit(data)

    def read_uart(self, numbytes=8):
        data = self.nRF.read(numbytes)
        data_string = None
        er = None
        if data is not None:
            try:
                data_string = struct.unpack("d", data)
            except Exception as e:
                print("No data found. \nError message:", e)
                er = e
        return (data_string, er)
        # print(data_string) gives (123.0,) as out, how to interpret?


class SendThread(QThread):  # try to see if i can run multiple threads at once if one has a loop going
    update_progress = pyqtSignal(int)  # need to define what kind of signal we want to send
    #sender_complete = pyqtSignal(int)
    #nRF = serial.Serial("/dev/ttyS0", 15000, timeout=0.3)

    def __init__(self, nRF, state="bad"):  # should allow us to pass arguements into class
        super().__init__()
        self.nRF = nRF
        self.state = state

    def run(self):  # TODO take button presh to send state change to nRF
        # take a value to indicate which state to send
        # turn GPIO digital out high
        self.update_progress.emit(1)
        if self.state == "Idle":
            val = 0
        elif (self.state == "Forward"): 
            val = 1
        elif (self.state == "Turn: Left"):
            val = 2
        elif (self.state == "Turn: Right"):
            val = 3
        elif self.state == "Locate":
            val = 4
        elif self.state == "bad":
            val = 5
            print("Did not pass in new state")
        else:  # error case
            val = 5
        self.send_bytes(val)
        self.state = "bad"
        
    def run_reference(self):  # why does this method go when worker.start() is called?
        # running our process
        # note can't call method in separate class
        # this thread has a finished signal we can catch in main class
        self.update_progress.emit(123)
        for x in range(12):
            self.update_progress.emit(x)
            time.sleep(0.1)
        self.worker_complete.emit([(1, 0, 1.1), (1, 2, 3)])  # emitted at same time as finish signal
        # this is where we get each piece of data and send in an ordered manner to gui

    def send_bytes(self, value):  # sending single int to indicate state
        self.nRF.write(struct.pack("f", 999))
        self.nRF.write(struct.pack("f", value))  # use struct.unpack to get float back
        self.nRF.write(struct.pack("f", 666))


if __name__ == "__main__":
    import sys
    nRF = serial.Serial("/dev/ttyS0", 15000, timeout=0.3)
    app = QtWidgets.QApplication(sys.argv)
    Goomba = QtWidgets.QWidget()
    ui = Ui_Goomba(nRF)
    ui.setupUi(Goomba)
    Goomba.show()
    sys.exit(app.exec())

# rpi_main.py - ME 106 GOOMBA Project Code
# Raspeberry Pi
#
# Written by Dylan Robinson and Ryan Sands (sandsryanj@gmail.com)
#   v0.80 29-Apr-2021 Drafting of functions to interpret data and comm to nRF,
#                     next step is to implement GUI

import gpiozero
from gpiozero import Button
import RPi.GPIO as GPIO
import time

import serial
import struct

from math import cos, sin, atan2, degrees

import sys
import goomba_panel  # PyQt file to run GUI & multithreading

from PyQt5 import QtCore, QtWidgets 
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QMessageBox

nRF = serial.Serial("/dev/ttyS0", 20000, timeout=0.3)


def error(err_string):
    raise Exception(err_string)


def read_uart(rpi, numbytes=8):
    data = rpi.read(numbytes)
    data_string = None
    er = None
    if data is not None:
        try:
            data_string = struct.unpack("d", data)
        except Exception as e:
            print("No data found. \nError message:", e)
            er = e
    return (data_string, er)

def send_bytes(rpi, origin_data):
    rpi.write(struct.pack("f", 999))
    for count0, d_list in enumerate(origin_data):
        for value in d_list:
            rpi.write(struct.pack("f", value))  # use struct.unpack to get float back
            print(struct.pack("f", value))
    rpi.write(struct.pack("f", 666))

# launch GUI
app = QtWidgets.QApplication(sys.argv)
goomba = QtWidgets.QWidget()
ui = goomba_panel.Ui_Goomba()
ui.setupUi(goomba)
goomba.show()  # TODO properly implement goomba_panel working version

# Dylan GPIO stuff
CLK = 18
MOSI = 23

# TODO if some variable is true, have functions turn a GPIO to digital high and call send_bytes function
def setupSpiPins(mosiPin):
    ''' Set all pins as an output except MOSI (Master Output, Slave Input)'''
    pass     


def readAdc(channel, mosiPin):
    if (channel < 0) or (channel > 7):
        print("Invalid ADC Channel number, must be between [0,7]")
        return -1


button = Button(2)

while True:
    if button.is_pressed:
        print("Pressed")
    else:
        print("Released")
    time.sleep(1)

nRF.close()

    sys.exit(app.exec())  # once i close window, this exits the program
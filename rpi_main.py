# rpi_main.py - ME 106 GOOMBA Project Code
# Raspeberry Pi
#
# Written by Ryan Sands (sandsryanj@gmail.com)
#   v0.80 29-Apr-2021 Drafting of functions to interpret data and comm to nRF,
#                     next step is to implement GUI

import gpiozero
import time

import serial
import struct

from math import cos, sin, atan2, degrees

import sys
from goomba_panel import Ui_Goomba  # PyQt file to run GUI

from PyQt5 import QtCore, QtGui, QtWidgets 
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import QThread


def error(err_string):
    raise Exception(err_string)


# functions for RPi to interpret data?
def vector_2_degrees(self, x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle


def magnet_angle(packets):
    magnet_x, magnet_y, _ = packets
    return vector_2_degrees(magnet_x, magnet_y)


def distance(enc_change0, enc_change1):
    enc_change = (enc_change0 + enc_change1)/2
    dist = enc_change * constant
    return dist


def angle(enc_change0, enc_change1, prior_ang=0):  # cross reference w/ magnetometer?
    enc_change = (enc_change0 + enc_change1)/2
    ang = enc_change * constant
    ang_rad = ang * (3.141592/180) + prior_ang
    return ang_rad


def new_vect(ang, dist):  # takes radians and cm for movement of goomba
    vect = []
    vect[0] = float(dist) * cos(ang)
    vect[1] = float(dist) * sin(ang)
    return vect


nRF = serial.Serial("/dev/ttyS0", 15000, timeout=0.3)

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


# launch GUI
app = QtWidgets.QApplication(sys.argv)
goomba = QtWidgets.QWidget()
ui = Ui_Goomba()
ui.setupUi(goomba)
goomba.show()  # TODO include distance traveled in GUI


last_time = time.monotonic()
blink_time = .1
while True:
    if time.monotonic() - last_time > blink_time:
        last_time = time.monotonic()
        received_data, er = read_uart(nRF)
        if received_data is None:
            nRF.write(bytes(str(er), "utf-8"))  # transmit data serially 
        else:
            pass
    if ui.pwr_state:  # this is does not work while gui running. has to be in GUI class
            print("on")
    else:
        '''
        if goomba.pwr_state:  
            print("on")'''
        pass

    sys.exit(app.exec())  # once i close window, this exits the program
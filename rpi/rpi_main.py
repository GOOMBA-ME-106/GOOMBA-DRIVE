# rpi_main.py - ME 106 GOOMBA Project Code
# Raspeberry Pi
#
# Written by Dylan Robinson and Ryan Sands (sandsryanj@gmail.com)
#   v0.80 29-Apr-2021 Drafting of functions to interpret data and comm to nRF,
#                     next step is to implement GUI

from gpiozero import LED
import serial

import sys
import goomba_panel  # PyQt file to run GUI & multithreading
from PyQt5 import QtCore, QtWidgets 

nRF = serial.Serial("/dev/ttyS0", 20000, timeout=0.3)
CS_OUT = LED(23)

'''
 consider reworking this file as everything is currently handled in goomba_panel?
 or consolidate everything to goomba_panel
 '''

# launch GUI
app = QtWidgets.QApplication(sys.argv)
goomba = QtWidgets.QWidget()
ui = goomba_panel.Ui_Goomba(nRF, CS_OUT)
ui.setupUi(goomba)
goomba.show()
sys.exit(app.exec_())  # once i close window, this exits the program

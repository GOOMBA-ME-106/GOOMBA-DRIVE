# GOOMBA-DRIVE
Main program to drive the goomba (two wheeled vehicle meant to automatically map the floor plan of a room).

## Sensors
Consists of 3 sonar sensors to the front and sides along with an IR proximity sensor to sense cliffs.
A magentomneter is also used to verify angle of the goomba.

## Devices
- nRF52840 Bluefruit Sense utilizes the goomba_driving,py file to drive retain data from sensors and control the motors.
- raspberry pi periodically receives the data by UART and handles the GUI.
- Mobile device with Bluefruit Connect app to remotely interact with the nRF.

## Outputs
There are two DC brushed motors to control the two wheels. They are driven by two (half?) H-bridges.

## Libraries
Needed libraries for CircuitPython can be found [here](https://learn.adafruit.com/adafruit-feather-sense/feather-sense-circuitpython-libraries "Feather Sense CircuitPython Libraries")

The needed libraries from there are:
- adafruit_lis3mdl
- adafruit_hcsr04
- adafruit_lsm6ds.lsm6ds33
- adafruit_motor 
- adafruit_ble
- adafruit_bluefruit_connect

PyQt5:
- [pyqt-tools](https://pypi.org/project/pyqt5-tools/ "install pyqt tools for the designer")
- [PyQt5](https://pypi.org/project/PyQt5/ "necessary for Raspberry Pi to desiplay GUI")
- 
**note**: on Raspberry Pi OS 5.10, the background-color will not work propoerly and will need to be [fixed](https://www.google.com/ "temp link")

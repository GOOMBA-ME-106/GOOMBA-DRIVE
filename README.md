# GOOMBA-DRIVE
Main program to drive the goomba (two wheeled vehicle meant to automatically map the floor plan of a room)

## Sensors
Consists of 3 sonar sensors to the front and sides along with an IR sensor to sense cliffs.
A magentomneter is also used to...?

## Outputs
There are two DC brushed motors to control the two wheels. They are driven by two (half?) H-bridges.

## Libraries
Needed libraries for CircuitPython can be found [here](https://learn.adafruit.com/adafruit-feather-sense/feather-sense-circuitpython-libraries "Feather Sense CircuitPython Libraries")
The needed libraries from there are:
- adafruit_lis3mdl
- adafruit_irremote
- adafruit_hcsr04
- adafruit_motor
- [adafruit_bus_device](https://github.com/adafruit/Adafruit_CircuitPython_BusDevice "adafruit_bus_device")

and probably  
  
- adafruit_ble
- adafruit_bluefruit_connect

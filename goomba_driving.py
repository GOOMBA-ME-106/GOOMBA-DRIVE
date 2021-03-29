# goomba_driving.py - ME 106 Project Code
#   For use with the EduShields TriplerBaseboard
#
# Written by Ryan Sands (sandsryanj@gmail.com)
#   v0.8 25-Mar-2021 Drafting of functions for sensors and classes for state machine

import board
from board import SCL, SDA, SCK, MOSI, MISO
import pwmio
import rotaryio
import digitalio  # for SPI CS
import pulseio  # pulseio for IR sensor
import time
import busio
import adafruit_lis3mdl  # magnetometer
import adafruit_irremote
import adafruit_hcsr04  # sonar sensor
from adafruit_bus_device.spi_device import SPIDevice
from adafruit_motor import motor

from math import cos
from math import sin

from adafruit_ble import BLERadio  # for testing motors remotely
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket

'''
 we have 7 digital pins, 6 analog pins explicitly - D13, D12, D11, D10, D9, D6, D5, D2
 Analog available A0, A1, A2, A3, A4, A5 just take out IW
 gives tentative total of 13 pins at our disposal, no I2C pins
 Sck, MO, MI pins can be used for general purpose IO (GPIO), but we use SPI
 so IR stuff goes to other board
'''
# if we get I2C to work, move a sensor and IR sensor to free 2 gpio for CS for SPI and IRLED out
#PIN_IR = board.TX  # placeholder
#PIN_IRLED = board.RX # placeholder
# also need to move a sonar or encoder over if no I2C

# sensor input pins
PIN_SON_L0 = board.D11
PIN_SON_L1 = board.D10
PIN_SON_F0 = board.D9
PIN_SON_F1 = board.D6
PIN_SON_R0 = board.D5
PIN_SON_R1 = board.D2

PIN_ENC_L0 = board.A0
PIN_ENC_L1 = board.A1
PIN_ENC_R0 = board.D12
PIN_ENC_R1 = board.D13

encL = rotaryio.IncrementalEncoder(PIN_ENC_L0, PIN_ENC_L1)
encR = rotaryio.IncrementalEncoder(PIN_ENC_R0, PIN_ENC_R1)


sonarL = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_L0, echo_pin=PIN_SON_L1)  # sonar dist in cm
#sonarF = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_F0, echo_pin=PIN_SON_F1)
#sonarR = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_R0, echo_pin=PIN_SON_R1)

i2c = busio.I2C(SCL, SDA)
lis3 = adafruit_lis3mdl.LIS3MDL(i2c)
#sonarL = adafruit_hcsr04.HCSR04(i2c)  # gives error pluggin straight into SCL SDA

pulsein = pulseio.PulseIn(PIN_IR, maxlen=150, idle_state=True)
decoder = adafruit_irremote.GenericDecode()

# output pins
PIN_MOTL0 = board.A4
PIN_MOTL1 = board.A5
PIN_MOTR0 = board.A2  # these pins replacing IR stuff
PIN_MOTR1 = board.A3

motL0 = pwmio.PWMOut(PIN_MOTL0)
motL1 = pwmio.PWMOut(PIN_MOTL1)
motL = motor.DCMotor(motL0, motL1)
motL.FAST_DECAY = 1
motR0 = pwmio.PWMOut(PIN_MOTR0)
motR1 = pwmio.PWMOut(PIN_MOTR1)
motR = motor.DCMotor(motR0, motR1)
motR.FAST_DECAY = 1

# SPI stuff
'''
 any free digital I/O pin to rpi CS/chip select
 most chips expect CS line to be held high when they aren’t in use
 then pulled low when the processor is talking to them,
 but check your device’s datasheet
'''
PIN_CS = board.RX  # placeholder pin
cs = digitalio.DigitalInOut(PIN_CS)
cs.direction = digitalio.Direction.OUTPUT
spi = busio.SPI(SCK, MISO, MOSI)
raspi = SPIDevice(spi, cs, baudrate=2*(10**6), polarity=0, phase=0)  # need to look up for our rpi
'''
 the SPI device class only supports devices with a chip select
 and whose chip select is asserted with a low logic signal.
 otherwise we have to lock/unlock spi and enable/disable cs manually
'''

DUTY_MAX = 2**16-1

ble = BLERadio()
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)

B1 = ButtonPacket.BUTTON_1
B2 = ButtonPacket.BUTTON_2
B3 = ButtonPacket.BUTTON_3
B4 = ButtonPacket.BUTTON_4
DOWN = ButtonPacket.DOWN
UP = ButtonPacket.UP
LEFT = ButtonPacket.LEFT
RIGHT = ButtonPacket.RIGHT


def error(err_string):
    raise Exception(err_string)


def motor_level(level, mot):  # input is range of percents, -100 to 100
    mot.throttle = float(level/100)


def turning(direction, mag=75):
    direc = str(direction).upper()
    if direc == "LEFT":
        motor_level(mag, motL)
        motor_level(-mag, motR)
    elif direc == "RIGHT":
        motor_level(mag, motR)
        motor_level(-mag, motL)
    else:
        print(direction, "is not a valid direction to turn.")


def distance(enc_change0, enc_change1):  # is there some way to use magnetometer w/ this?
    enc_change = (enc_change0 + enc_change1)/2
    dist = enc_change * constant
    return dist


def angle(enc_change0, enc_change1, prior_ang=0):  # is there some way to use magnetometer w/ this to verify?
    enc_change = (enc_change0 + enc_change1)/2
    ang = enc_change * constant
    ang_rad = ang * (3.141592/180) + prior_ang
    return ang_rad


def new_vect(ang, dist):  # takes radians and cm
    vect = []
    vect[0] = float(dist) * cos(ang)
    vect[1] = float(dist) * sin(ang)
    return vect


def send_bytes(data_list):
    for count0, value0 in enumerate(values):
        for  value1 in value0:
            spi.write(bytes(struct.pack("d", float(value1))))
            
#struct.unpack("d", a) to get the values back


def motor_test(mot1, mot2, drive_time, mag=60):
    drive = drive_time/2
    motor_level(mag, mot1)
    motor_level(mag, mot2)
    time.sleep(drive)
    motor_level(-mag, mot1)
    motor_level(-mag, mot2)
    time.sleep(drive)
    motor_level(0, mot1)
    motor_level(0, mot2)
    time.sleep(0.1)

# how to use SPI
with raspi:
    result = bytearray(4)  # 4 byte buffer is created to hold the result of the SPI read
    spi.readinto(result)  #  called to read 4 bytes of data from the rpi
print(result)  # need to check rpi’s datasheet to see how to interpret the data
# can also use the busio.SPI.write() function to send data over the MOSI line in bytes
# for example
with raspi:
    spi.write(bytes([0x01, 0xFF]))
'''
 CS line is asserted for the entire with statement block,
 so if you need to make two different transactions
 be sure to put them in their own with statement blocks
'''


# Main loop
step = 5
speed1 = 0
speed2 = 0
while True:  # the testing loop
    ble.start_advertising(advertisement)
    while not ble.connected:
        #dists = [sonarL.distance, sonarF.distance, sonarR.distance]
        dists = [sonarL.distance]
        #print("Sonar distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
        print(dists)
        print('Magnetometer: {0:10.2f}X {1:10.2f}Y {2:10.2f}Z uT'.format(*lis3.magnetic))
        print('Encoders: {0:10.2f}L {1:10.2f}R pulses'.format(encL.position, encR.position))

        time.sleep(0.5)
        mot_test0 = input("Test motors? /n Y or N ")
        mot_test1 = mot_test0.upper()
        if mot_test1 == "Y":
            duration = float(input("How long? "))
            motor_test(motL, motR, duration)
        elif mot_test1 == "END":
            break

    # Now we're connected

    while ble.connected:
        if uart.in_waiting:
            packet = Packet.from_stream(uart)
            if isinstance(packet, ButtonPacket):
                if packet.pressed:
                    if packet.button == B1:
                        # The 1 button was pressed.
                        print("1 button pressed! It was super effective!")
                        speed1 = 90
                        speed2 = 90
                    elif packet.button == UP:
                        # The UP button was pressed.
                        print("UP button pressed! The left motor's speed sharply rose!")
                        if speed1 != 100:
                            speed1 += step
                    elif packet.button == DOWN:
                        # The DOWN button was pressed.
                        print("DOWN button pressed! The  left motor's speed sharply fell!")
                        if speed1 != -100:
                            speed1 += -step
                    elif packet.button == LEFT:
                        # The LEFT button was pressed.
                        print("LEFT button pressed! The right motor's speed sharply fell!")
                        if speed2 != -100:
                            speed2 += -step
                    elif packet.button == RIGHT:
                        # The RIGHT button was pressed.
                        print("RIGHT button pressed! The right motor's speed sharply rose!")
                        if speed2 != 100:
                            speed2 += step
                    elif packet.button == B2:
                        # The 2 button was pressed.
                        print("2 button pressed! It was super effective but in reverse.")
                        speed1 = -90
                        speed2 = -90
                    elif packet.button == B3:
                        # The 3 button was pressed.
                        print("3 button pressed! It was not very effective.")
                    elif packet.button == B4:
                        # The 4 button was pressed.
                        print("4 button pressed! It was a one hit KO!")
                        speed1 = 0
                        speed2 = 0

        motor_level(speed1, motL)
        motor_level(speed2, motR)

        #dists = [sonarL.distance, sonarF.distance, sonarR.distance]

        #print("Sonar distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
        print('Magnetometer: {0:10.2f}X {1:10.2f}Y {2:10.2f}Z uT'.format(*lis3.magnetic))
        print('Encoders: {0:10.2f} L {1:10.2f} R pulses'.format(encL.position, encR.position))

        time.sleep(0.1)
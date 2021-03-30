# goomba_driving.py - ME 106 Project Code
#   For use with the EduShields TriplerBaseboard
#
# Written by Ryan Sands (sandsryanj@gmail.com)
#   v0.8 25-Mar-2021 Drafting of functions for sensors and classes for state machine

import board
from board import SCL, SDA, SCK, MOSI, MISO, RX, TX
import pwmio
import rotaryio
import digitalio  # for SPI CS
import pulseio  # pulseio for IR sensor
import time
import busio
import adafruit_lis3mdl  # magnetometer
import adafruit_irremote
import adafruit_hcsr04  # sonar sensor
from adafruit_motor import motor

from math import cos
from math import sin

from adafruit_ble import BLERadio  # for testing motors remotely
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket

import digitalio  # for pull up
'''
 we have 7 digital pins, 6 analog pins explicitly - D13, D12, D11, D10, D9, D6, D5, D2
 Analog available A0, A1, A2, A3, A4, A5 just take out IW
 gives tentative total of 13 pins at our disposal, no I2C pins
 Sck, MO, MI pins can be used for general purpose IO (GPIO), but we use SPI
 so IR stuff goes to other board
'''
# if we get I2C to work, move a sensor and IR sensor to free 2 gpio for CS for SPI and IRLED out
PIN_IR = board.MOSI  # placeholder
PIN_IRLED = board.MISO # placeholder
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

i2c = board.I2C()
lis3 = adafruit_lis3mdl.LIS3MDL(i2c)

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


# UART stuff for RPI
rpi_write = busio.UART(TX, RX, baudrate=9600, timeout=0.5)
rpi_read = busio.UART(SCL, SDA, baudrate=9600)

def send_bytes(origin_data):  # TODO send bytes representing data through SPI 
    for count0, d_list in enumerate(origin_data):
        for value in d_list:
            uart_rpi.write(bytes(struct.pack("d", float(value))))  # use struct.unpack to get float back
#values = [[1234, 1237], ("21.967", "-62.146", "-4.516")]
#send_bytes(values)

def read_uart(numbytes):
    data = uart_rpi.read(numbytes)
    if data is not None:
        try:
            data_string = struct.unpack("d", data)
            print(data_string, end="")
        except:
            print("No data found.")
# may want to use SCK pin as a chipselect pin for communication


# Main loop
step = 5
speed1 = 0
speed2 = 0
while True:  # the testing loop
    #dists = [sonarL.distance, sonarF.distance, sonarR.distance]
    dists = [sonarL.distance]
    encs = [encL.position, encR.position]
    
    ble.start_advertising(advertisement)
    while not ble.connected:
        
        #print("Sonar distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
        print(dists)
        print('Magnetometer: {0:10.2f}X {1:10.2f}Y {2:10.2f}Z uT'.format(*lis3.magnetic))
        print('Encoders: {0:10.2f}L {1:10.2f}R pulses'.format(*encs))

        time.sleep(0.5)
        mot_test0 = input("Test motors? /n Y or N ")
        mot_test1 = mot_test0.upper()
        uart_test0 = input("Test UART? /n Y or N ")
        uart_test1 = uart_test0.upper()
        if mot_test1 == "Y":
            duration = float(input("How long? "))
            motor_test(motL, motR, duration)
        if uart_test1 == "Y":
            origins = [dists, encs, lis3.magnetic]
            send_bytes(origins)
            read_uart(8)
        elif (mot_test1 == "END") or (uart_test1 == "END"):
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
                        origins = [dists, encs, lis3.magnetic]
                        send_bytes(origins)
                        read_uart(8)
                    elif packet.button == B4:
                        # The 4 button was pressed.
                        print("4 button pressed! It was a one hit KO!")
                        speed1 = 0
                        speed2 = 0

        motor_level(speed1, motL)
        motor_level(speed2, motR)

        #print("Sonar distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
        print('Magnetometer: {0:10.2f} X {1:10.2f} Y {2:10.2f} Z uT'.format(*lis3.magnetic))
        print('Encoders: {0:10.2f} L {1:10.2f} R pulses'.format(encs))

        time.sleep(0.1)
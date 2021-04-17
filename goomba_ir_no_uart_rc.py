# goomba_driving.py - ME 106 Project Code
#   For use with the EduShields TriplerBaseboard
#
# Written by Ryan Sands (sandsryanj@gmail.com)
#   v0.80 25-Mar-2021 Drafting of functions for sensors and classes for state machine
#   v0.90 26-Mar-2021 Initial version of state machine to handle 5 events. Needs to finish sensor handling
#   v0.91 11-Apr-2021 Added sharp-gp2y0a21yk0f cliff sensor functionality and bluetooth

import board
from board import SCL, SDA, SCK, MOSI, MISO, RX, TX
import pwmio
from analogio import AnalogIn
import rotaryio
import time
import struct
from busio import UART
import adafruit_lis3mdl  # magnetometer
import adafruit_hcsr04  # sonar sensor
import adafruit_lsm6ds.lsm6ds33  # acceleromter
from adafruit_motor import motor

from math import atan2, degrees

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

PIN_IR = board.A2  # MUST BE analog pin
IR = AnalogIn(PIN_IR)

encL = rotaryio.IncrementalEncoder(PIN_ENC_L0, PIN_ENC_L1)
encR = rotaryio.IncrementalEncoder(PIN_ENC_R0, PIN_ENC_R1)


sonarL = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_L0, echo_pin=PIN_SON_L1)  # sonar dist in cm
sonarF = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_F0, echo_pin=PIN_SON_F1)
sonarR = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_R0, echo_pin=PIN_SON_R1)

i2c = board.I2C()
lis3 = adafruit_lis3mdl.LIS3MDL(i2c)
lsm6 = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c)


# output pins
PIN_MOTL0 = board.A4
PIN_MOTL1 = board.A5
PIN_MOTR0 = SCK
PIN_MOTR1 = MOSI

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


def vector_2_degrees(x, y):  # we can prob move these over to the RPi
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


# UART stuff for RPI
rpi_write = UART(TX, RX, baudrate=9600, timeout=1)
#rpi_read = UART(SCL, SDA, baudrate=9600)  # need pull up resistor for this?


def send_bytes(origin_data, rpi):
    for count0, d_list in enumerate(origin_data):
        for value in d_list:
            rpi.write(bytes(struct.pack("d", float(value))))  # use struct.unpack to get float back


def read_uart(numbytes, rpi):
    data = rpi.read(numbytes)
    if data is not None:
        try:
            data_string = struct.unpack("d", data)
            print(data_string)
        except Exception:
            print("No data found.")


# States of state machine
class state_machine():
    go = None
    start = "IDLE"

    def __init__(self,  motL, motR, encL, encR, magneto, accel):
        self.state = self.start
        self.mot1 = motL
        self.mot2 = motR
        self.encL = encL
        self.encR = encR
        self.magnet = magneto
        self.accel = accel

    def forward(self, speed):
        motor_level(speed, self.mot1)
        motor_level(speed, self.mot2)

    def idle(self, ignite):
        motor_level(0, self.mot1)
        motor_level(0, self.mot2)
        if ignite is True:
            self.state = "LOCATE"

    def locate(self):  # include indicator for cliff?
        global dists
        thing = [[(0, 0, 0), (0, 0), (0, 0, 0), (0, 0, 0)]]
        thing[0][0] = self.magnet.magnetic
        thing[0][1] = (self.encL.position, self.encR.position)
        thing[0][2] = self.accel.acceleration
        thing[0][3] = dists
        if self.state == "LOCATE":
            self.state = "FORWARD"
        else:
            self.state = "TURN"
        return thing

    def turn(self, direction, mag=20):
        direc = str(direction).upper()
        if direc == "LEFT":
            motor_level(mag, motL)
            motor_level(-mag, motR)
        elif direc == "RIGHT":
            motor_level(mag, motR)
            motor_level(-mag, motL)
        else:
            print(direction, "is not a valid direction to turn.")

    def cliff_dist(self, cliff):  # in cm, good for ~9 to ~30
        volt = (cliff.value * 3.3) / 65536
        try:
            return (volt ** -1.173) * 29.988
        except ZeroDivisionError:
            print("The cliff sensor is giving bad readings.")
            time.sleep(0.05)


def cliff_function(dist):  #output true when cliff
    try:
        if dist >= 20:
            return True
        else:
            return False
    except TypeError:
        return True


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


testing = True
last = time.monotonic()
print_time = .2
test_q = "Y"

# Main loop
vector_array = {}
origins = [[(0, 0, 0), (0, 0), (0, 0, 0)], \
    [(0, 0, 0), (0, 0), (0, 0, 0)]]
i = 0

s_threshhold = 30  # in cm
start_button = None
step = 5
speed1 = 0
speed2 = 0

goomba = state_machine(motL, motR, encL, encR, lis3, lsm6)
while True:  # actual main loop
    ble.start_advertising(advertisement)
    while not ble.connected:  # for testing while connected
        try:
            dists = [sonarL.distance, sonarF.distance, sonarR.distance]
            print("Sonar distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
        except Exception:
            print("At least one sonar is not detected.")
        encs = [encL.position, encR.position]
        print('Magnetometer: {0:10.2f}X {1:10.2f}Y {2:10.2f}Z uT'.format(*lis3.magnetic))
        print('Encoders: {0:10.2f}L {1:10.2f}R pulses'.format(*encs))
        print("Acceleration: {:.2f} {:.2f} {:.2f} m/s^2".format(*lsm6.acceleration))
        cliff = goomba.cliff_dist(IR)
        print("Cliff distance:", cliff, "cm")
        print("Cliff?", cliff_function(cliff), "cm")

        time.sleep(print_time)
        if test_q != "skip":
            mot_test0 = input("Test motors? /n Y or N ")
            mot_test1 = mot_test0.upper()
            test_q = mot_test0
            if (mot_test1 == "END"):
                break
            elif mot_test1 == "SKIP":
                pass
            else:
                uart_test0 = input("Test UART? /n Y or N ")
                uart_test1 = uart_test0.upper()
                if test_q != "skip":
                    test_q = uart_test0
                if (uart_test1 == "END"):
                    break
                elif mot_test1 == "Y":
                    duration = float(input("How long? "))
                    motor_test(motL, motR, duration)
                elif uart_test1 == "Y":
                    origins = [dists, encs, lis3.magnetic, lsm6.acceleration]
                    send_bytes(origins, rpi_write)
                    read_uart(8, rpi_write)

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
                            if (speed1 >= -30) and (speed1 <= 0):
                                speed1 += 30
                            else:
                                speed1 += step
                    elif packet.button == DOWN:
                        # The DOWN button was pressed.
                        print("DOWN button pressed! The  left motor's speed sharply fell!")
                        if speed1 != -100:
                            if (speed1 >= 0) and (speed1 <= 30):
                                speed1 += -30
                            else:
                                speed1 += -step
                    elif packet.button == LEFT:
                        # The LEFT button was pressed.
                        print("LEFT button pressed! The right motor's speed sharply fell!")
                        if speed2 != -100:
                            if (speed2 >= 0) and (speed2 <= 30):
                                speed2 += -30
                            else:
                                speed2 += -step
                    elif packet.button == RIGHT:
                        # The RIGHT button was pressed.
                        print("RIGHT button pressed! The right motor's speed sharply rose!")
                        if speed2 != 100:
                            if (speed2 >= -30) and (speed2 <= 0):
                                speed2 += -30
                            else:
                                speed2 += step
                    elif packet.button == B2:
                        # The 2 button was pressed.
                        print("2 button pressed! It was super effective but in reverse?")
                        speed1 = -90
                        speed2 = -90
                    elif packet.button == B3:
                        # The 3 button was pressed.
                        print("3 button pressed! It used telepathy.")
                        #origins = [dists, encs, lis3.magnetic, lsm6.acceleration]
                        origins = [encs, lis3.magnetic, lsm6.acceleration]
                        send_bytes(origins, rpi_write)
                        read_uart(8, rpi_write)
                    elif packet.button == B4:
                        # The 4 button was pressed.
                        print("4 button pressed! It was a one hit KO!")
                        speed1 = 0
                        speed2 = 0
        motor_level(speed1, motL)
        motor_level(speed2, motR)

# this state machine can take 5 events:
# true or false input for cliffs
# less than threshold value on any of the left front or right sonar
# and taking some sort of reset button
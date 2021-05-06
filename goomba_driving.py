# goomba_driving.py - ME 106 GOOMBA Project Code
# nRF52840 Feathersense microcontroller. no TriplerBaseBoard.
#
# Written by Justin Ramos, Dylan Robinson, Ryan Sands (sandsryanj@gmail.com)
#   v0.80 25-Mar-2021 Drafting of functions for sensors and classes for state machine
#   v0.90 26-Mar-2021 Initial version of state machine to handle 5 events. ~Needs to finish sensor handling~
#   v0.91 11-Apr-2021 Added sharp-gp2y0a21yk0f cliff sensor functionality and bluetooth
#   v1.00 29-Apr-2021 QoL updates for testing, fixed data reording, final pin assignments, working state machine
#   v1.01 29-Apr-2021 Prep for RPi

import board
from board import SCL, SDA, SCK, MOSI, MISO, RX, TX
import pwmio
import digitalio
from analogio import AnalogIn
import rotaryio
import time
import struct
from busio import UART
import adafruit_lis3mdl  # magnetometer
import adafruit_hcsr04  # sonar sensor
import adafruit_lsm6ds.lsm6ds33  # acceleromter
from adafruit_motor import motor

from adafruit_ble import BLERadio  # for testing motors remotely
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket

'''
 we have 7 digital pins, 6 analog pins explicitly - D13, D12, D11, D10, D9, D6, D5, D2
 Analog available A0, A1, A2, A3, A4, A5 just take out IW
 Sck, MO, MI pins can be used for general purpose IO (GPIO)
 SCL SDA should be free for GPIO, but gives pull up error
 gives total of 16 pins at our disposal
'''

# sensor input pins
# sensor input pins
PIN_SON_L0 = board.D11
PIN_SON_L1 = board.D10
PIN_SON_F0 = board.D9
PIN_SON_F1 = board.D6
PIN_SON_R0 = board.D13
PIN_SON_R1 = board.D12

PIN_ENC_L0 = board.A0
PIN_ENC_L1 = board.A1
PIN_ENC_R0 = board.D2
PIN_ENC_R1 = board.D5

PIN_IR = board.A2  # MUST BE analog pin
IR = AnalogIn(PIN_IR)
PIN_RPI_IN = MISO
RPI_CS = digitalio.DigitalInOut(PIN_RPI_IN)
RPI_CS.direction = digitalio.Direction.INPUT


encL = rotaryio.IncrementalEncoder(PIN_ENC_L0, PIN_ENC_L1)
encR = rotaryio.IncrementalEncoder(PIN_ENC_R0, PIN_ENC_R1)

sonarL = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_L0, echo_pin=PIN_SON_L1)  # sonar dist in cm
sonarF = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_F0, echo_pin=PIN_SON_F1)
sonarR = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_R0, echo_pin=PIN_SON_R1)
sonar = [sonarL, sonarF, sonarR]

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

DUTY_MAX = 2 ** 16 - 1

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

EVENT_NONE = 0
EVENT_TIMER = 1
DATA_SEND_INTERVAL = 0.3

timer_time = None


def error(err_string):
    raise Exception(err_string)


def motor_level(level, mot):  # input is range of percents, -100 to 100
    mot.throttle = float(level / 100)


def motor_test(mot1, mot2, drive_time, mag=60):
    drive = drive_time / 2
    motor_level(mag, mot1)
    motor_level(mag, mot2)
    time.sleep(drive)
    motor_level(-mag, mot1)
    motor_level(-mag, mot2)
    time.sleep(drive)
    motor_level(0, mot1)
    motor_level(0, mot2)
    time.sleep(0.1)


# States of state machine
class state_machine():
    go = None
    start = "IDLE"
    origins = []

    def __init__(self, motL, motR, encL, encR, magneto, accel, cliff, sonar, rpi_serial):
        self.state = self.start
        self.mot1 = motL
        self.mot2 = motR
        self.encL = encL
        self.encR = encR
        self.magnet = magneto
        self.accel = accel
        self.sL = sonar[0]
        self.sF = sonar[1]
        self.sR = sonar[2]
        self.cliff = cliff
        self.serial = rpi_serial

    def forward(self, speed=70):
        motor_level(speed, self.mot1)
        motor_level(speed, self.mot2)

    def idle(self, ignite):
        motor_level(0, self.mot1)
        motor_level(0, self.mot2)
        if ignite is True:
            self.state = "LOCATE"

    def locate(self):
        thing = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        thing[0] = self.magnet.magnetic
        thing[1] = (self.encL.position, self.encR.position)
        thing[2] = self.accel.acceleration
        thing[3] = self.grab_sonar()
        thing[4] = [float(self.cliff_dist()), 0, 0]
        if self.state == "LOCATE":
            self.state = "FORWARD"
            thing[4][1] = 0
        elif self.state == "TURN":
            self.state = "FORWARD"
            thing[4][1] = 2
        else:
            self.state = "TURN"
            thing[4][1] = 1
        return thing

    def turn(self, direction, mag=50):
        direc = str(direction).upper()
        if direc == "LEFT":
            motor_level(mag, motL)
            motor_level(-mag, motR)
        elif direc == "RIGHT":
            motor_level(mag, motR)
            motor_level(-mag, motL)
        else:
            print(direction, "is not a valid direction to turn.")

    def cliff_dist(self):  # in cm, good for ~9 to ~30
        volt = (self.cliff.value * 3.3) / 65536
        try:
            return (volt ** -1.173) * 29.988
        except ZeroDivisionError:
            print("The cliff sensor is giving bad readings.")
            time.sleep(0.05)
            return "-1"

    def cliff_det(self):
        dist = self.cliff_dist()
        try:
            if dist >= 20:
                return True
            else:
                return False
        except TypeError:
            return True

    def grab_sonar(self):
        try:
            distL = self.sL.distance
        except Exception:
            print("The left sonar is not detected.")
            distL = 0
        try:
            distR = self.sR.distance
        except Exception:
            print("The right sonar is not detected.")
            distR = 0
        try:
            distF = self.sF.distance
        except Exception:
            print("The front sonar is not detected.")
            distF = 0
        return [distL, distF, distR]

    def send_bytes(self, origin_data):
        for count0, d_list in enumerate(origin_data):
            for value in d_list:
                self.serial.write(bytes(struct.pack("d", float(value))))  # use struct.unpack to get float back

    def read_uart(self, numbytes=4):
        data = self.serial.read(numbytes)
        data_string = None
        er = None
        if data is not None:
            try:
                data_string = struct.unpack("d", data)
            except Exception as e:
                print("Error message:", e)
                er = e
        return (data_string, er)
    
    def evt_handler(self):
        o = self.locate()  # changes state and grabs sensor data
        self.origins.append(o)    # stores sensor data
        self.send_bytes(o)  # sends data
        timer_set()  # resets timer


def timer_event():
    global timer_time

    if (timer_time is not None) and time.monotonic() >= timer_time:
        timer_time = None
        return EVENT_TIMER
    else:
        return EVENT_NONE


def timer_set():
    global timer_time
    timer_time = time.monotonic() + DATA_SEND_INTERVAL


# testing parameters
testing = True
print_time = .2
test_q = "Y"

# initializing variables
last = time.monotonic()
origins = [[(0, 0, 0), (0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]]
i = 0
s_thold = 25  # in cm
start_button = None
# UART stuff for RPI
rpi_serial = UART(TX, RX, baudrate=20000, timeout=0.3)
# creating instance of state machine
goomba = state_machine(motL, motR, encL, encR, lis3, lsm6, IR, sonar, rpi_serial)
while True:  # actual main loop
    ble.start_advertising(advertisement)
    while not ble.connected:  # for testing while connected
        dists = goomba.grab_sonar()
        print("Sonar distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
        encs = [encL.position, encR.position]
        print('Magnetometer: {0:10.2f}X {1:10.2f}Y {2:10.2f}Z uT'.format(*lis3.magnetic))
        print('Encoders: {0:10.2f}L {1:10.2f}R pulses'.format(*encs))
        print("Acceleration: {:.2f} {:.2f} {:.2f} m/s^2".format(*lsm6.acceleration))
        cliff = goomba.cliff_dist()
        print("Cliff distance:", cliff, "cm")
        print("Cliff?", goomba.cliff_det(), "cm")

        time.sleep(print_time)
        if test_q != "SKIP":
            mot_test0 = input("Test motors? \nY or N ")
            mot_test1 = mot_test0.upper()
            test_q = mot_test0.upper()
            if (mot_test1 == "END"):
                break
            if mot_test1 != "SKIP":
                uart_test0 = input("Test UART? \nY or N ")
                uart_test1 = uart_test0.upper()
                if uart_test1 == "SKIP":
                    test_q = uart_test1
                if (uart_test1 == "END"):
                    break
                elif mot_test1 == "Y":
                    duration = float(input("How long? "))
                    motor_test(motL, motR, duration)
                elif uart_test1 == "Y":
                    c_det = int(goomba.cliff_det())
                    o = [lis3.magnetic, encs, lsm6.acceleration, dists, (c_det, 0, 3)]
                    goomba.send_bytes(o)

        if RPI_CS.value is True:  # idea for signalling when to read from RPi
            data = [0, 0, 0]
            er = [0, 0, 0]
            print("RPi sending data!")
            for i in range(3):
                data[i], er[i] = goomba.read_uart()
            print(data)


    while ble.connected:
        if testing:
            if time.monotonic() - last > print_time:
                print(goomba.state)
                last = time.monotonic()
        if RPI_CS is True:  # initial version of read functionality
            rpi_in, er = goomba.read_uart()
            if int(rpi_in) == 666:  # received start signal
                state_in = goomba.read_uart()
                if state_in[0] is float:
                    new_state = int(state_in[0])
                    if new_state == 0:
                        goomba.state == "IDLE"
                    elif new_state == 1:
                        goomba.state == "FORWARD"
                    elif new_state == 2:
                        goomba.state == "TURN"
                    elif new_state == 3:
                        goomba.state == "TURN"
                    elif new_state == 4:
                        goomba.state == "LOCATE"
                    elif new_state == 5:
                        goomba.state == "IDLE"
                        print("\"bad\" sent from RPi. Check SendThread class")
                    else:
                        print("Invalid state from RPi")
                else:
                    print("Signal interrupted from RPi")

        if uart.in_waiting:
            packet = Packet.from_stream(uart)
            if isinstance(packet, ButtonPacket):
                if packet.pressed:
                    if packet.button == B1:
                        start_button = True  # may need to fix this?
                        print("Button 1 pressed! It was a reset?!")
                    elif packet.button == B2:
                        # some way to send origins in an organized manner?
                        print("Button 2 pressed! Data by UART WIP.")
                        c_det = goomba.cliff_dist()
                        dists = goomba.grab_sonar()
                        encs = (goomba.encL.position, goomba.encR.position)
                        o = [lis3.magnetic, encs, lsm6.acceleration, dists, (c_det, 0, 3)]
                        goomba.send_bytes(o)
                    elif packet.button == B3:
                        # some way to send origins in an organized manner?
                        print("Button 3 pressed! Toggled print statements.")
                        testing = not testing

        if goomba.state == "IDLE":
            goomba.idle(start_button)
            start_button = False
        elif goomba.state == "LOCATE":
            goomba.evt_handler()

        if goomba.state != "IDLE":
            if timer_time is None:  # timer acts independently and does not use locate to not change state
                timer_set()
            if timer_event() == EVENT_TIMER:
                c_det = goomba.cliff_dist()
                dists = goomba.grab_sonar()
                encs = (goomba.encL.position, goomba.encR.position)
                if goomba.state == "FORWARD":
                    comm = 1
                if goomba.state == "TURN":
                    comm = 2
                o = [lis3.magnetic, encs, lsm6.acceleration, dists, (c_det, comm, 1)]
                goomba.send_bytes(o)

        elif goomba.state == "FORWARD":
            goomba.forward()
            if (goomba.sL.distance <= s_thold) and (goomba.sF.distance <= s_thold):
                goomba.evt_handler()
                goomba.go = "RIGHT"
            elif (goomba.sR.distance <= s_thold) and (goomba.sF.distance <= s_thold):
                goomba.evt_handler()
                goomba.go = "LEFT"
            elif goomba.cliff_det() is True:
                goomba.evt_handler()
                goomba.go = "RIGHT"
            elif start_button is True:  # deprecated implementation?
                goomba.state = "IDLE"
                start_button = False

        elif goomba.state == "TURN":
            goomba.turn(goomba.go)
            if (goomba.sF.distance >= s_thold) and (goomba.cliff_det() is False):
                goomba.evt_handler()
            elif start_button is True:  # deprecated implementation?
                goomba.state = "IDLE"
                start_button = False

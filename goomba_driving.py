# goomba_driving.py - ME 106 Project Code
#   For use with the EduShields TriplerBaseboard
#
# Written by Ryan Sands (sandsryanj@gmail.com)
#   v0.8 25-Mar-2021 Drafting of functions for sensors and classes for state machine
#   v0.9 26-Mar-2021 Initial version of state machine to handle 5 events. Needs to finish sensor handling

import board
from board import SCL, SDA, SCK, MOSI, MISO, RX, TX
import pwmio
import rotaryio
import pulseio  # pulseio for IR sensor
import time
import busio  # for i2c and SPI
from busio import UART
import adafruit_lis3mdl  # magnetometer
import adafruit_irremote
import adafruit_hcsr04  # sonar sensor

from adafruit_motor import motor  # need to look into what i can do with this

from math import cos
from math import sin

import struct

'''
 we have 7 digital pins, 6 analog pins explicitly - D13, D12, D11, D10, D9, D6, D5, D2
 Analog available A0, A1, A2, A3, A4, A5 just take out IW
 gives tentative total of 13 pins at our disposal, no I2C pins?
 Sck, MO, MI pins can be used for general purpose IO (GPIO) or SPI
 so IR stuff goes to other board
'''
PIN_IR = MOSI  # we have space if we don't use SPI
PIN_IRLED = MISO
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
sonarF = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_F0, echo_pin=PIN_SON_F1)
sonarR = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_R0, echo_pin=PIN_SON_R1)

i2c = busio.I2C(SCL, SDA)
lis3 = adafruit_lis3mdl.LIS3MDL(i2c)

pulsein = pulseio.PulseIn(PIN_IR, maxlen=150, idle_state=True)
decoder = adafruit_irremote.GenericDecode()

# output pins
PIN_MOTL0 = board.A4  # we need to take out the IW before we use this
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
''' bluetooth stuff
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
'''
def error(err_string):
    raise Exception(err_string)


def motor_level(level, mot):  # input is range of percents, -100 to 100
    mot.throttle = float(level/100)


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

# TODO IR stuff
'''
 puslein object can be read as a list. after pulsein[0] which is the time spent waiting for first detected high
 in ms, it then gives how long it detected that pulse at pulsein[1]. this gives the duration of pulse high for odd
 numbers and the time waiting for even numbers of pulsein[x]
 if i set the IR LED to a 50% duty cycle with a like 1 kHz frequency, i can say that if the difference in is
 pulsein[i]-pulsein[i-1] >= 100, there is a drop?

 the decoder will remove long starting pulse width
 read_pulses function will wait for a remote control press to be detected
 (or if one had previously happened and not been processed it will grab it instead
'''

# commented out because it holds up program waiting for signal w/ no IR
#pulse = decoder.read_pulses(pulsein)  # look at the decoder code and note behavior
#pulse2 = decoder.read_pulses(pulsein)
def fuzzy_pulse_compare(pulse1, pulse2, fuzzyness=0.1):  # interpret matching signals from IR sensor
    if len(pulse1) != len(pulse2):
        return False
    for i in range(len(pulse1)):
        threshold = int(pulse1[i] * fuzzyness)
        if abs(pulse1[i] - pulse2[i]) > threshold:
            return False
    return True
#fuzzy_pulse_compare(pulse, pulse2)


def cliff_function(ir_stuff):  # TODO get some true or false input when it detects a cliff
    ir_stuff = 0


# UART stuff for RPI
uart_rpi = UART(TX, RX, baudrate=9600, timeout=0.5)

def send_bytes(origin_data):  # TODO send bytes representing data through SPI 
    for count0, d_list in enumerate(origin_data):
        for value in d_list:
            uart_rpi.write(bytes(struct.pack("d", float(value))))  # use struct.unpack to get float back
#values = [[1234, 1237], ("21.967", "-62.146", "-4.516")]
#send_bytes(values)

def read_uart(numbytes):
    data = uart_rpi.read(numbytes)
    if data is not None:
        data_string = struct.unpack("d", data)
        print(data_string, end="")
# may want to use SCK pin as a chipselect pin for communication

# States of state machine
class state_machine():
    go = None
    debug = True

    def __init__(self, initial_state, motL, motR, magneto):
        self.state = str(initial_state).upper()
        self.mot1 = motL
        self.mot2 = motR
        self.magnet = magneto

    def forward(self, speed):
        motor_level(speed, self.mot1)
        motor_level(speed, self.mot2)
    
    def idle(self, ignite):
        motor_level(0, self.mot1)
        motor_level(0, self.mot2)
        if ignite is True:
            self.state = "LOCATE"
    
    def locate(self, encL, encR, start):  # for initialization
        thing = [[(0, 0, 0), (0, 0)]]
        thing[0][0] = self.magnet.magnetic
        thing[0][1] = (encL.position, encR.position)
        self.state = "FORWARD"
        if debug is True:
            print(thing)
            time.sleep(0.05)
        return thing
    
    def locate(self, encL, encR):  # for use between turning and forward
        thing = [[(0, 0, 0), (0, 0)]]
        thing[0][0] = self.magnet.magnetic
        thing[0][1] = (encL.position, encR.position)
        self.state = "TURN"
        if debug is True:
            print(thing)
            time.sleep(0.05)
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


# Main loop
vector_array = {}
origins = [[(0, 0, 0), (0, 0)]]  # initializing the list 
i = 0
start = "IDLE"
s_threshhold = 30  # in cm
goomba = state_machine(start, motL, motR, lis3)
while True:  # actual main loop
    #start_button = some input here
    if goomba.state == "IDLE":
        goomba.idle(start_button) # button input could be used from the bluefruit
    elif goomba.state == "LOCATE":
        origins[i] = goomba.locate(encL, encR, TRUE)
        i += 1
    elif goomba.state == "FORWARD":
        goomba.forward(60)
        if (sonarL.distance <= s_threshhold) and (sonarF.distance <= s_threshhold):
            origins[i] = goomba.locate(encL, encR)
            i += 1
            goomba.go = "RIGHT"
        elif (sonarR.distance <= s_threshhold) and (sonarF.distance <= s_threshhold):
            origins[i] = goomba.locate(encL, encR)
            i += 1 
            goomba.go = "LEFT"
        elif cliff_function() is True:
            origins[i] = goomba.locate(encL, encR)
            i += 1 
            goomba.go = "RIGHT"
        elif start_button is True:
            goomba.state = "IDLE"
        
    elif goomba.state == "TURN":
        goomba.turn(goomba.go)
        if (sonarF.distance >= s_threshhold) and (cliff_function() is False):
            origins[i] = goomba.locate(encL, encR)
            i += 1
            goomba.state = "FORWARD"
        elif start_button is True:
            goomba.state = "IDLE"

# this state machine can take 5 events:
# true or false input for cliffs
# less than threshold value on any of the left front or right sonar
# and taking some sort of reset button

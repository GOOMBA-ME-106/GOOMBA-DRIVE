# goomba_driving.py - ME 106 Project Code
#   For use with the EduShields TriplerBaseboard
#
# Written by Ryan Sands (sandsryanj@gmail.com)
#   v0.8 25-Mar-2021 Drafting of functions for sensors and classes for state machine

import board
from board import SCL, SDA
import pwmio, rotaryio, pulseio  # pulseio for IR sensor
# import analogio -- not sure if we need this yet
import time, busio
import adafruit_lis3mdl # magnetometer
import adafruit_irremote
import adafruit_hcsr04  # sonar sensor
from adafruit_motor import motor  # need to look into what i can do with this

from numpy import cos
from numpy import sin

from goomba_state import state
''' probably unnecessary bluetooth stuff
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
'''

'''
 we have 7 digital pins,  6 analog pins explicitly - D13, D12, D11, D10, D9, D6, D5, D2
 Analog available A0, A1, A2, A3, A4, A5 just take out IW
 gives tentative total of 13 pins at our disposal, no I2C pins
 Sck, MO, MI pins can be used for general purpose IO (GPIO), but we use SPI
 so IR stuff goes to other board
'''
#PIN_IR = board.A2 move these pins to the thunderboard
#PIN_IRLED = board.A3

# sensor input pins
#PIN_SON_L0 = board.D11
#PIN_SON_L1 = board.D10
#PIN_SON_F0 = board.D9
#PIN_SON_F1 = board.D6
#PIN_SON_R0 = board.D5
#PIN_SON_R1 = board.D2

#PIN_ENC_L0 = board.A0
#PIN_ENC_L1 = board.A1
#PIN_ENC_R0 = board.D12
#PIN_ENC_R1 = board.D13

encL = rotaryio.IncrementalEncoder(PIN_ENC_L0, PIN_ENC_L1)
encR = rotaryio.IncrementalEncoder(PIN_ENC_R0, PIN_ENC_R1)

sonarL = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_L0, echo_pin=PIN_SON_L1) # sonar dist in cm
sonarF = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_F0, echo_pin=PIN_SON_F1)
sonarR = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_R0, echo_pin=PIN_SON_R1)


i2c = busio.I2C(SCL, SDA)
lis3 = adafruit_lis3mdl.LIS3MDL(i2c)
pulsein = pulseio.PulseIn(PIN_IR, maxlen=150, idle_state=True)
decoder = adafruit_irremote.GenericDecode()

# output pins
PIN_MOTL0 = board.A4 # we need tot take out the IW before we use this
PIN_MOTL1 = board.A5
PIN_MOTR0 = board.A2 # these pins replacing IR stuff
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
blink_time = 0.1
last_time = 0

# States of state machine
class forward_state(state):
    def on_event(self, event, dist, ang):
        if event == "front":
            # some way to output distance of wall here
            return ["turn","left"] # i want to check for multiple events at once?
            # or could i just say turn on direction until there is no longer a front event
            # also want something in here to store distance of object

        elif event == "left":
            # some way to output distance of wall here
            stuff = 0
        elif event == "right":
            # some way to output distance of wall here
            stuff = 0
        elif event == "cliff":
            stuff = 0


class turn_state(state):
    def __init__(self):
        again = 1
        noty= 2 
        stuff_here = 1
        testing_stuff = again + noty * stuff_here


class idling_state(state):
    def __init__(self):
        idk = again


class state_machine(object):
    def __init__(self, initial_state):
        self.state = idling_state()


def error(err_string):
    #print(err_string)
    raise Exception(err_string)

def motor_level(level,mot): # input is range of percents, -100 to 100
    mot.throttle = float(level/100)

# not sure about including duration cause i'll need it to keep checking
def turning(direction):
    mag = 75
    direc = str(direction).upper()
    if direction == "LEFT":
        motor_level(mag, motL)
        motor_level(-mag, motR)
        
    elif direction == "RIGHT":
        motor_level(mag, motR)
        motor_level(-mag, motL)
    else:
        print(direction,"is not a valid direction to turn.")

def distance(enc_change0, enc_change1): # is there some way to use magnetometer w/ this?
    enc_change = (enc_change0 + enc_change1)/2
    dist = enc_change * constant
    return dist

def angle(enc_change0, enc_change1, prior_ang=0): # is there some way to use magnetometer w/ this to verify?
    enc_change = (enc_change0 + enc_change1)/2
    ang = enc_change * constant
    ang_rad = ang * (3.141592/180) + prior_ang
    return ang_rad

def new_vect(ang, dist): # takes radians and cm
    vect = []
    vect[0] = float(dist) * cos(ang)
    vect[1] = float(dist) * sin(ang)
    return vect

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

pulse = decoder.read_pulses(pulsein) # look at the decoder code and note behavior
pulse2 = decoder.read_pulses(pulsein)
def fuzzy_pulse_compare(pulse1, pulse2, fuzzyness=0.1): #interpret matching singals from IR sensor
    if len(pulse1) != len(pulse2):
        return False
    for i in range(len(pulse1)):
        threshold = int(pulse1[i] * fuzzyness)
        if abs(pulse1[i] - pulse2[i]) > threshold:
            return False
    return True
#fuzzy_pulse_compare(pulse, pulse2)

def cliff_dist(ir_stuff):
    ir_stuff = 0

def vector_store(vec_list, outfile):
    # JSON stuff here?
    json = "jason"


# Main loop
vector_array = {}
origins = [] # would this part be easier with object oriented programming?
i = 0


while True:
    if i == 0:
        origins[i][1] = (0,0)
    if event == "turning":
        i += 1
        origins[i][0] = lis3.magnetic
    mag_x, mag_y, mag_z = origins[i][0]
    dists = [sonarL.distance, sonarF.distance, sonarR.distance]
    distL, distF, distR = dists
    print("Distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
    print('Magnetometer: {0:10.2f}X {1:10.2f}Y {2:10.2f}Z uT'.format(mag_x, mag_y, mag_z))
    print('Encoders: {0:10.2f}L {1:10.2f}R pulses'.format(encL.position, encR.position))

    time.sleep(0.25)

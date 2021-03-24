# goomba_driving.py - ME 106 Project Code
#   For use with the EduShields TriplerBaseboard
#
# Written by Ryan Sands (sandsryanj@gmail.com)
#   v0.8 22-Mar-2021 Drafting of functions for sensors and classes for state machine

import board
from board import SCL, SDA
import pwmio, rotaryio, pulseio  # pulseio for IR sensor
# import analogio -- not sure if we need this yet
import time, busio
import adafruit_lis3mdl # magnetometer
import adafruit_irremote
import adafruit_hcsr04  # sonar sensor
''' probably unnecessary bluetooth stuff
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
'''
from adafruit_motor import motor  # need to look into what i can do with this

from numpy import cos
from numpy import sin

from goomba_state import state

'''
 we have 7 digital pins,  6 analog pins explicitly - D13, D12, D11, D10, D9, D6, D5, D2
 Analog available A0, A1, A2, A3, A4, A5 just take out IW
 gives tentative total of 15? pins at our disposal, no I2C pins
 Sck, MO, MI pins can be used for general purpose IO (GPIO),
 but these also have special function for communicating by Serial Peripheral Interface
'''

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
PIN_IR = board.A2
encL = rotaryio.IncrementalEncoder(PIN_ENC_L0, PIN_ENC_L1)
encR = rotaryio.IncrementalEncoder(PIN_ENC_R0, PIN_ENC_R1)

sonarL = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_A0, echo_pin=PIN_SON_A1) # sonar dist in cm
sonarF = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_A0, echo_pin=PIN_SON_A1)
sonarR = adafruit_hcsr04.HCSR04(trigger_pin=PIN_SON_A0, echo_pin=PIN_SON_A1)


i2c = busio.I2C(SCL, SDA)
lis3 = adafruit_lis3mdl.LIS3MDL(i2c)
pulsein = pulseio.PulseIn(PIN_IR, maxlen=150, idle_state=True)
decoder = adafruit_irremote.GenericDecode()

# output pins
PIN_MOTL0 = board.A4 #these share channels with on board rot enc so don't twist it while in use
PIN_MOTL1 = board.A5
PIN_MOTR0 = board.SCK
PIN_MOTR1 = board.MI
#PIN_IRLED = board.A3? may want to have it controlled for the freq or duty because of how the pulse library works?
# dylan says he has another board so we can use that instead for this

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
# LIS_MAX = 100 # idk how to interpret magnetometer yet. might want to use it to get close to original location?

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
        testing_stuff = again + noty - stuff_here


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
# brings up issue of how do I reverse direction? prob need two pins for each motor
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

def distance(enc_change0, enc_change1): # is there some way to use magentometer w/ this?
    enc_change = (enc_change0 + enc_change1)/2
    dist = enc_change * constant
    return dist

def angle(enc_change0, enc_change1, prior_ang=0): # is there some way to use magentometer w/ this to verify?
    enc_change = (enc_change0 + enc_change1)/2
    ang = enc_change * constant
    ang_rad = ang * (3.141592/180) + prior_ang
    return ang_rad

def new_vect(ang, dist): # takes radians
    vect = []
    vect[0] = dist*cos(ang)
    vect[1] = dist*sin(ang)
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

def vector_store(vec_list,outfile):
    # JSON stuff here?
    json = "jason"


# Main loop
vector_array = {}
origins = [] #would this be easier with object oriented programming?
i = 0

#print('X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT'.format(mag_x, mag_y, mag_z))
while True:
    origins[i][0] = lis3.magnetic
    #origins[i][1] starts at (0,0) in 2D, will add vector calc next pass
    mag_x, mag_y, mag_z = origins[i][0]
    dists = [sonarL.distance,sonarF.distance,sonarR.distance]
    distL, distF, distR = dists
    print("Distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
    print(origins[i])
    time.sleep(0.25)
    i += 1
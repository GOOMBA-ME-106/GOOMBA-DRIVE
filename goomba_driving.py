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
# import analogio  # not sure if we need this yet
import time
import busio
import adafruit_lis3mdl  # magnetometer
import adafruit_irremote
import adafruit_hcsr04  # sonar sensor
from adafruit_bus_device.spi_device import SPIDevice
from adafruit_motor import motor  # need to look into what i can do with this

from math import cos
from math import sin

from goomba_state import state
''' probably unnecessary bluetooth stuff
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
'''

'''
 we have 7 digital pins, 6 analog pins explicitly - D13, D12, D11, D10, D9, D6, D5, D2
 Analog available A0, A1, A2, A3, A4, A5 just take out IW
 gives tentative total of 13 pins at our disposal, no I2C pins
 Sck, MO, MI pins can be used for general purpose IO (GPIO), but we use SPI
 so IR stuff goes to other board
'''
PIN_IR = board.TX  # move these to the thunderboard?
#PIN_IRLED = board.A3
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
PIN_MOTL0 = board.A4  # we need tot take out the IW before we use this
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
raspi = SPIDevice(spi, cs, baudrate=5000000, polarity=0, phase=0)  # need to look up for our rpi
'''
 the SPI device class only supports devices with a chip select
 and whose chip select is asserted with a low logic signal.
 otherwise we have to lock/unlock spi and enable/disable cs manually
'''

DUTY_MAX = 2**16-1
blink_time = 0.1
last_time = 0

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

# IR stuff
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


def cliff_function(ir_stuff):
    ir_stuff = 0


def vector_store(vec_list, outfile):
    # JSON stuff here?
    json = "jason"

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

# to try multiple I2C devices for sunday
#while not i2c.try_lock():
    #pass
 
try:
    print("I2C addresses found:", [hex(device_address) for device_address in i2c.scan()])
except: 
    print("No I2C addresses found")

# Unlock I2C now that we're done scanning.
i2c.unlock()
# https://e2e.ti.com/blogs_/b/analogwire/posts/how-to-simplify-i2c-tree-when-connecting-multiple-slaves-to-an-i2c-master


# States of state machine
class state_machine():
    go = None

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
        if ignite = True:
            self.state = "LOCATE"
    
    def locate(self, encL, encR, start):  # for initialization
        thing = [[(0, 0, 0), (0, 0)], [(0, 0, 0), (0, 0)]]
        thing[0][0] = self.magnet.magnetic
        thing[0][1] = (encL.position, encR.position)
        self.state = "FORWARD"
        return  thing
    
    def locate(self, encL, encR):  # for use between turning and forward
        thing = [[(0, 0, 0), (0, 0)], [(0, 0, 0), (0, 0)]]
        thing[0][0] = self.magnet.magnetic
        thing[0][1] = (encL.position, encR.position)
        self.state = "TURN"
        return  thing

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
origins = [[(0, 0, 0), (0, 0)], [(0, 0, 0), (0, 0)]]  # would this part be easier with object oriented programming?
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
            i += 1 # i want the origin to only grab one list of values when it starts turning
            # then one when it stops
            goomba.go = "RIGHT"
        elif (sonarR.distance <= s_threshhold) and (sonarF.distance <= s_threshhold):
            origins[i] = goomba.locate(encL, encR)
            i += 1 
            goomba.go = "LEFT"
        elif cliff_function() = True:
            origins[i] = goomba.locate(encL, encR)
            i += 1 
            goomba.go = "RIGHT"
        elif start_button = True:
            goomba.state = "IDLE"
        
    elif goomba.state == "TURN":
        goomba.turn(goomba.go)
        if (sonarF.distance >= s_threshhold) and (cliff_function() = False):
            origins[i] = goomba.locate(encL, encR)
            i += 1
            goomba.state = "FORWARD"
        elif start_button = True:
            goomba.state = "IDLE"

# this state machine can take 5 events:
# true or false input for cliffs
# less than threshold value on any of the left front or right sonar
# and taking some sort of reset button
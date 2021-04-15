import board
import time
from analogio import AnalogIn

PIN_IR = board.A3


def get_cliff_dist(pin):
    an_in = AnalogIn(pin)
    volt = (an_in.value * 3.3) / 65536
    return (volt ** -1.173) * 29.988


while True:
    print(get_cliff_dist(PIN_IR))
    time.sleep(0.1)

import pulseio, digitalio
import board
import time
from analogio import AnalogIn

PIN_IR = board.A3
PIN_IRLED = board.D13

led = digitalio.DigitalInOut(PIN_IRLED)
led.direction = digitalio.Direction.OUTPUT

analog_in = AnalogIn(PIN_IR)


def get_voltage(pin):
    return (pin.value * 3.3) / 65536


blink1 = 5*(10**6) #in ns
blink2 = 500
last1 = time.monotonic_ns()
last2 = time.monotonic_ns()
led.value = False

i = 0
while i < 5:
    if time.monotonic_ns() - last1 >= blink1:
        led.value = not led.value
        print((get_voltage(analog_in),int(led.value)*3.3))
        last1 = time.monotonic_ns()
        i += 0.5
    if time.monotonic_ns() - last2 >= blink2:
        print((get_voltage(analog_in),int(led.value)*3.3))
        last2 = time.monotonic_ns()
        print(last2)

print(i,"pulses sent")
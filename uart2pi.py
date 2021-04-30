import board
from board import MISO, RX, TX
import digitalio
from busio import UART
import struct
import time



led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT


PIN_RPI_IN = MISO
RPI_CS = digitalio.DigitalInOut(PIN_RPI_IN)
RPI_CS.direction = digitalio.Direction.INPUT


# UART stuff for RPI
rpi_write = UART(TX, RX, baudrate=9600)
#rpi_read = UART(SCL, SDA, baudrate=9600)  # need pull up resistor for this?

def send_bytes(rpi, d_list):
    rpi.write(struct.pack("d", 0))
    for d_tuple in d_list:
        #for value in d_tuple:  # this somehow broke after prior testing, idk how to fix
        rpi.write(struct.pack("f", d_tuple[0]))
        rpi.write(struct.pack("f", d_tuple[1]))
        rpi.write(struct.pack("f", d_tuple[2]))   # brute force loop
    rpi.write(struct.pack("f", 999))


def read_uart(rpi, numbytes=16):
    data = rpi.read(numbytes)
    data_string = None
    if data is not None:
        try:
            data_string = struct.unpack("d", data)
            print(data_string)
        except Exception as e:
            print("No data found. \nError message:", e)
    return data_string

blink_time = .7
last_time = time.monotonic()
origins = [(0, 12.0, 2.3), (3, 254.1, 0), (5, 6, 7), (8, 9, 10), (1, 2, 3)]

while True:
    if time.monotonic() - last_time > blink_time:
        last_time = time.monotonic()
        send_bytes(rpi_write, origins)
        data = rpi_write.read(36)
        print(data)
        if data is not None:
            led.value = True

            # convert bytearray to string
            data_string = ''.join([chr(b) for b in data])
            print(data_string, end="")

            led.value = False
            print("\n")

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


# this was kinda working, but wouldn't show past last bytes
# modified rates between the two, lost ending 
def send_bytes(rpi, origin_data):
    rpi.write(struct.pack("d", 999))
    for count0, d_list in enumerate(origin_data):
        for value in d_list:
            rpi.write(struct.pack("d", value))  # use struct.unpack to get float back
            rpi.write(struct.pack("d", 1.11))  # indicate break?
    rpi.write(struct.pack("d", 666))


# UART stuff for RPI
rpi_write = UART(TX, RX, baudrate=9600)


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


blink_time = 1
last_time = time.monotonic()
origins = [(0, 12.0, 2.3), (3, 254.1, 0), (5, 6, 7), (8, 9, 10), (1, 2, 3)]
while True:
    if time.monotonic() - last_time > blink_time:
        last_time = time.monotonic()
        send_bytes(rpi_write, origins)
        #send_bitties(rpi_write, origins)
        data = rpi_write.read(36)
        print(data)
        if data is not None:
            led.value = True

            # convert bytearray to string
            data_string = ''.join([chr(b) for b in data])
            print(data_string, end="")

            led.value = False
            print("\n")

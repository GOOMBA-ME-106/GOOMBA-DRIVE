import random
import time
import serial

com_devicefile = "COM4"                  # Windoze example
#com_devicefile = "/dev/tty.usbmodem146201" # macOS example

ser = serial.Serial(com_devicefile)

while True:
    n = random.randint(1, 10)
    print("Sending:", n, end="")

    s = str(n) + "\r\n"
    ser.write(bytes(s, "utf-8"))

    s2 = ser.read()
    print("status: '"+s2+"'", end="")
    time.sleep(0.05)

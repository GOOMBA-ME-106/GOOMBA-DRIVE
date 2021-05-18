# simpler interface to serial port input and demo
#
# Written by Eric B. Wertz (eric@edushields.com)
# Last updated 29-Apr-2020

import serial

COM_DEVICEFILE = "COM8"                    # Windoze example
#COM_DEVICEFILE = "/dev/tty.usbmodem146201" # macOS example

ser = serial.Serial(COM_DEVICEFILE, timeout=0.0)

g_ignore_newline = False
g_buffer = ""

def com_port_readline():
    global g_ignore_newline, g_buffer

    while True:
        b = ser.read(1)
        if len(b) == 0:
            return None
        c = b.decode("utf-8")

        if c == '\r':
            s = g_buffer
            g_buffer = ""
            g_ignore_newline = True           # EOL = "\r\n"
            return s

        if c == '\n':
            if g_ignore_newline:
                g_ignore_newline = False
            else:
                s = g_buffer
                g_buffer = ""
                g_ignore_newline = False
                return s
        else:
            g_ignore_newline = False
            g_buffer += c

    return  None  # NOTREACHED

# FROM HERE DOWN IS THE TEST/DEMO
while True:
    s = com_port_readline()
    if s is not None:
        print(s)


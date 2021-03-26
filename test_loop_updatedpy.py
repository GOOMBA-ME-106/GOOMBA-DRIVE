from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket

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


def motor_test(mot1, mot2, drive_time):
    drive = drive_time/3
    motor_level(60, mot1)
    motor_level(60, mot2)
    time.sleep(drive)
    motor_level(-60, mot1)
    motor_level(-60, mot2)
    time.sleep(drive)
    motor_level(0, mot1)
    motor_level(0, mot2)
    time.sleep(drive)

i = 0
event = 0
step = 5
speed1 = 0
speed2 = 0
while True:  # the testing loop
    ble.start_advertising(advertisement)
    while not ble.connected:
        if i == 0:
            origins[i][1] = (0, 0)
            origins[i][0] = lis3.magnetic  # to store vector w/ magnetic data
        if event == "TURN":
            i += 1
            origins[i][0] = lis3.magnetic
        dists = [sonarL.distance, sonarF.distance, sonarR.distance]

        print("Sonar distances: {:.2f}L {:.2f}F {:.2f}R (cm)".format(*dists))
        print('Magnetometer: {0:10.2f}X {1:10.2f}Y {2:10.2f}Z uT'.format(lis3.magnetic))
        print('Encoders: {0:10.2f}L {1:10.2f}R pulses'.format(encL.position, encR.position))

        time.sleep(0.5)
        mot_test0 = input("Test motors? /n Y or N ")
        mot_test1 = mot_test0.upper
        if mot_test1 == "Y":
            duration = float(input("How long? "))
            motor_test(motL, motR, duration)
        elif mot_test1 == "END":
            break

    # Now we're connected

    while ble.connected:
        if uart.in_waiting:
            packet = Packet.from_stream(uart)
            if isinstance(packet, ButtonPacket):
                if packet.pressed:
                    if packet.button == B1:
                        # The 1 button was pressed.
                        print("1 button pressed! It was not very effective.")
                    elif packet.button == UP:
                        # The UP button was pressed.
                        print("UP button pressed! The left motor's speed sharply rose!")
                        if speed1 != 100:
                            speed1 += step
                    elif packet.button == DOWN:
                        # The DOWN button was pressed.
                        print("DOWN button pressed! The  left motor's speed sharply fell!")
                        if speed1 != -100:
                            speed1 += -step
                    elif packet.button == LEFT:
                        # The LEFT button was pressed.
                        print("LEFT button pressed! The right motor's speed sharply fell!")
                        if speed2 != -100:
                            speed2 += -step
                    elif packet.button == RIGHT:
                        # The RIGHT button was pressed.
                        print("RIGHT button pressed! The right motor's speed sharply rose!")
                        if speed2 != 100:
                            speed2 += step
                    elif packet.button == B2:
                        # The 2 button was pressed.
                        print("2 button pressed! It was not very effective.")
                    elif packet.button == B3:
                        # The 3 button was pressed.
                        print("3 button pressed! It was not very effective.")
                    elif packet.button == B4:
                        # The 4 button was pressed.
                        print("4 button pressed! It was not very effective.")
        motor_level()
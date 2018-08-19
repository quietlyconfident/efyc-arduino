''' This module is responsible for talking to the adruino board
    and getting the data out '''
import serial
SERIAL_PORT = serial.Serial('/dev/tty.usbserial-DN03ZSD1', 9600)
while True:
    print(SERIAL_PORT.readline())

import serial
import time

#####################################################
# Notes:
# 1. I need to use Arduino IDE to put code into teensy, but I need
#    to exit out of Ard IDE before running .py bc error keeps coming up otherwise idk why
#####################################################

# initializes serial and adds a 1s time delay
ser = serial.Serial(port = 'COM5', baudrate = 115200)
time.sleep(1)

x = 5.0
y = 5.0

while True:
    # merges values into string with the form: number,number,etc....
    msg = f"{x},{y}\n"
    # bytes() function allows for the data to be sent over
    ser.write(bytes(msg, 'UTF-8'))
    value = ser.readline()
    valueInString = str(value, 'UTF-8')
    print(valueInString)
    # changes x and y values before running the loop again
    x = x + 0.3
    y = y + 0.7
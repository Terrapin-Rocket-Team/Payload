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

# Initialize your simulation state
x = 0.0
y = 0.0
z = 100.0


############ Super basic past code that works (use as benchmark)
while True:
    start_time = time.time()
    # merges values into string with the form: number,number,etc....
    msg = f"{x},{y},{z} \n"
    # bytes() function allows for the data to be sent over
    ser.write(bytes(msg, 'UTF-8'))
    value = ser.readline()
    valueInString = str(value, 'UTF-8')

    end_time = time.time()
    elapsed_time = end_time - start_time

    print(valueInString + f"Elapsed time: {elapsed_time:.4f} seconds")
    # changes x and y values before running the loop again
    x = x + 0.3
    y = y + 0.7
    z = z - 1.5
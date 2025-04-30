import serial
import time
import math
# Set up serial connection to Teensy
ser = serial.Serial('COM5', 115200, timeout=0.1)  # Replace COM5 with whatever port being used for teensy
time.sleep(2)  # Wait for connection to stabilize

def send_state(heading, x, y, angularVelocity):
    msg = f"{heading},{x},{y},{angularVelocity}\n"
    ser.write(msg.encode())

def read_output():
    line = ser.readline().decode().strip()
    try:
        return float(line)
    except ValueError:
        return None

# Initialize your simulation state
heading = 0.0
velocity = 5 # m/s
x = 0
y = 0
angularVelocity = 0


DT = 0.1  # seconds

while True:
    send_state(heading, x, y, angularVelocity)
    servoAngle = read_output()

    if servoAngle is not None:
        print(f"Received angle: {servoAngle:.4f}")

        heading = heading + angularVelocity*DT
        x = x + (velocity*math.cos(heading))*DT
        y = y + (velocity*math.sin(heading))*DT
        send_state(ser, heading, x, y, angularVelocity)
        servoAngle = read_output(ser)    

    time.sleep(DT)

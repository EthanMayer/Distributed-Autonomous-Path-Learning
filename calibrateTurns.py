from Motor import *

import time

def turn(speed=0.5):
    wheels.setMotorModel(int(-4095 * speed), int(-4095 * speed),
                         int(4095 * speed), int(4095 * speed))

def stop():
    wheels.setMotorModel(0, 0, 0, 0)


wheels = Motor()
speed = 0.25

print("Enter an amount of time to turn in seconds.\n")
while True:
    timer = int(input("> "))
    turn(speed)
    time.sleep(timer)
    stop()
    turn(-speed)
    time.sleep(timer)
    stop()
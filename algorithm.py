import time
from timeit import default_timer as timer

from Motor import *
from Ultrasonic import *
from servo import *

import traceback
import logging

from enum import Enum
import sys
import math
import networking.client
from networking.client import *

# Utilities
ultrasonic = Ultrasonic()
ultrasonic.pwm_S = Servo()
wheels = Motor()


def forward(speed=0.5):
    wheels.setMotorModel(int(-4095*speed), int(-4095*speed),
                         int(-4095*speed), int(-4095*speed))


def stop():
    wheels.setMotorModel(0, 0, 0, 0)


def turn(speed=0.5):
    wheels.setMotorModel(int(-4095*speed), int(-4095*speed),
                         int(4095*speed), int(4095*speed))


def getUltrasonicDistance():
    # Average out some values to ensure the reading is sensible since it can fluctuate
    # in complex obstacle courses.
    avg = 0
    count = 10
    for i in range(0, count):
        avg += ultrasonic.get_distance()
        time.sleep(0.02)
    return avg / count


class CarConfig(Enum):
    WhereEam = 1
    sbond75 = 2
    Ethan = 3


# Get `use` for which CarConfig to use:
if len(sys.argv) <= 3:
    # Prompt for input
    use = input("Enter the user of the car. Must be one of " +
                str(list(map(str, CarConfig))) + ", or a dry run will be performed: ")
    isReceivingVehicle = bool(int(input(
        "Is this the navigation vehicle or the receiving vehicle? Enter 0 for navigation, 1 for receiving: ")))
else:
    use = sys.argv[1]
    isReceivingVehicle = bool(int(sys.argv[2]))
    debugWindowEnabled = bool(int(sys.argv[3]))
# Verify `use`:
try:
    carConfig = CarConfig[use]
except KeyError:
    carConfig = None
    print("Unknown name given.")
    exit

# Configuration
d = 25  # Centimeters from car to object at which to stop and scan from
turn_method = 2  # distance polling(0), gyroscope(1), optical flow(2)
dist_epsilon = 30  # For distance polling(0)
angle_epsilon = 1  # For gyroscope(1)

if carConfig == CarConfig.WhereEam:
    middleHoriz = 75
    middleVert = 55
elif carConfig == CarConfig.sbond75:
    middleHoriz = 90
    middleVert = 90
elif carConfig == CarConfig.Ethan:
    pass

# If we are the receiving vehicle, we need to wait for a path first.
if isReceivingVehicle:
    client = Client()
    # recordedPath = client.receivePath()
    recordedPath = [[2, 0.15], [2, 0.25], [3, 0.15]]
else:
    recordedPath = []

# Initialize sensors
if turn_method == 1:
    import board
    import busio
    import adafruit_mpu6050
    i2c = busio.I2C(1, 0)
    mpu = adafruit_mpu6050.MPU6050(i2c)
elif turn_method == 2:
    import flow
    flow = flow.OpticalFlow(show_debug=debugWindowEnabled)

try:
    if not isReceivingVehicle:  # Then we are the navigation vehicle. We are forging a new path!
        while True:
            # Move the ultrasonic sensor to the middle:
            ultrasonic.pwm_S.setServoPwm('0', middleHoriz)
            ultrasonic.pwm_S.setServoPwm('1', middleVert)
            # Allow time to settle:
            time.sleep(0.5)

            # Move forward
            speed = 0.15
            forward(speed)
            start_time = timer()
            flag = True
            while flag:
                for angle in range(middleHoriz - 20, middleHoriz + 30, 10):
                    ultrasonic.pwm_S.setServoPwm('0', angle)
                    c = getUltrasonicDistance()  # Grab distance from bot to object
                    print("dist: " + str(c))
                    if c <= d:
                        print("c <= d")
                        stop()
                        flag = False
                        break
                    time.sleep(0.11)
                for angle in range(middleHoriz + 20, middleHoriz - 30, -10):
                    ultrasonic.pwm_S.setServoPwm('0', angle)
                    c = getUltrasonicDistance()  # Grab distance from bot to object
                    print("dist: " + str(c))
                    if c <= d:
                        print("c <= d")
                        stop()
                        flag = False
                        break
                    time.sleep(0.11)
            end_time = timer()
            print("Time forward " + str(end_time - start_time))
            recordedPath.append([end_time - start_time, speed])

            # Look around, checking distances
            destination_distance = 0
            destination_angle = 0
            for angle in range(middleHoriz - 90, middleHoriz + 90, 10):
                ultrasonic.pwm_S.setServoPwm('0', angle)
                time.sleep(0.5)
                dist = getUltrasonicDistance()
                if dist > destination_distance:
                    destination_distance = dist
                    destination_angle = angle
                print("Recorded distance " + str(dist) +
                      " for angle " + str(angle))
            print("Chose angle " + str(destination_angle))
            ultrasonic.pwm_S.setServoPwm('0', middleHoriz)
            time.sleep(0.5)

            start_time = timer()
            current_degree = 0
            speed = 0
            # Turn
            if destination_angle < 0:
                print("Turning left")
                speed = -0.25
                turn(speed)
            elif destination_angle == 0:
                print("Forward but this shouldn't happen")
                speed = 0.15
                forward(speed)
            elif destination_angle > 0:
                print("Turning right")
                speed = 0.25
                turn(speed)

            if turn_method == 0:
                while True:
                    # Get current distance as we turn
                    c = getUltrasonicDistance()
                    if abs(c - destination_distance) <= dist_epsilon:
                        stop()
                        break
            elif turn_method == 1:
                while True:
                    # Grab current angular velocity z in degrees per second.
                    (dRoll, dPitch, dYaw) = mpu.gyro
                    end_time = timer()
                    current_degree -= (end_time - start_time) * dYaw
                    print("At " + str(current_degree) + " degrees out of " +
                          str(destination_angle) + " degrees")
                    if abs(current_degree - destination_angle) <= angle_epsilon:
                        stop()
                        break
            elif turn_method == 2:
                flow.prepare()
                while True:
                    # Get current distance as we turn
                    c = getUltrasonicDistance()
                    (closestPointToCenter, flowVector) = flow.computeCentermostFlow()
                    current_degree -= math.degrees(
                        flow.computeRadiansOfCameraRotation(c, flowVector))
                    print("At " + str(current_degree) + " degrees out of " +
                          str(destination_angle) + " degrees")
                    if abs(current_degree - destination_angle) <= angle_epsilon:
                        stop()
                        break

            end_time = timer()
            print("Time turning " + str(end_time - start_time))
            recordedPath.append([end_time - start_time, speed])

            # Check for end condition: all polled distance values are less
            # than or equal to some constant end_dist:
            if destination_distance <= d:
                print("End reached")

                # Write the path (as an array of serialized bytes) to a file:
                timestr = time.strftime("%Y%m%d-%H%M%S")
                with open("path_" + timestr, "w") as f:
                    for path in recordedPath:
                        f.write(str(path[0]) + "," + str(path[1]))

                # Connect to the server and send the path
                client = Client()
                client.sendPath(recordedPath)
                break

    # else:  # Then we are the receiving vehicle, so we have a path already.
    while len(recordedPath) != 0:
        data = recordedPath.pop(0)
        forward(data[1])
        time.sleep(data[0])
        data = recordedPath.pop(0)
        turn(data[1])
        time.sleep(data[0])
        # format: hardcoded
        # forward(0.15)
        # time.sleep(2.75)
        # turn(0.25)
        # time.sleep(0.5)
        # forward(0.15)
        # time.sleep(0.22)
        # turn(0.25)
        # time.sleep(0.5)
        # forward(0.15)
        # time.sleep(0.23)
        # turn(0.25)
        # time.sleep(2.5)
        # forward(0.15)
        # time.sleep(4)

finally:
    logging.error(traceback.format_exc())
    stop()
    ultrasonic.pwm_S.setServoPwm('0', middleHoriz)
    ultrasonic.pwm_S.setServoPwm('1', middleVert)

    if client:
        client.close()

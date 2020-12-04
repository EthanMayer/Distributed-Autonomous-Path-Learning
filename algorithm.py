import time
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

from timeit import default_timer as timer
import functools 
import numpy as np

# Car configurations/profiles
class CarConfig(Enum):
    WhereEam = 1
    sbond75 = 2
    Ethan = 3

    # https://stackoverflow.com/questions/53297848/get-enum-name-in-python-without-class-name
    def __str__(self):
        return self.name
# Get `use` for which CarConfig to use:
if len(sys.argv) <= 1: # If we don't have a 1st arg
    # Prompt for input
    use = input("Enter the user of the car. Must be one of " + str(list(map(str, CarConfig))) 
                    + ", or a dry run will be performed: ")
else:
    use = sys.argv[1]
# Verify `use`:
try:
    carConfig = CarConfig[use]
except KeyError:
    carConfig = None
    print("Unknown name given, so this is a dry run: no car motion will "
            + "happen, but the ultrasonic sensor will move.")

# Get `carNumber`:
isReceivingVehicle = None
if len(sys.argv) <= 2: # If we don't have a 2nd arg
    carNumber = input("Is this the navigation vehicle or the receiving vehicle? Enter"
                        + "1 for navigation (or press enter), 2 for receiving: ")
else:
    carNumber = sys.argv[2]
# Verify `carNumber`:
if len(carNumber) == 0 or carNumber == "1":
    isReceivingVehicle = False
elif carNumber == "2":
    isReceivingVehicle = True
else:
    raise Exception("Invalid vehicle number given: " + str(carNumber))

# Get `debugWindowEnabled` (requires Raspberry Pi's desktop environment to be logged into):
if len(sys.argv) >= 4: # If we have a 3rd arg
    debugWindowEnabled = sys.argv[3] == "1"
else:
    debugWindowEnabled = False

# Configuration
d = 27 #25 #30 #20 # Centimeters from car to object at which to stop and scan from
halfW = 8 # Half the width of the car in centimeters.
# TODO: use this value to prevent the car from running into the wall on an edge
if carConfig == CarConfig.WhereEam:
    middleHoriz = 50 # 60 # The center for the servo motor for the ultrasonic sensor, horizontally
    middleVert = 25 # The center for the ultrasonic sensor vertically.
elif carConfig == CarConfig.sbond75:
    middleHoriz = 90
    middleVert = 90
elif carConfig == CarConfig.Ethan:
    pass
else: # Dry run
    middleHoriz = 90
    middleVert = 90
sleep_time_short = 0.01 #0.1
sleep_time_long = 0.4 #0.7 #0.4 #0.2
#end_dist = 10 # If the distance value at each angle is measured to be less than this for each angle,
# the program exits and the end of the course is considered to be reached.

# Algorithm modifiers
stop_cond = 2 # 1 # Selects whether to stop based on 
                  # distance polling(0) [somewhat unreliable depending on obstacle course] 
                  # or with gyroscope(1) [BROKEN], or optical flow(2).
dist_epsilon = 30 #1   # For stop_cond == 0
angle_epsilon = 1      # For stop_cond == 1

# Utilities
ultrasonic=Ultrasonic()
ultrasonic.pwm_S = Servo()  
wheels=Motor()

# first: front-right
# second: back-right
# third: front-left
# fourth: back-left
#def forward(speed=0.3):
#    # wheels.setMotorModel(2000,2000,2000,2000)       #Forward (<--originally)
#    # wheels.setMotorModel(-2000,-2000,-2000,-2000)       #Forward (different on Will's bot)
#    if carConfig == CarConfig.WhereEam:
#        # Need to compensate for not going straight, using an offset:
#        wheels.setMotorModel(int(1500*speed),int(1000*speed),
#                             int(1600*speed),int(1700*speed))
#    elif carConfig == CarConfig.sbond75:
#        wheels.setMotorModel(int(-2000*speed),int(-2000*speed),
#                             int(-2000*speed),int(-2000*speed))
#    elif carConfig == CarConfig.Ethan:
#        pass
# [New]
forwardSpeed = 0.125 #0.15
def forward(speed=forwardSpeed):
    wheels.setMotorModel(int(-4095*speed), int(-4095*speed),
                         int(-4095*speed), int(-4095*speed))
# Pass `None` for `secs` to just set the PWM output to that until the next time
# that the motors are set.
def left_(secs=1, speed=0.6):
    wheels.setMotorModel(int(-500*speed),int(-500*speed),
                         int(2000*speed),int(2000*speed))       #Left  (<--originally)
    if secs:
        time.sleep(secs)
def right_(secs=1, speed=0.6):
    wheels.setMotorModel(int(2000*speed),int(2000*speed),
                         int(-500*speed),int(-500*speed))       #Right (<--originally)  
    if secs:
        time.sleep(secs)
def left(secs=1):
    if carConfig == CarConfig.WhereEam:
        right_(secs)
    elif carConfig == CarConfig.sbond75:
        right_(secs)
    elif carConfig == CarConfig.Ethan:
        pass
def right(secs=1):
    if carConfig == CarConfig.WhereEam:
        left_(secs)
    elif carConfig == CarConfig.sbond75:
        left_(secs)
    elif carConfig == CarConfig.Ethan:
        pass
# [New]
def turn(speed=0.5):
    if carConfig is not None:
        wheels.setMotorModel(int(-4095*speed), int(-4095*speed),
                            int(4095*speed), int(4095*speed))

def getUltrasonicDistance():
    # Average out some values to ensure the reading is sensible since it can fluctuate
    # in complex obstacle courses.
    avg = 0
    count = 10
    for i in range(0, count):
        avg += ultrasonic.get_distance()
    return avg / count

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')

    # If we are the receiving vehicle, we need to wait for a path first.
    if isReceivingVehicle:
        client = Client()
        receivedPath = client.receivePath()
    else:
        receivedPath = None
        recordedPath = []
        recordedDurations = []
        client = None

    # Initialize sensors
    if stop_cond == 1:
        import board
        import busio
        import adafruit_mpu6050

        #print(board.__dict__)
        
        # Normally, this would work, but we have GPIO 0 on the RPi as our gyroscope data 
        # connection.: 
        #i2c = busio.I2C(board.SCL, board.SDA)
        
        # Instead, we use:
        # This doesn't work, todo: i2c = busio.I2C(28, 27)
        i2c = busio.I2C(1, 0) 
        # https://github.com/adafruit/Adafruit_CircuitPython_MPU6050 or try https://github.com/m-rtijn/mpu6050
        # or use http://abyz.me.uk/rpi/pigpio/ , recommended by Will
        # sudo i2cdetect -y 1

        mpu = adafruit_mpu6050.MPU6050(i2c)
    elif stop_cond == 2:
        import flow
        flow = flow.OpticalFlow(show_debug=debugWindowEnabled)

    try:
        while True:
            # Move the ultrasonic sensor to the middle:
            ultrasonic.pwm_S.setServoPwm('0',middleHoriz)
            ultrasonic.pwm_S.setServoPwm('1',middleVert)
            # Allow time to settle:
            time.sleep(sleep_time_long) # 3

            while True:
                # Grab distance from bot to object
                c = getUltrasonicDistance()   
                print("dist: " + str(c)) 

                if c <= d:
                    break

                # Move forward
                # [New]
                speed = forwardSpeed
                forward(speed)
                start_time = timer()
                flag = True
                distances = []
                while True:
                    offset = 32
                    movement = 20
                    distances_prev = distances
                    for angle in range(middleHoriz - offset, middleHoriz + offset, movement):
                        ultrasonic.pwm_S.setServoPwm('0', angle)
                        c = getUltrasonicDistance()  # Grab distance from bot to object
                        if abs(angle-middleHoriz) > 50 and c < 20:
                            adj = abs(2*c) # Angles further off center are less important if they are close
                        else:
                            adj = c
                        print("dist: " + str(c), "adjusted:", adj)
                        if adj <= d:
                            flag = False
                            break
                        distances.append(c)
                        time.sleep(0.11)
                    for angle in range(middleHoriz + offset, middleHoriz - offset, -movement):
                        ultrasonic.pwm_S.setServoPwm('0', angle)
                        c = getUltrasonicDistance()  # Grab distance from bot to object
                        if abs(angle-middleHoriz) > 30:
                            adj = abs(2*c) # Angles further off center are less important if they are close
                        else:
                            adj = c
                        print("dist: " + str(c), "adjusted:", adj)
                        if adj <= d:
                            flag = False
                            break
                        distances.append(c)
                        time.sleep(0.11)
                    # Check if all distances were the same across the arrays by a threshold
                    if functools.reduce(lambda a,b: a and b, np.isclose(distances, distances_prev, 2)):
                        # Speed up
                        print("Speeding up")
                        speed += 0.0013
                        forward(speed)
                    if not flag:
                        break
                end_time = timer()
                print("Time going forward: " + str(end_time - start_time))
                recordedDurations.append((end_time - start_time, speed))
                #time.sleep(sleep_time_short) # Rest CPU for a bit
            print("c <= d")

            # Stop
            wheels.setMotorModel(0,0,0,0)                   #Stop
            
            if not isReceivingVehicle: # Then we are the navigation vehicle. We are forging 
                # a new path!
                # Look around, checking distances
                end = 180
                distances = []
                #angles = [0, 45, 75, 90, end - 75, end - 45, end]
                #[Old] angles = [0, 30, 60, 90, end - 60, end - 30, end]
                destination_distance = None
                destination_angle = None
                for angle in range(middleHoriz - 90, middleHoriz + 90, 10): 
                    #for angle in angles: # range(start, stop, separator)
                    ultrasonic.pwm_S.setServoPwm('0',angle)
                    time.sleep(sleep_time_long)
                    dist = getUltrasonicDistance()
                    if destination_distance is None or dist > destination_distance:
                        destination_distance = dist
                        destination_angle = angle
                    distances.append(dist)
                    print("Recorded distance " + str(dist) + " for angle " + str(angle))
                print("Chose angle " + str(destination_angle) + " with distance " + str(destination_distance))

                # Save this angle
                recordedPath.append(destination_angle)

                # Check for end condition: all polled distance values are less 
                # than or equal to `d`:
                if destination_distance <= d:
                    print("End reached")

                    # Write the path (as an array of serialized bytes) to a file:
                    timestr = time.strftime("%Y%m%d-%H%M%S")
                    with open("path_" + timestr, "wb") as f:
                        outputRobotPath(array.array('B', recordedPath), f)
                    # Connect to the server and send the path
                    client = Client()
                    client.sendPath(recordedPath)
                    break

                # Move sensor back to middle
                ultrasonic.pwm_S.setServoPwm('0',middleHoriz)

                # Wait to allow the sensor to settle
                # (important to prevent reading a larger value again)
                time.sleep(0.1)
            else: # Then we are the receiving vehicle, so we have a path already.
                distances = None
                largest_index = None
                destination_distance = None

                # Check for end condition: all angles popped
                if len(receivedPath) == 0:
                    print("End reached")
                    break

                # "Dequeue" an item off the path:
                destination_angle = receivedPath.pop(0) # Pop off index 0 from the array
                # (see https://docs.python.org/3/library/array.html for more info)

            # Prepare gyro or optical flow variables
            if stop_cond == 1 or stop_cond == 2:
                degrees_total = 90
                degrees_total_prev = degrees_total
                speed = 0
                noMovementCounter = 0

                # ( https://stackoverflow.com/questions/1938048/high-precision-clock-in-python )
                #prev_time = time.time_ns() / 1000 / 1000 / 1000  # ns to seconds
            
            start_time = timer()

            # Go in that direction, reaching it once the sensor reports a distance equal to that
            # found distance (Major NOTE: this doesn't work if the shape of an obstacle is a curve
            # that keeps distance the same regardless of sensor position, but that is unlikely):
            if stop_cond == 2:
                # Optical flow: prepare
                flow.prepare()
            # Turn
            dir = None
            turnSpeed = 0.21 # WORKS but super slow: 0.20 # 0.25
            if destination_angle < 90:
                print("Turning left")
                #left(None)
                speed = -turnSpeed
                turn(speed)
                dir = -1
            elif destination_angle == 90:
                print("Forward but this shouldn't happen")
                speed = forwardSpeed
                forward()
                dir = 0
            elif destination_angle > 90:
                print("Turning right")
                #right(None)
                speed = turnSpeed
                turn(speed)
                dir = 1
            else:
                raise Exception("Unexpected case")
            # Check for the condition to stop turning (stop_cond)
            while True:
                if stop_cond == 1 or stop_cond == 2:
                    degrees_total_prev = degrees_total
                if stop_cond == 0:
                    if isReceivingVehicle:
                        # Not supported
                        raise Exception("Receiving vehicle cannot use distances"
                                            + " since they are not currently sent"
                                            + " by the navigation vehicle")
                    
                    # Get current distance as we turn
                    c = getUltrasonicDistance() 

                    if abs(c - destination_distance) <= dist_epsilon:
                        break
                    # elif destination_distance > c:
                        # We overshot
                    #    print("Overshot by " + str(destination_distance - c))
                    #    break
                elif stop_cond == 1:
                    # Gyroscope
                    # Grab current angular velocity, in a tuple containing
                    # x, y, and z in degrees per second.
                    (dRoll, dPitch, dYaw) = mpu.gyro
                    end_time = timer()
                    degrees_total -= (end_time - start_time) * dYaw
                    print("At " + str(degrees_total) + " degrees, want " +
                          str(destination_angle) + " degrees")
                    if dir == -1:
                        if degrees_total - destination_angle <= angle_epsilon:
                            break
                    elif dir == 1:
                        if destination_angle - degrees_total <= angle_epsilon:
                            break
                    if degrees_total - degrees_total_prev < 0.04:
                        noMovementCounter += 1
                        print("No movement")
                        if noMovementCounter > 30:
                            # Bump up how much we turn
                            speed += 0.005
                            turn(speed)
                            noMovementCounter = 0
                            print("Increasing turn speed")
                elif stop_cond == 2:
                    #current_time = time.time_ns() / 1000 / 1000 / 1000

                    # Get current distance as we turn
                    #c = getUltrasonicDistance() 

                    # Optical flow
                    (closestPointToCenter, flowVector) = flow.computeCentermostFlow()
                    print("Flow reliability (closer to 0 is better): " 
                            + str(flow.reliabilityOfPoint(closestPointToCenter)))
                    
                    rads = flow.computeRadiansOfCameraRotation(c, flowVector)
                    degrees = math.degrees(rads)
                    degrees_total -= degrees
                    
                    #radsTotal = flow.computeRadiansOfCameraRotation(c, flowVector)
                    #degrees_total = math.degrees(radsTotal)

                    print("At " + str(degrees_total) + " degrees, want "
                            + str(destination_angle) + " degrees")
                    #time.sleep(0.1)
                    if dir == -1:
                        if degrees_total - destination_angle <= angle_epsilon:
                            flow.reset()
                            break
                    elif dir == 1:
                        if destination_angle - degrees_total <= angle_epsilon:
                            flow.reset()
                            break
                else:
                    raise Exception("Unimplemented stop_cond")
            
            end_time = timer()
            print("Time turning: " + str(end_time - start_time))
            recordedDurations.append((end_time - start_time, speed))
            # Go forward again, until we reach a wall, in which case we look around again as above.

    #except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, this will be executed.
    #except Exception as e:
    finally:
        logging.error(traceback.format_exc())
        wheels.setMotorModel(0,0,0,0)
        ultrasonic.pwm_S.setServoPwm('0',middleHoriz)
        ultrasonic.pwm_S.setServoPwm('1',middleVert)

        if client:
            client.close()
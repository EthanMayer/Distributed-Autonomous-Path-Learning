import time
from Motor import *
from Ultrasonic import *
from servo import *
import traceback
import logging

from enum import Enum
import sys
import math

# Car configurations/profiles
class CarConfig(Enum):
    WhereEam = 1
    sbond75 = 2
    Ethan = 3
if len(sys.argv) <= 1:
    # Prompt for input
    use = input("Enter the user of the car. Must be one of " + str(list(map(str, CarConfig))) 
                    + ", or a dry run will be performed: ")
else:
    use = sys.argv[1]
try:
    carConfig = CarConfig[use]
except KeyError:
    carConfig = None
    print("Unknown name given, so this is a dry run: no car motion will "
            + "happen, but the ultrasonic sensor will move.")

# Configuration
d = 20 # Centimeters from car to object at which to stop and scan from
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
sleep_time_long = 0.4 #0.2

# Algorithm modifiers
stop_cond = 2 # 1 # Selects whether to stop based on 
                  # distance polling(0) [somewhat unreliable depending on obstacle course] 
                  # or with gyroscope(1) [BROKEN], or optical flow(2).
dist_epsilon = 30 #1   # For stop_cond == 0
angle_epsilon = 5      # For stop_cond == 1

# Utilities
ultrasonic=Ultrasonic()
ultrasonic.pwm_S = Servo()  
wheels=Motor()

# first: front-right
# second: back-right
# third: front-left
# fourth: back-left
def forward():
    # wheels.setMotorModel(2000,2000,2000,2000)       #Forward (<--originally)
    # wheels.setMotorModel(-2000,-2000,-2000,-2000)       #Forward (different on Will's bot)
    if carConfig == CarConfig.WhereEam:
        # Need to compensate for not going straight, using an offset:
        wheels.setMotorModel(1500,1000,1600,1700)
    elif carConfig == CarConfig.sbond75:
        wheels.setMotorModel(-2000,-2000,-2000,-2000)
    elif carConfig == CarConfig.Ethan:
        pass
# Pass `None` for `secs` to just set the PWM output to that until the next time
# that the motors are set.
def left_(secs=1):
    wheels.setMotorModel(-500,-500,2000,2000)       #Left  (<--originally)
    if secs:
        time.sleep(secs)
def right_(secs=1):
    wheels.setMotorModel(2000,2000,-500,-500)       #Right (<--originally)  
    if secs:
        time.sleep(secs)
def left(secs=1):
    if carConfig == CarConfig.WhereEam:
        right_(secs)
    elif carConfig == CarConfig.sbond75:
        left_(secs)
    elif carConfig == CarConfig.Ethan:
        pass
def right(secs=1):
    if carConfig == CarConfig.WhereEam:
        left_(secs)
    elif carConfig == CarConfig.sbond75:
        right_(secs)
    elif carConfig == CarConfig.Ethan:
        pass

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')

    # Initialize sensors
    if stop_cond == 1:
        import board
        import busio
        import adafruit_mpu6050

        # Normally, this would work, but we have GPIO 0 on the RPi as our gyroscope data 
        # connection.: i2c = busio.I2C(board.SCL, board.SDA)
        # Instead, we use:
        # This doesn't work, todo: i2c = busio.I2C(28, 27)
        # sudo i2cdetect -y 1

        mpu = adafruit_mpu6050.MPU6050(i2c)
    elif stop_cond == 2:
        import flow
        flow = flow.OpticalFlow(show_debug=True)

    try:
        while True:
            # Move the ultrasonic sensor to the middle:
            ultrasonic.pwm_S.setServoPwm('0',middleHoriz)
            ultrasonic.pwm_S.setServoPwm('1',middleVert)
            # Allow time to settle:
            time.sleep(3)

            while True:
                # Grab distance from bot to object
                c = ultrasonic.get_distance()   
                print("dist: " + str(c)) 

                if c <= d:
                    break

                # Move forward
                forward()      #Forward
                time.sleep(sleep_time_short) # Rest CPU for a bit
            print("c <= d")

            # Stop
            wheels.setMotorModel(0,0,0,0)                   #Stop
            
            # Look around, checking distances
            end = 180
            distances = []
            #angles = [0, 45, 75, 90, end - 75, end - 45, end]
            angles = [0, 30, 60, 90, end - 60, end - 30, end]
            for angle in angles: # range(start, stop, separator)
                ultrasonic.pwm_S.setServoPwm('0',angle)
                time.sleep(sleep_time_long)
                dist = ultrasonic.get_distance()
                distances.append(dist)
                print("Recorded distance " + str(dist) + " for angle " + str(angle))

            # Get the largest distance's index and value
            largest_index = max(range(len(distances)), key=distances.__getitem__)
            destination_angle = angles[largest_index]
            destination_distance = distances[largest_index]
            print("Chose distance " + str(destination_distance))

            # Move sensor back to middle
            ultrasonic.pwm_S.setServoPwm('0',middleHoriz)

            # Wait to allow the sensor to settle
            # (important to prevent reading a larger value again)
            time.sleep(0.1)

            # Prepare gyro or optical flow variables
            if stop_cond == 1 or stop_cond == 2:
                time_total = 0
                degrees_total = 90

                # ( https://stackoverflow.com/questions/1938048/high-precision-clock-in-python )
                prev_time = time.time_ns() / 1000 / 1000 / 1000  # ns to seconds
            
            # Go in that direction, reaching it once the sensor reports a distance equal to that
            # found distance (Major NOTE: this doesn't work if the shape of an obstacle is a curve
            # that keeps distance the same regardless of sensor position, but that is unlikely):
            if stop_cond == 2:
                # Optical flow: prepare
                flow.prepare()
            while True:
                # Turn
                dir = None
                if destination_angle < 90:
                    print("Turning left")
                    left(None)
                    dir = -1
                elif destination_angle == 90:
                    print("Forward but this shouldn't happen")
                    forward()
                    dir = 0
                elif destination_angle > 90:
                    print("Turning right")
                    right(None)
                    dir = 1
                else:
                    raise Exception("Unexpected case")

                if stop_cond == 0:
                    # Get current distance as we turn
                    c = ultrasonic.get_distance() 

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
                    (velX,velY,velZ) = mpu.gyro # z is left to right rotation
                    current_time = time.time_ns() / 1000 / 1000 / 1000
                    time_diff = current_time - prev_time
                    prev_time = current_time
                    degrees_total += velZ * time_diff
                    if abs(degrees_total - destination_angle) <= angle_epsilon:
                        break
                elif stop_cond == 2:
                    current_time = time.time_ns() / 1000 / 1000 / 1000

                    # Get current distance as we turn
                    c = ultrasonic.get_distance() 

                    # Optical flow
                    (closestPointToCenter, flowVector) = flow.computeCentermostFlow()
                    print("Flow reliability (lower is better): " 
                            + flow.reliabilityOfPoint(closestPointToCenter))
                    rads = flow.computeRadiansOfCameraRotation(c, flowVector)
                    degrees = math.degrees(rads)
                    print("Turned " + degrees + " degrees")
                    
                    time_diff = current_time - prev_time
                    prev_time = current_time
                    degrees_total += dir * degrees * time_diff
                    if dir == -1:
                        if degrees_total - destination_angle <= angle_epsilon:
                            break
                    elif dir == 1:
                        if destination_angle - degrees_total <= angle_epsilon:
                            break
                else:
                    raise Exception("Unimplemented stop_cond")

                    
                    # TODO: Can handle this by moving right here by a smaller amount if needed.

            # Go forward again, until we reach a wall, in which case we look around again as above.
            # TODO: However, we may have reached the end. Need to check for this somehow.

    #except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, this will be executed.
    #except Exception as e:
    finally:
        logging.error(traceback.format_exc())
        wheels.setMotorModel(0,0,0,0)
        ultrasonic.pwm_S.setServoPwm('0',middleHoriz)
        ultrasonic.pwm_S.setServoPwm('1',middleVert)
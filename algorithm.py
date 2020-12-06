import time
from time import sleep
from Motor import *
from Ultrasonic import *
from servo import *
import traceback
import logging

from enum import Enum
import sys
import math
from math import radians, degrees
import networking.client
from networking.client import *

from timeit import default_timer as timer
import functools 
import numpy as np
import threading
from threading import Thread, Lock
from intervaltree import Interval, IntervalTree # https://pypi.org/project/intervaltree/
from ADC import *
import debugUtils

# From https://stackoverflow.com/questions/1628386/normalise-orientation-between-0-and-360 :
# Normalizes any number to an arbitrary range 
# by assuming the range wraps around when going below min or above max 
def normalize(value, start, end):
    width = end - start
    offsetValue = value - start # value relative to 0
    return (offsetValue - (math.floor(offsetValue / width) * width)) + start
    # + start to reset back to start of original range
def normalizeDegrees(value):
    return normalize(value, 0, 360)


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
forwardSpeedOrig = 0.125 #0.15
# Read battery voltage of the car and up the forward speed orig as battery is detected lower and lower:
adc=Adc()
def batteryVoltage():
    Power=adc.recvADC(2)
    voltage = Power*3
    print ("The battery voltage is "+str(voltage)+"V")
    return voltage
battVoltage = batteryVoltage()
if battVoltage > 7.8: # I got around 8.07V for about a full charge.
    pass
elif battVoltage > 6: # UNTESTED
    forwardSpeedOrig += 0.02
elif battVoltage > 5: # UNTESTED
    forwardSpeedOrig += 0.02
    forwardSpeedOrig += 0.02
forwardSpeed = forwardSpeedOrig
forwardSpeedup = 0.013 #0.007
#forwardMutex = Lock()
global forwardCounter
forwardCounter = 0
forwardCounterTotal = forwardCounter
travelled = [0, 0] # 2D vector of how far we travelled and in what direction
class ForwardSpeedManagerThread(Thread):
    def __init__(self):
        ''' Constructor. '''
 
        Thread.__init__(self, daemon=True)
        self.counter = 0
        self.stop = False
 
    def run(self):
        global forwardCounter
        #while forwardSpeed-self.counter > forwardSpeedOrig and not self.stop:
        while forwardSpeed-self.counter > forwardSpeed * 0.7 and not self.stop:
            # for i in range(1, self.val):
                #print('Value %d in thread %s' % (i, self.getName()))
    
            # Sleep for random time between 1 ~ 3 second
            #secondsToSleep = randint(1, 5)
            secondsToSleep=0.02
            print('%s sleeping for %f seconds...' % (self.getName(), secondsToSleep))
            time.sleep(secondsToSleep)

            # Slow down
            self.counter += 0.002
            forward(forwardSpeed-self.counter)
            forwardCounter += 1

turnSpeedOrig = 0.21 # WORKS but super slow: 0.20 # 0.25
turnSpeed = turnSpeedOrig
if battVoltage > 7.5:
    pass
else:
    turnSpeed += 0.03
global motionThreads
motionThreads = []
def stopMotionThreads():
    global motionThreads
    if threading.current_thread() is not threading.main_thread():
        return # Main thread is the only one who stops the other threads
    if len(motionThreads) > 0:
        # Override the threads
        for t in motionThreads:
            t.stop = True
        for t in motionThreads:
            t.join()
        motionThreads = []
def forward(speed=forwardSpeed):
    global motionThreads
    if carConfig is None:
        return
    stopMotionThreads()
    #forwardMutex.acquire()
    try:
        wheels.setMotorModel(int(-4095*speed), int(-4095*speed),
                            int(-4095*speed), int(-4095*speed))
        #if speed == forwardSpeed and speed > forwardSpeedOrig and len(motionThreads) == 0:
        if len(motionThreads) == 0:
            # Spawn a thread to slow down after a bit, due to inertia, the car will continue going forward under less power.
            # Needed to increase accuracy. If we hit a wall then the sensor can report huge values for things too close.
            x = ForwardSpeedManagerThread()
            x.start()
            motionThreads.append(x)
    finally:
        #forwardMutex.release()
        pass
def stop():
    stopMotionThreads()
    wheels.setMotorModel(0, 0, 0, 0)
# Pass `None` for `secs` to just set the PWM output to that until the next time
# that the motors are set.
def left_(secs=1, speed=0.6):
    stopMotionThreads()
    wheels.setMotorModel(int(-500*speed),int(-500*speed),
                         int(2000*speed),int(2000*speed))       #Left  (<--originally)
    if secs:
        time.sleep(secs)
def right_(secs=1, speed=0.6):
    stopMotionThreads()
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
        stopMotionThreads()
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

if stop_cond == 2:
    import cv2
    import imutils

    # Source: https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/ :

    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)
    from collections import deque
    maxlen = 64
    pts = deque(maxlen=maxlen)
def checkForTennisBall(opticalFlowObj, imgRenderTarget):
    resultBallPos = None
    frame = opticalFlowObj.frame

    # Source: https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/ :

    # resize the frame, blur it, and convert it to the HSV
    # color space
    #frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            if imgRenderTarget is not None:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(imgRenderTarget, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(imgRenderTarget, center, 5, (0, 0, 255), -1)
            resultBallPos = center
    # update the points queue
    pts.appendleft(center)

    if imgRenderTarget is not None:
        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(maxlen / float(i + 1)) * 2.5)
            cv2.line(imgRenderTarget, pts[i - 1], pts[i], (0, 0, 255), thickness)
    
    return resultBallPos

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

    def endOfCourseForNavigationVehicle():
        # Write the path (as an array of serialized bytes) to a file:
        timestr = time.strftime("%Y%m%d-%H%M%S")
        with open("path_" + timestr, "wb") as f:
            outputRobotPath(array.array('B', recordedPath), f)
        # Connect to the server and send the path
        client = Client()
        client.sendPath(recordedPath)

    # How much we have turned since the start of the course, relative to 0 degrees being ahead
    degrees_entire = 0
    # What we currently consider to be the backwards direction. Starts at 180 since that is behind the car at the
    # start of the course.
    degrees_backwards_direction = 180
    # Distances for angles we have seen before that are absolute, i.e. fixed and not relative to the car's
    # current rotation.
    distancesAtAbsoluteAngles = IntervalTree() # Given an absolute angle range, this gives the largest distance seen.
    # ^^ Overall idea is: reorient the backwards direction when an absolute direction becomes larger than it was
    # *before* it was measured, indicating a turn.

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

            # Move forward
            # [New]
            speed = forwardSpeed
            forward(speed)
            start_time = timer()
            distances = []
            c = None
            while True:
                offset = 40 #32
                movement = 15
                distances_prev = distances
                distances = []
                global adjustedCount
                adjustedCount = 0
                def scan(dir): # Returns whether to continue or not.
                    global adjustedCount
                    start = (middleHoriz - offset) if dir==1 else (middleHoriz + offset)
                    end = (middleHoriz + offset) if dir==1 else (middleHoriz - offset)
                    inc = movement if dir==1 else -movement
                    for angle in range(start, end, inc):
                        ultrasonic.pwm_S.setServoPwm('0', angle)
                        c = getUltrasonicDistance()  # Grab distance from bot to object
                        if abs(angle-middleHoriz) > 10 and c < 30:
                            print("Adjusted")
                            adjustedCount += 1
                            adj = abs(2*c) # Angles further off center are less important if they are close
                        else:
                            adj = c
                        print("dist: " + str(c), "adjusted:", adj)
                        if adj <= d:
                            if abs(angle - middleHoriz) < 12:
                                print("Not going straight because it is blocked")
                                continue # Never want to go straight if it is blocked
                            return (c, False) # Exit
                        distances.append(c)
                        time.sleep(0.11)
                    return (None, True)
                c, shouldContinue = scan(1)
                if not shouldContinue:
                    break
                c, shouldContinue = scan(-1)
                if not shouldContinue:
                    break
                if adjustedCount > 5: # Stop because we're probably hitting a wall now
                    print("Probably hitting a wall, stopping")
                    break
                # Check if all distances were the same across the arrays by a threshold
                if len(distances) == len(distances_prev):
                    if functools.reduce(lambda a,b: a and b, np.isclose(distances, distances_prev, 0.5)):
                        # Speed up
                        print("Speeding up")
                        speed += forwardSpeedup
                        forward(speed)
                        forwardSpeed = (forwardSpeed + speed) / 2 # Approach the new speed here
            print("c <= d")
            end_time = timer()
            print("Time going forward: " + str(end_time - start_time))
            if not isReceivingVehicle:
                recordedDurations.append((end_time - start_time, speed))
            #time.sleep(sleep_time_short) # Rest CPU for a bit

            # Stop
            stop()
            
            if not isReceivingVehicle: # Then we are the navigation vehicle. We are forging 
                # a new path!
                # Look around, checking distances
                end = 180
                distances = []
                angles = []
                #angles = [0, 45, 75, 90, end - 75, end - 45, end]
                #[Old] angles = [0, 30, 60, 90, end - 60, end - 30, end]
                destination_distance = None
                destination_angle = None
                largest_index = None
                i = 0
                for angle in range(middleHoriz - 90, middleHoriz + 90 + 10, 10): # + 10 bceause last value is excluded
                    #for angle in angles: # range(start, stop, separator)
                    ultrasonic.pwm_S.setServoPwm('0',angle)
                    time.sleep(sleep_time_long)
                    dist = getUltrasonicDistance()

                    absoluteAngle = degrees_entire + (angle - middleHoriz) # What we consider to be the 
                    # angle in a fixed coordinate system (i.e., one that doesn't rotate with the car).
                    

                    # Check whether we have seen this absoluteAngle before, up to some threshold, and if so,
                    # then we want to adjust our `degrees_backwards_direction` so that it makes what is considered
                    # "turning around"* to instead be behind us currently
                    threshold = 10
                    absoluteAngle_normalized = normalizeDegrees(absoluteAngle)
                    minA = normalizeDegrees(absoluteAngle_normalized - threshold)
                    maxA = normalizeDegrees(absoluteAngle_normalized + threshold)
                    if maxA < minA:
                        # We can't have this stored in the interval tree, so we do a workaround:
                        maxA = maxA + 360 # Make it wrap once (do we need to undo this later?)
                    distancesA = distancesAtAbsoluteAngles[minA:maxA]
                    if len(distancesA) > 0: # `distancesA` is an array of `Interval`s, each of 
                        # which has a `begin`, `end`, and `data` (which holds the distance in this case)

                        # Check if our distance is larger by another threshold
                        larger = False # Assume False.
                        for dInterval in distancesA:
                            if dist > dInterval.data + 8:
                                # Larger, so we need to try using this
                                larger = True
                                break
                        if larger:
                            # Reorient the backwards direction to be behind us
                            newBackwardsOrientation = normalizeDegrees(absoluteAngle + 180.0)
                            print("Reorienting backwards direction to", newBackwardsOrientation, 
                                    "from", degrees_backwards_direction)
                            degrees_backwards_direction = newBackwardsOrientation
                    # Record it
                    print("Adding interval", str(minA) + ":" 
                            + str(maxA), "with distance", dist)
                    distancesAtAbsoluteAngles[minA:maxA] = dist
                    

                    if destination_distance is None or dist > destination_distance: # Record a new largest distance
                        # 90 works but we need to be a little more picky because sometimes it's ok, in order to complete the semicircle. So we will
                        # only activate this if the distance isn't big (second part after the first `if`)

                        #if abs(absoluteAngle) > (degrees_backwards_direction - 180) + 90: # *Then we would turn around, don't!
                        if abs(absoluteAngle_normalized - (degrees_backwards_direction - 180) < 180): # *Then we would turn around, don't!
                            print("Potential turnaround, distance", dist, "for angle", angle)
                            #if dist < 50:
                            #res = input("Avoid turnaround (y/n)? ")
                            if True: #if res == "y":
                                print("Avoided turnaround")
                                distances.append(0)
                                angles.append(angle)
                                i += 1
                                continue
                        destination_distance = dist
                        destination_angle = angle
                        largest_index = i
                    distances.append(dist)
                    angles.append(angle)
                    print("Recorded distance " + str(dist) + " for angle " + str(angle))
                    i += 1
                # Check if we have "Forward but shouldn't happen"
                if destination_angle == 90:
                    # Now we need to see what other distances are good
                    print("Prevented going forward. Choosing next largest angle")
                    i = 0
                    largest = None
                    for dist in distances:
                        if i == largest_index:
                            i += 1
                            continue
                        if largest is None or dist > largest:
                            largest = dist
                            destination_angle = angles[i]
                            largest_index2 = i
                        i += 1
                    destination_distance = largest
                    largest_index = largest_index2
                    
                print("Chose angle " + str(destination_angle) + " with distance " + str(destination_distance))

                # Save this angle
                recordedPath.append(destination_angle)

                if False:
                    # Check for end condition: all polled distance values are less 
                    # than or equal to `d`:
                    if destination_distance <= d:
                        isEnd=input("End reached (y/n)? ")
                        if not isEnd or isEnd.lower().strip() == "n":
                            pass
                        else:
                            endOfCourseForNavigationVehicle()
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
                degrees_total = middleHoriz
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
            if destination_angle < 90:
                print("Turning left")
                #left(None)
                speed = -turnSpeed
                turn(speed)
                dir = -1
            elif destination_angle == 90:
                print("Forward but this shouldn't happen")
                speed = forwardSpeed
                #forward()
                # This actually happened once, let's turn to fix it:
                turn(turnSpeed)
                time.sleep(0.1)
                stop()
                continue # Let's try this again
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
                elif stop_cond == 2:
                    #current_time = time.time_ns() / 1000 / 1000 / 1000

                    # Get current distance as we turn
                    #c = getUltrasonicDistance() 

                    # Optical flow, [nvm, doesn't work: also look for end condition: seeing a tennis ball (via the `renderHook`)]
                    #(closestPointToCenter, flowVector) = flow.computeCentermostFlow(renderHook=checkForTennisBall)
                    (closestPointToCenter, flowVector) = flow.computeCentermostFlow(renderHook=None)
                    #print("Flow reliability (closer to 0 is better): " 
                    #        + str(flow.reliabilityOfPoint(closestPointToCenter)))
                    
                    # Check for tennis ball position [nvm:, and if there is any then move towards it]
                    if flow.renderHookRes is not None:
                        #print("Tennis ball (end of course) found!")
                        #endOfCourseForNavigationVehicle()
                        #break
                        pass
                    
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

                if stop_cond == 1 or stop_cond == 2:
                    if abs(degrees_total - degrees_total_prev) < 0.1:
                        noMovementCounter += 1
                        print("No movement")
                        if noMovementCounter > 15:
                            # Bump up how much we turn
                            speed += np.sign(speed)*0.01
                            turn(speed)
                            noMovementCounter = 0
                            print("Increasing turn speed")
                            turnSpeed = (turnSpeedOrig + abs(speed)) / 2 # Approach the new speed here
            
            # Track how much we turned overall since the start of the course.
            if stop_cond == 1 or stop_cond == 2:
                travelled[0] += forwardCounter*math.cos(radians(degrees_entire))
                travelled[1] += forwardCounter*math.sin(radians(degrees_entire))
                print("Travelled:", travelled)
                degrees_entire += degrees_total - 90
                print("degrees_entire:", degrees_entire)
                forwardCounterTotal += forwardCounter
                print("forwardCounterTotal:", forwardCounterTotal)
                forwardCounter = 0
                threshold = forwardCounterTotal / 2
                if False: #if abs(travelled[0]) > threshold and abs(travelled[1]) > threshold:
                    print("Travelled diagonally, resetting degrees_entire to 0")
                    #if forwardCounter > 20 and abs(degrees_entire) > 0:
                    degrees_entire = 0 # Reset because once we go far enough in a diagonal direction since we actually 
                    # do want to turn around to complete a semicircular course. Otherwise once it gets far enough, then
                    # going in the direction of the end of the course will be considered "Avoided turnaround" which we
                    # don't want.
                    travelled[0] = 0
                    travelled[1] = 0
            
            end_time = timer()
            print("Time turning: " + str(end_time - start_time))
            if not isReceivingVehicle:
                recordedDurations.append((end_time - start_time, speed))
            # Go forward again, until we reach a wall, in which case we look around again as above.

    #except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, this will be executed.
    #except Exception as e:
    finally:
        logging.error(traceback.format_exc())
        stop()
        ultrasonic.pwm_S.setServoPwm('0',middleHoriz)
        ultrasonic.pwm_S.setServoPwm('1',middleVert)

        # Enter REPL real quick; useful to inspect the last state of the car.
        print("Entering REPL; press Ctrl-C to exit")
        debugUtils.enterREPL(globals(), locals())

        if client:
            client.close()
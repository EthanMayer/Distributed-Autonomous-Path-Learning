import time
from Motor import *
from Ultrasonic import *
from servo import *
import traceback
import logging

# Configuration
d = 20 # Centimeters from car to object at which to stop and scan from
halfW = 8 # Half the width of the car in centimeters.
# TODO: use this value to prevent the car from running into the wall on an edge
middleHoriz = 50 # 60 # The center for the servo motor for the ultrasonic sensor, horizontally
middleVert = 25 # The center for the ultrasonic sensor vertically.
sleep_time_short = 0.01 #0.1
sleep_time_long = 0.4 #0.2

# Utilities
ultrasonic=Ultrasonic()
ultrasonic.pwm_S = Servo()  
wheels=Motor()

def forward():
    # wheels.setMotorModel(2000,2000,2000,2000)       #Forward (<--originally)
    wheels.setMotorModel(-2000,-2000,-2000,-2000)       #Forward
# Pass `None` for `secs` to just set the PWM output to that until the next time
# that the motors are set.
def left(secs=1):
    wheels.setMotorModel(2000,2000,-500,-500)       #Right (<--originally)  
    if secs:
        time.sleep(secs)
def right(secs=1):
    wheels.setMotorModel(-500,-500,2000,2000)       #Left  (<--originally)
    if secs:
        time.sleep(secs)

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        while True:
            # Move the ultrasonic sensor to the middle:
            ultrasonic.pwm_S.setServoPwm('0',middleHoriz)
            ultrasonic.pwm_S.setServoPwm('1',middleVert)

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
            destination_distance = distances[largest_index]
            print("Chose distance " + str(destination_distance))

            # Move sensor back to middle
            ultrasonic.pwm_S.setServoPwm('0',middleHoriz)

            # Go in that direction, reaching it once the sensor reports a distance equal to that
            # found distance (Major NOTE: this doesn't work if the shape of an obstacle is a curve
            # that keeps distance the same regardless of sensor position, but that is unlikely):
            while True:
                # Turn
                if angles[largest_index] < 90:
                    print("Turning left")
                    left(None)
                elif angles[largest_index] == 90:
                    print("Forward but this shouldn't happen")
                    forward()
                elif angles[largest_index] > 90:
                    print("Turning right")
                    right(None)
                else:
                    raise Exception("Unexpected case")

                # Get current distance as we turn
                c = ultrasonic.get_distance() 

                epsilon = 30 #1
                if abs(c - destination_distance) <= epsilon:
                    break
                # elif destination_distance > c:
                    # We overshot
                #    print("Overshot by " + str(destination_distance - c))
                #    break
                    
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
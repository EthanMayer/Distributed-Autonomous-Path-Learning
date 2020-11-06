import time
from Motor import *
from Ultrasonic import *
from servo import *

# Configuration
d = 10 # Centimeters from car to object at which to stop and scan from
halfW = 10 # Half the width of the car in centimeters. # TODO: current value is only a guess, replace it

# Utilities
ultrasonic=Ultrasonic()
ultrasonic.pwm_S = Servo()  
wheels=Motor()

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        while True:
            # Move the ultrasonic sensor to the middle:
            ultrasonic.pwm_S.setServoPwm('0',90)

            while True:
                # Grab distance from bot to object
                c = ultrasonic.get_distance()    

                # Move forward
                wheels.setMotorModel(2000,2000,2000,2000)       #Forward
                time.sleep(0.1) # Rest CPU for a bit

                if c >= d:
                    break

            # Stop
            wheels.setMotorModel(0,0,0,0)                   #Stop
            
            # Look around, checking distances
            end = 180
            distances = []
            angles = [45, 75, 90, end - 75, end - 45]
            for angle in angles: # range(start, stop, separator)
                ultrasonic.pwm_S.setServoPwm('0',angle)
                time.sleep(0.2)
                distances.append(ultrasonic.get_distance())

            # Get the smallest distance's index and value
            smallest_index = min(range(len(distances)), key=distances.__getitem__)
            destination_distance = distances[smallest_index]

            # Move sensor back to middle
            ultrasonic.pwm_S.setServoPwm('0',90)

            # Go in that direction, reaching it once the sensor reports a distance equal to that
            # found distance (Major NOTE: this doesn't work if the shape of an obstacle is a curve
            # that keeps distance the same regardless of sensor position, but that is unlikely):
            while True:
                # Turn
                if angles[smallest_index] < 90:
                    wheels.setMotorModel(-500,-500,2000,2000)       #Left 
                elif angles[smallest_index] == 90:
                    print("Forward but this shouldn't happen")
                    wheels.setMotorModel(2000,2000,2000,2000)       #Forward but shouldn't happen
                elif angles[smallest_index] > 90:
                    wheels.setMotorModel(-500,-500,2000,2000)       #Right 
                
                # Get current distance as we turn
                c = ultrasonic.get_distance() 

                epsilon = 1
                if abs(c - destination_distance) <= epsilon:
                    break
                elif destination_distance > c:
                    # We overshot
                    print("Overshot by " + destination_distance - c)
                    
                    # TODO: Can handle this by moving right here by a smaller amount if needed.

            # Go forward again, until we reach a wall, in which case we look around again as above.
            # TODO: However, we may have reached the end. Need to check for this somehow.

    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, this will be executed.
        wheels.setMotorModel(0,0,0,0)
        ultrasonic.pwm_S.setServoPwm('0',90)
from servo import *

pwm=Servo()
print("""Enter an amount of degrees to turn to. 
Put v at the end to test vertical movement (for example, 50v).\n""")
while True:
    degreesStr = input("> ")
    if degreesStr.endswith("v"):
        degrees = int(degreesStr[:-1])
        pwm.setServoPwm('1', degrees)
    else:
        degrees = int(degreesStr)
        pwm.setServoPwm('0', degrees)

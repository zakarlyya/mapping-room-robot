#
# sensor.py
#
# Launched by logic thread
# Pan the sensor back and forth using servos (while measuring with sensor)
# Sends points to data processing thread with the detected points or if turn is necessary

import zmq
import time
import math
from Ultrasonic import *
from servo import *

def sensor_main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")

    pwm=Servo()
    ultrasonic=Ultrasonic()   
    pwm.setServoPwm('0', 90)
    pwm.setServoPwm('1', 92)
    angle = 0
    increasing = True

    num_trials = 10

    while True:
        distance = 0
        for i in range(num_trials):
            distance = distance + ultrasonic.get_distance()/num_trials

        socket.send_string("{}, {}".format(angle, distance))
        
        if increasing:
            angle = angle + 5
        else:
            angle = angle - 5

        if angle == 90:
            increasing = False
        elif angle == -90:
            increasing = True
        
        pwm.setServoPwm('0', 90 - angle)
        
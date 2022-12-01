#
# sensor.py
#
# Launched by logic thread
# Pan the sensor back and forth using servos (while measuring with sensor)
# Sends points to data processing thread with the detected points or if turn is necessary

# import necessary libraries
import zmq
import time
from Ultrasonic import *
from servo import *

def sensor_main():
    # create ZMQ context and PUB socket
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")

    # create an instance of servo and ultrasonic classes
    pwm=Servo()
    ultrasonic=Ultrasonic()  

    # point robot straight ahead 
    pwm.setServoPwm('0', 90)
    pwm.setServoPwm('1', 92)

    # set default values including sensor angle and number of readings per angle
    angle = -90
    num_trials = 2 

    while True:
        # get the average distance from the sensor using multiple readings to reduce noise
        distance = 0
        for i in range(num_trials):
            time.sleep(0.05)
            distance += ultrasonic.get_distance()/num_trials

        # send the average distance measured for some angle
        socket.send_string("{}, {}".format(angle, distance))
        
        # update the angle iteratively
        angle += 2

        # update increasing value to pan in other direction
        if angle > 90:
            angle = -90
        
        # set the robot's head pan angle
        pwm.setServoPwm('0', 90 - angle)
        
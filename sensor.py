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
    num_trials = 3

    while True:
        # array which stores distances
        distances = []

        for i in range(num_trials):
            # get the distance from the sensor using multiple readings to reduce noise
            distances.append(ultrasonic.get_distance())
        
        # remove outliers over 5 cm from the median distance
        median = sorted(distances)[len(distances)//2]
        distances = [x for x in distances if x < median + 5 and x > median - 5]

        # get the average distance
        distance = sum(distances)/len(distances)

        # send the average distance measured for some angle
        socket.send_string("{}, {}".format(angle, distance))
        
        # update the angle iteratively
        angle += 2

        # update increasing value to pan in other direction
        if angle > 20:
            angle = -90
        
        # set the robot's head pan angle
        pwm.setServoPwm('0', 90 - angle)
        if angle == 90:
            time.sleep(0.2)
        
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

context = zmq.Context()
laptop_ip_address = "192.168.1.115"

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://{}:5555".format(laptop_ip_address))

pwm=Servo()
ultrasonic=Ultrasonic()   
pwm.setServoPwm('0', 90)
pwm.setServoPwm('1', 92)

def convert_to_position(reading, angle):
    return (reading * math.cos(math.radians(angle)), reading * math.sin(math.radians(angle)))

def pan_ultrasonic():
    angle = 0
    increasing = True

    try:
        while True:
            sum = 0
            for i in range(20):
                sum = sum + ultrasonic.get_distance()

            data = sum/20
            point = convert_to_position(data, angle)

            temp = "Object detected " + str(data) + " cm away at" + str(point) + " at angle of " + str(angle)
            print (temp)
            socket.send_string(temp)
            
            if increasing:
                angle = angle + 3
            else:
                angle = angle - 3

            if angle == 45:
                increasing = False
            elif angle == -45:
                increasing = True
            
            pwm.setServoPwm('0', 90 - angle)

            message = socket.recv_string()
            print("Received reply: " + message)


    except KeyboardInterrupt:
        print ("\nEnd of program")
        return

while True:
    pan_ultrasonic()
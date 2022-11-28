#
# motors.py
# 
# Waits for motor movement request from logic.py
# Executes motor movement (move forward, turn left, turn right... etc)
# Transmit reply indicating movement

import zmq
import time
import logging
import threading

# import motor class from motor_class.py
from motor_class import Motor

def motors_main():
    # create a zmq context and socket for REQ/REP
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")

    # create an instance of the Motor class
    motor = Motor()

    # wait for a request from logic.py
    while True:
        message = socket.recv()
        logging.info("Received motor request: %s" % message)

        # parse message as a character followed by a number
        # character indicates direction of movement
        # floating point number indicates distance of movement
        direction = chr(message[0])
        distance = float(message[1:])
        logging.info("Direction: %s, Distance: %s" % (direction, distance))

        # if the message is move backward, move the robot backward
        if direction == 'B':
            motor.setMotorModel(500,500,500,500)             # Move backward 
            time.sleep(distance)
            motor.setMotorModel(0,0,0,0)                         # Stop
            socket.send(b"Moved backward")
        # if the message is turn right, turn the robot right
        elif direction == 'R':
            motor.setMotorModel(500,500,1500,1500)             # Turn right
            time.sleep(distance)
            motor.setMotorModel(0,0,0,0)
            socket.send(b"Turned right")
        # if the message is turn left, turn the robot left
        elif direction == 'L':
            motor.setMotorModel(1500,1500,500,500)             # Turn left
            time.sleep(distance)
            motor.setMotorModel(0,0,0,0)
            socket.send(b"Turned left")
        # if the message is move forward, move the robot forward
        elif direction == 'F':
            motor.setMotorModel(-500,-500,-500,-500)            # Move forward
            time.sleep(distance)
            motor.setMotorModel(0,0,0,0)
            socket.send(b"Moved forward")
        # if the message is unknown, send an error message
        else:
            motor.setMotorModel(0,0,0,0)
            socket.send(b"Unknown command")

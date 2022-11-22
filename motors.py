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
    # createa zmq context and socket for REQ/REP
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")

    # create an instance of the Motor class
    motor = Motor()

    # wait for a request from logic.py
    while True:
        message = socket.recv()
        logging.info("Received motor request: %s" % message)

        # if the message is move forward, move the robot forward
        if message == b"F":
            motor.setMotorModel(2000,2000,2000,2000)             # Move Forward 
            time.sleep(3)
            motor.setMotorModel(0,0,0,0)                         # Stop
            socket.send(b"Moved forward")
        # if the message is turn left, turn the robot left
        elif message == b"L":
            motor.setMotorModel(-500,-500,2000,2000)             # Turn Left
            time.sleep(3)
            motor.setMotorModel(0,0,0,0)
            socket.send(b"Turned left")
        # if the message is turn right, turn the robot right
        elif message == b"R":
            motor.setMotorModel(2000,2000,-500,-500)             # Turn Right
            time.sleep(3)
            motor.setMotorModel(0,0,0,0)
            socket.send(b"Turned right")
        # if the message is move backward, move the robot backward
        elif message == b"B":
            motor.setMotorModel(-2000,-2000,-2000,-2000)
            time.sleep(3)
            motor.setMotorModel(0,0,0,0)
            socket.send(b"Moved backward")
        # if the message is unknown, send an error message
        else:
            motor.setMotorModel(0,0,0,0)
            socket.send(b"Unknown command")

#
# logic.py
#
# Retrieves sensor data. While conducing "wall following" mode, it first 
# calculates the coordinate location all new points (if any). 

# If any points are in front of the robot within one robot's length and 
# to the right, it turns left. If no object in front or right, it moves 
# forward and turns right until it detects an object on the right.
#
# Transmits motor movements as requests to motors
# Waits for motor reply
# Requests sensor data after motor movement
# Wait for sensor data
# Calculate next move
# Repeat
#
# Terminate when robot comes back to 0,0

import enum
import logging
import threading
import time
import queue
import zmq

# Import motor class from motor_class.py
from motor_class import Motor

directions = enum('NORTH', 'EAST', 'SOUTH', 'WEST')

def logic_main():
    
    # Initialize variables
    current_pos = [0,0]
    current_direction = directions.NORTH

    # create instance of Robot class
    robot = Robot(pos=current_pos, direction=current_direction)

    # Create a zmq context and socket for REQ/REP with start stop signal from main
    context = zmq.Context()
    start_socket = context.socket(zmq.REP)
    start_socket.bind("tcp://*:5557")

    # wait for the start signal on start socket
    while True:
        message = start_socket.recv()
        if message == b"START":
            start_socket.send(b"Starting")
            logging.info("Received start signal from main")
            break
        else:
            logging.ERROR("Received unknown start signal from main: %s" % message)

    # retrieve first sensor measurements
    robot.getSensorData()
    

class Robot:
    def __init__(self, pos, dir):
        self.pos = pos
        self.dir = dir

        # create a list for all sensor data points
        self.sensor_data = []

        motor = Motor()

        # create a zmq REQ/REP motor_socket to communicate with the motors
        context = zmq.Context()
        self.motor_socket = context.socket(zmq.REQ)
        self.motor_socket.connect("tcp://localhost:5555")

        # create a separate zmq PUB/SUB to communicate with the sensor
        self.sensor_socket = context.socket(zmq.SUB)
        self.sensor_socket.connect("tcp://localhost:5556")
        self.sensor_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # Distance is specified in inches
    def moveForward(self, distance):
        
        # round distance to nearest integer
        distance = round(distance)

        # transmit a message the motors via zmq socket the number distance times and wait for reply
        for i in range(distance):
            self.motor_socket.send(b"F")
            message = self.motor_socket.recv()
            logging.info("Received reply from motors %s" % message)

        # if the robot is facing north, update the y coordinate of the robot position
        if self.dir == directions.NORTH:
            self.pos[1] = self.pos[1] + distance
        # if the robot is facing east, update the x coordinate of the robot position
        elif self.dir == directions.EAST:
            self.pos[0] = self.pos[0] + distance
        # if the robot is facing south, update the y coordinate of the robot position
        elif self.dir == directions.SOUTH:
            self.pos[1] = self.pos[1] - distance
        # if the robot is facing west, update the x coordinate of the robot position
        elif self.dir == directions.WEST:
            self.pos[0] = self.pos[0] - distance


    def turnLeft(self):

        # transmit a message to the motors via zmq socket to turn left and wait for reply
        self.motor_socket.send(b"L")
        message = self.motor_socket.recv()
        logging.info("Received reply to turn from motors %s" % message)

        if self.dir == directions.NORTH:
            self.dir = directions.WEST
        elif self.dir == directions.WEST:
            self.dir = directions.SOUTH
        elif self.dir == directions.SOUTH:
            self.dir = directions.EAST
        elif self.dir == directions.EAST:
            self.dir = directions.NORTH

    def turnRight(self):

        # transmit a message to the motors via zmq socket to turn right and wait for reply
        self.motor_socket.send(b"R")
        message = self.motor_socket.recv()
        logging.info("Received reply to turn from motors %s" % message)

        if self.dir == directions.NORTH:
            self.dir = directions.EAST
        elif self.dir == directions.EAST:
            self.dir = directions.SOUTH
        elif self.dir == directions.SOUTH:
            self.dir = directions.WEST
        elif self.dir == directions.WEST:
            self.dir = directions.NORTH

    def getSensorData(self):
        self.sensor_socket.send(b"Q")
        message = self.sensor_socket.recv()
        return message

    def storeSensorData(self, data):
        self.sensor_data.append(data)

    def getPos(self):
        return self.pos
    
    def getDir(self):
        return self.dir
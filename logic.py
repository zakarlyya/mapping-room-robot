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

import logging
import threading
import time
import queue
import zmq

from enum import Enum

# Import motor class from motor_class.py
from motor_class import Motor

# define the values NORTH, EAST, SOUTH, WEST as 0, 1, 2, 3
class Direction(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3



def logic_main():
    
    # Initialize variables
    current_pos = [0,0]
    
    # use direction enum to set current_direction to NROTH
    current_direction = Direction.NORTH

    # create a data structure which stores the coordinates of all points visited by the robot
    positions = []  # list of lists

    # Create a zmq context and socket for REQ/REP with start stop signal from main
    context = zmq.Context()
    start_socket = context.socket(zmq.REP)
    start_socket.bind("tcp://*:5557")

    # create a zmq REQ/REP motor_socket to communicate with the motors
    context = zmq.Context()
    motor_socket = context.socket(zmq.REQ)
    motor_socket.connect("tcp://localhost:5555")

    # create a zmq PUB/SUB sensor_socket to communicate with the sensors
    sensor_socket = context.socket(zmq.SUB)
    sensor_socket.connect("tcp://localhost:5556")
    sensor_socket.setsockopt(zmq.SUBSCRIBE, b"")

    # create instance of Robot class
    robot = Robot(pos=current_pos, direction=current_direction, motor_socket=motor_socket)

    # wait for the start signal on start socket
    while True:
        message = start_socket.recv()
        if message == b"START":
            start_socket.send(b"Starting")
            logging.info("Received start signal from main")
            break
        else:
            logging.ERROR("Received unknown start signal from main: %s" % message)

    # poll the start socket for a stop signal, also poll the motor socket for a reply, also poll the sensor socket for sensor data
    poller = zmq.Poller()
    poller.register(start_socket, zmq.POLLIN)
    poller.register(motor_socket, zmq.POLLIN)
    poller.register(sensor_socket, zmq.POLLIN)
    socks = dict(poller.poll())

    # create a mapping_done flag
    mapping_done = False
    # Create a ready_to_move flag
    ready_to_move = True
    robot.moveForward(0.1)

    # until the robot receieves a STOP signal from start socket or mapping_done flag is set, loop
    while not mapping_done:

        current_readings = [] # list of sensor readings

        # mapping_done if robot is within 3 units of 0,0
        #if robot.pos[0] < 3 and robot.pos[1] < 3:
        #    mapping_done = True
        #    logging.info("Mapping done")
        #    break

        # if the start socket has a stop signal, mapping_done is true
        if start_socket in socks and socks[start_socket] == zmq.POLLIN:
            message = start_socket.recv()
            if message == b"STOP":
                mapping_done = True
                logging.info("Received stop signal from main")
                break
            else:
                logging.ERROR("Received unknown stop signal from main: %s" % message)
        elif motor_socket in socks and socks[motor_socket] == zmq.POLLIN:
            # if the motor socket has a reply, update positional data
            motor_movement = motor_socket.recv()
            ready_to_move = True
            logging.info("Received motor reply: %s" % motor_movement)

        elif sensor_socket in socks and socks[sensor_socket] == zmq.POLLIN:
            
            sensor_data = sensor_socket.recv()

            logging.info("Received sensor data: %s" % sensor_data)

            # parse the sensor data as [angle], [distance]
            sensor_data = sensor_data.split(",")
            current_readings.append(sensor_data)

            angle = sensor_data[0]
            distance = sensor_data[1]

            # calculate the absolute position of the measured object using the robots current position, 
            # measured angle, and measured distance and then add the location to the positions list

        # if the robot is ready to move, move it
        if ready_to_move:
            
            # In an effort to remove erroneous data points, each sensor reading is used as a "vote" for the next move
            vote_forward = 0
            vote_left = 0
            vote_right = 0
            isData = 0
            # iterate through all the data in the current sensor data list
            for data in current_readings:
                isData = 1
                # if a measurement is made at > 110º we know we are not turning right
                if data[0] > 110 and data[0]:
                    vote_right -= 1
                if 70 < data[0] < 110:
                    vote_forward -= 1
                if data[0] < 70:
                    vote_left -= 1
                
            # if there is no data, move forward
            if isData == 0:
                logging.error("No data retrieved from sensors... Waiting for next clock cycle to make a move")
                continue

            # check the votes and move the robot accordingly
            if vote_forward > vote_left and vote_forward > vote_right:
                robot.moveForward(0.1)
            elif vote_left > vote_forward and vote_left > vote_right:
                robot.turnLeft()
            elif vote_right > vote_forward and vote_right > vote_left:
                robot.turnRight()
            else:
                logging.error("No clear vote for next move, robot will move forward slightly")
                robot.moveForward(0.1)

            # if there is an object in front of the robot within one robot's length and to the right, turn left

            # if there is no object in front or right, move forward and turn right until it detects an object on the right

            ready_to_move = False
            



class Robot:
    def __init__(self, pos, dir, motorsocket):
        self.pos = pos
        self.dir = dir

        motor = Motor()

    # Distance is specified in inches
    def moveForward(self, distance):
        
        # round distance to nearest integer
        distance = round(distance)

        # transmit a message the motors via zmq socket as F[distance] as a string and wait for reply
        self.motor_socket.send(b"F" + str(distance).encode())
        message = self.motor_socket.recv()
        logging.info("Received reply to move from motors %s" % message)
        
        # if the robot is facing north, update the y coordinate of the robot position
        if self.dir == Direction.NORTH:
            self.pos[1] = self.pos[1] + distance
        # if the robot is facing east, update the x coordinate of the robot position
        elif self.dir == Direction.EAST:
            self.pos[0] = self.pos[0] + distance
        # if the robot is facing south, update the y coordinate of the robot position
        elif self.dir == Direction.SOUTH:
            self.pos[1] = self.pos[1] - distance
        # if the robot is facing west, update the x coordinate of the robot position
        elif self.dir == Direction.WEST:
            self.pos[0] = self.pos[0] - distance


    def turnLeft(self):

        # transmit a message to the motors via zmq socket to turn left and wait for reply
        self.motor_socket.send(b"L1")
        message = self.motor_socket.recv()
        logging.info("Received reply to turn from motors %s" % message)

        if self.dir == Direction.NORTH:
            self.dir = Direction.WEST
        elif self.dir == Direction.WEST:
            self.dir = Direction.SOUTH
        elif self.dir == Direction.SOUTH:
            self.dir = Direction.EAST
        elif self.dir == Direction.EAST:
            self.dir = Direction.NORTH

    def turnRight(self):

        # transmit a message to the motors via zmq socket to turn right and wait for reply
        self.motor_socket.send(b"R1")
        message = self.motor_socket.recv()
        logging.info("Received reply to turn from motors %s" % message)

        if self.dir == Direction.NORTH:
            self.dir = Direction.EAST
        elif self.dir == Direction.EAST:
            self.dir = Direction.SOUTH
        elif self.dir == Direction.SOUTH:
            self.dir = Direction.WEST
        elif self.dir == Direction.WEST:
            self.dir = Direction.NORTH

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
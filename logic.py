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
import math
import threading
import time
import queue
import zmq

from Ultrasonic import *
from servo import *
from sensor import sensor_main

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
    current_pos = [0, 0]
    
    # use direction enum to set current_direction to NROTH
    current_direction = Direction.EAST

    # create a data structure which stores the coordinates of all points visited by the robot
    points = []  # list of lists

    # Create a zmq context
    context = zmq.Context()

    # create a zmq REQ/REP socket for start or stop signal from main
    start_socket = context.socket(zmq.REP)
    start_socket.bind("tcp://*:5557")

    # wait for the start signal on start socket
    while True:
        message = start_socket.recv()
        if message == b"START":
            logging.info("Received start signal from main")
            break
        else:
            logging.ERROR("Received unknown start signal from main: %s" % message)

    # create a zmq REQ/REP motor_socket to communicate with the motors
    motor_socket = context.socket(zmq.REQ)
    motor_socket.connect("tcp://localhost:5555")
    
    # create instance of Robot class
    robot = Robot(pos=current_pos, dir=current_direction, motor_socket=motor_socket)
    vel = robot.calibrateMotors()

    logging.info("Calibrated motors. Calculated velocity: %s" % vel)

    # Wait for go from main
    start_socket.send(b"GO")
    message = start_socket.recv()
    if message == b"GO":
        start_socket.send(b"Ack")
        logging.info("Received go signal from main")

    # FIXME move forward until robot is certain distance away from wall

    robot.turnLeft()

    sensor_thread = threading.Thread(target=sensor_main)
    sensor_thread.start()

    # create a zmq PUB/SUB sensor_socket to communicate with the sensors
    sensor_socket = context.socket(zmq.SUB)
    sensor_socket.connect("tcp://localhost:5556")
    sensor_socket.setsockopt(zmq.SUBSCRIBE, b"")

    # create a zmq PUB/SUB communications_socket to communicate with the server
    server_socket = context.socket(zmq.PUB)
    server_socket.bind("tcp://*:5558")

    # poll the start socket for a stop signal, also poll the motor socket for a reply, also poll the sensor socket for sensor data
    poller = zmq.Poller()
    poller.register(start_socket, zmq.POLLIN)
    poller.register(sensor_socket, zmq.POLLIN)
    poller.register(motor_socket, zmq.POLLIN)

    # create a mapping_done flag
    mapping_done = False
    # Create a ready_to_move flag
    ready_to_move = True
    # Set net number of turns to track where in room
    net_num_left_turns = 0

    dist_in_front = 100

    # Log that we are beginning mapping
    logging.info("Beginning mapping")

    current_readings = []           # list of sensor readings

    # until the robot receieves a STOP signal from start socket or mapping_done flag is set, loop
    while not mapping_done:
        socks = dict(poller.poll())

        # mapping_done if robot net_num_left_turns is 4
        if net_num_left_turns == 4:
            mapping_done = True
            logging.info("Mapping complete")

        # if the start socket has a stop signal, mapping_done is true
        if start_socket in socks and socks[start_socket] == zmq.POLLIN:
            message = start_socket.recv()
            if message == b"STOP":
                mapping_done = True
                logging.info("Received stop signal from main")
                break
            else:
                logging.ERROR("Received unknown stop signal from main: %s" % message)

        if sensor_socket in socks and socks[sensor_socket] == zmq.POLLIN:
            sensor_data = sensor_socket.recv_string()

            # logging.info("Received sensor data: %s" % sensor_data)

            # parse the sensor data as [angle], [distance]
            sensor_data = sensor_data.split(",")
            if(float(sensor_data[1]) < 30):
                current_readings.append([float(sensor_data[0]), float(sensor_data[1])])

                # calculate the absolute position of the measured object using the robots current position,
                # measured angle, and measured distance and then add the location to the positions list
                point = robot.calculateAbsolutePosition(float(sensor_data[0]), float(sensor_data[1]))
                points.append(point)
                logging.info("Raw Angle %s\tRaw Dist %s\t Abs point: %s" % (sensor_data[0], sensor_data[1], point))
                server_socket.send_string("{}, {},point".format(point[0], point[1]))
            
            socks = dict(poller.poll())

        # if motor_socket has a reply, set ready_to_move to true
        if motor_socket in socks and socks[motor_socket] == zmq.POLLIN:
            message = motor_socket.recv()
            logging.info("IN POLLER: Received motor reply %s" % message)
            ready_to_move = True

        # if the robot is ready to move, move it
        if ready_to_move:
            # In an effort to remove erroneous data points, each sensor reading is used as a "vote" for the next move
            vote_forward = 0
            vote_left = 0
            vote_right = 0
            vote_not_forward = 0
            # iterate through all the data in the current sensor data list
            for data in current_readings:
                    
                # if a measurement is made on the right 
                if data[0] < -70:
                    #if data[1] > 15:
                        # FIXME: we will have to use this to correct for DRIFT
                    vote_forward += 1
                    logging.info("Voted forward")
                
                # check if a measurement is made in front
                if -20 < data[0] < 20:
                    dist_in_front = (0.25 * data[1]) + (1-0.25) * dist_in_front
                    logging.info("Distance to nearest object in front of robot: %s" % dist_in_front)
                    if(dist_in_front < 15):
                        vote_not_forward += 1
                        logging.info("Object in front, not voting forward")
                
            
            if vote_forward > vote_not_forward and vote_forward > 4:
                robot.moveForward(0.1)
                ready_to_move = False
            elif vote_left > vote_right and vote_left > 4:
                robot.turnLeft()
                net_num_left_turns += 1
                ready_to_move = False
            elif vote_right > vote_left and vote_right > 4:
                robot.turnRight()
                net_num_left_turns -= 1
                ready_to_move = False
            elif dist_in_front > 15:
                robot.moveForward(0.1)
                ready_to_move = False

            else: 
                #vote_left > vote_forward and vote_left > vote_right and vote_left > 3:
                    # if there is an object in front of the robot and to the right, then turn left
                #    robot.turnLeft()
                #    net_num_left_turns += 1
                #elif vote_right > vote_forward and vote_right > vote_left and vote_right > 3:
                    # if there is no object in front of the robot and there is no wall on the right, then turn right
                #    robot.turnRight()
                #    net_num_left_turns -= 1

                logging.error("Decided not to move forward")
                    # robot.moveForward(0.1)

            server_socket.send_string("{}, {},robot".format(robot.pos[0], robot.pos[1]))
            logging.info("Robot current position: %s" % robot.pos)
            current_readings = []
            ready_to_move = False
        else:
            logging.info("Number of current readings: %s" % len(current_readings))


class Robot:
    def __init__(self, pos, dir, motor_socket):
        self.pos = pos
        self.dir = dir
        self.velocity = 1
        self.motor_socket = motor_socket
        motor = Motor()

    def calibrateMotors(self):
        pwm=Servo()
        ultrasonic=Ultrasonic()   
        
        time.sleep(1)

        distance = 0
        for i in range(50):
            distance = distance + ultrasonic.get_distance()/50

        self.moveForward(1)
        message = self.motor_socket.recv()
        logging.info("Received message from motor socket: %s" % message)

        new_distance = 0
        for i in range(50):
            new_distance = new_distance + ultrasonic.get_distance()/50

        self.velocity = distance - new_distance
        return self.velocity

    # Distance is specified in inches
    def moveForward(self, time):

        # transmit a message the motors via zmq socket as F[distance] as a string and wait for reply
        self.motor_socket.send(b"F" + str(time).encode())
        # message = self.motor_socket.recv()
        # logging.info("Received reply to move from motors %s" % message)

        distance = time * self.velocity

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
        self.motor_socket.send(b"L0.7")
        # message = self.motor_socket.recv()
        # logging.info("Received reply to turn from motors %s" % message)

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
        # message = self.motor_socket.recv()
        # logging.info("Received reply to turn from motors %s" % message)

        if self.dir == Direction.NORTH:
            self.dir = Direction.EAST
        elif self.dir == Direction.EAST:
            self.dir = Direction.SOUTH
        elif self.dir == Direction.SOUTH:
            self.dir = Direction.WEST
        elif self.dir == Direction.WEST:
            self.dir = Direction.NORTH

    def getPos(self):
        return self.pos
    
    def getDir(self):
        return self.dir

    def calculateAbsolutePosition(self, angle, distance):
        # calculate the absolute position of the measured object using the robots current position, 
        # measured angle, and measured distance and then add the location to the positions list
        if self.dir == Direction.NORTH:
            return [self.pos[0] - distance * math.sin(math.radians(angle)), self.pos[1] + distance * math.cos(math.radians(angle))]
        elif self.dir == Direction.EAST:
            return [self.pos[0] + distance * math.cos(math.radians(angle)), self.pos[1] + distance * math.sin(math.radians(angle))]
        elif self.dir == Direction.SOUTH:
            return [self.pos[0] + distance * math.sin(math.radians(angle)), self.pos[1] - distance * math.cos(math.radians(angle))]
        elif self.dir == Direction.WEST:
            return [self.pos[0] - distance * math.cos(math.radians(angle)), self.pos[1] - distance * math.sin(math.radians(angle))]
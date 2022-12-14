#
# logic.py
#
# Retrieves sensor data. While conducing "wall following" mode, it first 
# calculates the coordinate location all new points (if any). 
#
# If any points are in front of the robot within one robot's length and 
# to the right, it turns left. If no object in front or right, it moves 
# forward and turns right until it detects an object on the right.
#
# Transmits motor movements as requests to motors
# Waits for motor reply
# Requests sensor data after motor movement
# Wait for sensor data
# Calculate next move and repeat
# Terminate when robot has looped around entire toom

# import necessary libraries
import logging
import math
import threading
import time
import zmq
from enum import Enum

# import class components
from Ultrasonic import *
from servo import *
from sensor import sensor_main

# set global constants
FULL_LEFT_TURN = 0.35
FULL_RIGHT_TURN = 0.4
ENABLE_DRIFT_CORRECTION = True
DRIFT_CORR_VAL = 0.05

# define the cardinal values NORTH, EAST, SOUTH, WEST as 0, 1, 2, 3
class Direction(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

# define the main logic for the mapping
def logic_main():

    points = []             # structure which stores the coordinates of all points seen by the robot
    num_readings = 55       # number of readings to be received from the sensor
    current_readings = []   # list of sensor readings (angle, distance) pairs

    # Create a zmq context
    context = zmq.Context()

    # create a zmq REQ/REP socket for start or stop signal from main
    start_socket = context.socket(zmq.REP)
    start_socket.bind("tcp://*:5557")
    
    # create a zmq REQ/REP motor_socket to communicate with the motors
    motor_socket = context.socket(zmq.REQ)
    motor_socket.connect("tcp://localhost:5555")
    
    # create instance of Robot class and calibrate velcity
    robot = Robot(motor_socket=motor_socket)

    # wait for the start signal from main.py on start socket
    while True:
        message = start_socket.recv()
        if message == b"START":
            logging.info("Received start signal from main")
            break
        elif message ==b"STOP":
            robot.moveForward(0)
            motor_socket.recv()
            motor_socket.close()
            logging.info("IN LOGIC: Received motor reply %s" % message)
            return
        else:
            logging.ERROR("Received unknown start signal from main: %s" % message)

    # calibrate the motors and get the robot's velocity at set speed
    velocity = robot.calibrateMotors()
    logging.info("Calibrated motors. Calculated velocity: %s" % velocity)

    # Wait for GO signal from main.py
    start_socket.send(b"GO")
    message = start_socket.recv()
    if message == b"GO":
        start_socket.send(b"Ack")
        logging.info("Received go signal from main")

    # turn to face the wall and wait for the motors to finish moving
    robot.turnLeft()
    message = motor_socket.recv()
    logging.info("IN LOGIC: Received motor reply %s" % message)
    
    # set the robot direction and position
    robot.dir = Direction.EAST
    robot.pos = [0,0]

    # start ultrasonic sensor thread
    sensor_thread = threading.Thread(target=sensor_main, daemon=True)
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

    # declare the relevant flags and variables
    mapping_done = False            # track if mapping is done
    starting_wall = False           # track if robot is at starting wall
    ready_to_move = True            # track if robot is ready to move
    disregard_data = False          # track if data is useful
    able_to_turn = False            # track if the robot is able to turn
    net_num_left_turns = 0          # net number of turns to track where in room
    measurements_at_90 = []         # store measurements at 90 degrees for drift correction

    # Log that we are starting
    logging.info("Beginning mapping")

    # Loop until the robot receieves a STOP signal from start socket or mapping_done flag is set, loop
    while not mapping_done:
        # poll the sockets
        socks = dict(poller.poll())

        # mapping_done if robot net_num_left_turns is 4
        if net_num_left_turns == 4:
            starting_wall = True
            logging.info("Starting wall reached")

        # if the start socket has a stop signal, mapping_done is true
        if start_socket in socks and socks[start_socket] == zmq.POLLIN:
            message = start_socket.recv()
            if message == b"STOP":
                mapping_done = True
                logging.info("Received stop signal from main")
                break
            else:
                logging.ERROR("Received unknown stop signal from main: %s" % message)

        # receive an entire panning of sensor readings
        while sensor_socket in socks and socks[sensor_socket] == zmq.POLLIN and len(current_readings) < num_readings:
            # receive and parse the sensor data as [angle], [distance]
            sensor_data = sensor_socket.recv_string()
            sensor_data = sensor_data.split(",")
            angle = float(sensor_data[0])
            distance = float(sensor_data[1])
            #logging.info("Sensor reads - angle: %s, distance: %s" % (sensor_data[0], sensor_data[1]))
            current_readings.append([angle, distance])

            # if the data is useful, send the points over to the server to plot
            if(not disregard_data):
                # calculate the absolute position of the measured object using the robots current position,
                # measured angle, and measured distance and then add the location to the positions list
                if(angle <= 20 and distance < 30):
                    point = robot.calculateAbsolutePosition(angle, distance)
                    points.append(point)
                    # logging.info("Abs point: %s" % point)
                    server_socket.send_string("{}, {},point".format(point[0], point[1]))

                # if the measurement is 90 or 85, store the value for drift correction
                if angle == -80:
                    #logging.info("APPEND VALUE %s" % str(distance))
                    measurements_at_90.append(distance)

            # poll the socket again
            socks = dict(poller.poll())

        # if motor_socket has a reply, set ready_to_move to true
        if motor_socket in socks and socks[motor_socket] == zmq.POLLIN:
            message = motor_socket.recv()
            logging.info("IN POLLER: Received motor reply %s" % message)
            server_socket.send_string("{}, {},robot".format(robot.pos[0], robot.pos[1]))
            logging.info("Robot current position: %s" % robot.pos)
            ready_to_move = True

        # if the robot is ready to move, move it
        if ready_to_move and len(current_readings) >= num_readings:
            if disregard_data:
                current_readings = []
                disregard_data = False
                continue

            # In an effort to remove erroneous data points, each sensor reading is used as a "vote" for the next move
            logging.info("Ready to move")
            vote_forward = 0
            vote_left = 0
            vote_right = 0
            vote_not_forward = 0

            # check if there are at least 10 sensor readings
            for data in current_readings:
                # Ignore data if measurement distance is more than 30 cm away
                if(data[1] < 40):
                    # if a measurement is made on the right 
                    if data[0] < -60:
                        vote_forward += 1
                        #logging.info("Voted forward")
                    
                    # check if a measurement is made in front
                    if -20 < data[0] < 20 and data[1] < 25:
                        vote_not_forward += 1
                        vote_left += 1
                        #logging.info("Object in front, not voting forward")

                # if the measurement is more than 30 away and angle is facing right
                elif data[0] < -60:
                    vote_right += 1
                    #logging.info("No object detected on the right, voted turn right")

            # compare votes for the next move and act accordingly
            if vote_forward > vote_not_forward and vote_forward > 4:

                # check if the robot's distance measurements on the right are drifting over time
                # if they are, then turn toward or away from the wall by (DRIFT_CORR_VAL)
                if(ENABLE_DRIFT_CORRECTION):
                    # If the robot has made at least 3 measurements at 90 degrees, check last 5 added values to determine if all the measurements are increasing or decreasing
                    if len(measurements_at_90) >= 3:
                        # calculate measurement diff
                        diff_one = measurements_at_90[-1] - measurements_at_90[-2]
                        diff_two = measurements_at_90[-2] - measurements_at_90[-3]
                        
                        # if the measurements are increasing, turn toward the wall
                        if diff_one > -0.5 and diff_two > -0.5 and diff_one < 0 and diff_two < 0:
                            logging.info("Small drift correction: turning toward from wall")
                            robot.turnLeft(DRIFT_CORR_VAL*0.75)
                            motor_socket.recv()
                            measurements_at_90 = []
                        # if the measurements are decreasing, turn away from the wall
                        elif diff_one < 0.5 and diff_two < 0.5 and diff_one > 0 and diff_two > 0:
                            logging.info("Small drift correction: turning away from wall")
                            robot.turnRight(DRIFT_CORR_VAL*0.75)
                            motor_socket.recv()
                            measurements_at_90 = []
                        elif 1 > diff_one > 0.5 and 1 > diff_two > 0.5:
                            logging.info("Medium drift correction: turning away from wall")
                            robot.turnRight(DRIFT_CORR_VAL)
                            motor_socket.recv()
                            measurements_at_90 = []
                        elif -1 < diff_one < -0.5 and -1 < diff_two < -0.5:
                            logging.info("Medium drift correction: turning toward from wall")
                            robot.turnLeft(DRIFT_CORR_VAL)
                            motor_socket.recv()
                            measurements_at_90 = []
                        elif diff_one < -1 and diff_two < -1:
                            logging.info("Large drift correction: turning toward from wall")
                            robot.turnLeft(DRIFT_CORR_VAL*1.25)
                            motor_socket.recv()
                            measurements_at_90 = []
                        elif diff_one > 1 and diff_two > 1:
                            logging.info("Large drift correction: turning away from wall")
                            robot.turnRight(DRIFT_CORR_VAL*1.25)
                            motor_socket.recv()
                            measurements_at_90 = []

                # move forward and set flags
                robot.moveForward(0.6)
                able_to_turn = True
                ready_to_move = False

            # check for votes to turn left
            elif able_to_turn and vote_left > vote_right and vote_left > 4:
                if starting_wall:
                    mapping_done = True
                    break
                robot.turnLeft()

                # clear drift correction measurements
                measurements_at_90 = []
                net_num_left_turns += 1
                ready_to_move = False
                able_to_turn = False

            # check for votes to turn right
            elif able_to_turn and vote_right > vote_left and vote_right > 4:
                if starting_wall:
                    mapping_done = True
                    break

                # disregard any next turn immediately after turning once
                disregard_data = True
                robot.moveForward(0.5)
                message = motor_socket.recv()
                logging.info("IN TURNING: Received motor reply %s" % message)

                # turn and move forward 
                robot.turnRight()
                message = motor_socket.recv()
                logging.info("IN TURNING: Received motor reply %s" % message)
                robot.moveForward(1)

                # clear drift correction measurements
                measurements_at_90 = []
                net_num_left_turns -= 1
                ready_to_move = False
                able_to_turn = False

            else:
                logging.info("No clear decision, moving forward")
                robot.moveForward(0.2)
                ready_to_move = False
                
            # empty the current readings for next iteration
            current_readings = []

    # signal to subscribers that we are done
    server_socket.send_string(",,done")

    # terminate threads and sockets
    logging.info("Mapping complete")
    motor_socket.close()
    server_socket.close()
    start_socket.close()
    sensor_socket.close()
    
    # save the points to a file
    with open('points.txt', 'w') as f:
        for point in points:
            f.write(str(point))
            f.write('\n')
        
    # mapping is complete and data is saved
    logging.info("Mapping complete, points saved to points.txt")


# Robot class that tracks the current position, direction, velocity, and movement 
class Robot:
    # initialize the variables: position, direction, velocity, and motor socket
    def __init__(self, motor_socket, pos=[0,0], dir=[Direction.EAST]):
        self.pos = pos
        self.dir = dir
        self.velocity = 1
        self.motor_socket = motor_socket

    # calibrate the robot velocity and move to the wall
    def calibrateMotors(self):
        pwm=Servo()
        ultrasonic=Ultrasonic()   
        time.sleep(1)

        # get the current distance from wall
        distance = 0
        for i in range(50):
            distance = distance + ultrasonic.get_distance()/50

        # move for one second
        self.moveForward(1)
        message = self.motor_socket.recv()
        logging.info("Received message from motor socket: %s" % message)

        # get the new distance from wall
        new_distance = 0
        for i in range(50):
            new_distance = new_distance + ultrasonic.get_distance()/50

        # calculate velocity in cm/s as difference in distance over time (1 second)
        self.velocity = distance - new_distance

        # if robot is far from wall, move closer to wall
        if(new_distance > 15):
            self.moveForward((new_distance - 15) / self.velocity)
            message = self.motor_socket.recv()
            logging.info("Received message from motor socket: %s" % message)

        return self.velocity

    # function to move forward and update robot position
    def moveForward(self, time):

        # transmit a message the motors via zmq socket as F[distance] as a string and wait for reply
        self.motor_socket.send(b"F" + str(time).encode())

        # calculate the distance traveled to update position 
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

    # function to turn left and update robot direction
    def turnLeft(self, turn=FULL_LEFT_TURN):
        # transmit a message to the motors via zmq socket to turn left and wait for reply
        self.motor_socket.send(b"L" + str(turn).encode())

        if turn == FULL_LEFT_TURN:
            if self.dir == Direction.NORTH:
                self.dir = Direction.WEST
            elif self.dir == Direction.WEST:
                self.dir = Direction.SOUTH
            elif self.dir == Direction.SOUTH:
                self.dir = Direction.EAST
            elif self.dir == Direction.EAST:
                self.dir = Direction.NORTH

    # function to turn right and update robot direction
    def turnRight(self, turn=FULL_RIGHT_TURN):
        # transmit a message to the motors via zmq socket "R[turn]" and wait for reply
        self.motor_socket.send(b"R" + str(turn).encode())

        if turn == FULL_RIGHT_TURN:
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

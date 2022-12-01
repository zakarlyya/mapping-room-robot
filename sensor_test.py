#
# This program tests the functionality of the robot's sensor and data => point conversion
# Point will be sent over a ZMQ PUB/SUB socket to a laptop that will graph the points
# 

import zmq
import time
import threading
import math
import random
from time import sleep
from sensor import sensor_main

def calculateAbsolutePosition(angle, distance):
        # calculate the absolute position of the measured object using the robots current position, 
        # measured angle, and measured distance and then add the location to the positions list
        return [distance * math.sin(math.radians(angle)), distance * math.cos(math.radians(angle))]

def randomData():
    distance = round(random.random()*300)
    angle = round(random.random()*180 - 90)
    return "{}, {}".format(angle, distance)

if __name__ == '__main__':
    position = [0,0]
    # create a zmq PUB/SUB communications_socket to communicate with the server
    context = zmq.Context()
    server_socket = context.socket(zmq.PUB)
    server_socket.bind("tcp://*:5558")

    sensor_thread = threading.Thread(target=sensor_main)
    sensor_thread.start()

    # create a zmq PUB/SUB sensor_socket to communicate with the sensors
    sensor_socket = context.socket(zmq.SUB)
    sensor_socket.connect("tcp://localhost:5556")
    sensor_socket.setsockopt(zmq.SUBSCRIBE, b"")

    while True:
        sensor_data = sensor_socket.recv_string()
        #sensor_data = randomData()

        sensor_data = sensor_data.split(",")    # parse the sensor data as [angle], [distance]

        # calculate the absolute position of the measured object using the robots current position,
        # measured angle, and measured distance and then add the location to the positions list
        point = calculateAbsolutePosition(float(sensor_data[0]), float(sensor_data[1]))
        print("{}, {},point".format(point[0], point[1]))
        #sleep(0.1)
        server_socket.send_string("{}, {},point".format(point[0], point[1]))

#
# server.py
# Create a network connection which communicates with the robot 
# 
# Implement a REQ/REP scheme which retrieves points from the robot
# and displays them on a live scatter plot in real time
#

import time
import zmq
import random
import numpy as np
import matplotlib.pyplot as plt

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")
x, y= [], []

plt.ion()
fig, ax = plt.subplots()

while True:
    # Wait for next point sent from robot
    message = socket.recv_string()

    point = eval(message[message.find('('):])
    x.append(point[0])
    y.append(point[1])
    
    ax.scatter(x,y, color='red')
    plt.pause(0.1)
    plt.draw()

    # Send reply back to client
    socket.send_string("OK")

plt.waitforbuttonpress()

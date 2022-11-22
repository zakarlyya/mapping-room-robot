#
# This program tests the functionality of the robot's motors
# The robot will move forward, turn left, turn right, and move backward
# 

import zmq
import time
import logging
import threading

# Use motors_main() defined in motors.py to test the motors
from motors import motors_main

if __name__ == '__main__':

    # create a thread which calls motors_main()
    motors_thread = threading.Thread(target=motors_main)

    # start the thread
    motors_thread.start()

    # create a zmq context and socket for REQ/REP
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")


    # send a message to move forward
    socket.send(b"F")
    message = socket.recv()
    logging.info("Received reply: %s" % message)

    # send a message to turn left
    socket.send(b"L")
    message = socket.recv()
    logging.info("Received reply: %s" % message)

    # send a message to turn right
    socket.send(b"R")
    message = socket.recv()
    logging.info("Received reply: %s" % message)

    # send a message to move backward
    socket.send(b"B")
    message = socket.recv()
    logging.info("Received reply: %s" % message)

    # send a message to stop (this will register as unknown)
    socket.send(b"S")
    message = socket.recv()
    logging.info("Received reply: %s" % message)

    # close the socket and forcibly terminate the motors thread
    socket.close()
    motors_thread._stop()





    

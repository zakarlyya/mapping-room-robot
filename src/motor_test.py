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

    # configure loggint to display info messages
    logging.basicConfig(level=logging.INFO)

    # In a loop, wait for terminal input and send the input to the motors thread, exit loop if input is 'q'
    while True:
        message = input("Enter a command: ")
        if message == 'q':
            break
        socket.send(message.encode('utf-8'))
        reply = socket.recv()
        logging.info("Received reply: %s" % reply)

    # close the socket and forcibly terminate the motors thread
    socket.close()
    motors_thread._stop()





    

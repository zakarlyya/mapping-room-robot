#
# main.py
#
# Launches threads for logic, motors, server, sensor
# Waits for the threads
# Terminates

import threading
import time
import queue
import logging
import sys
import zmq

# import logic.py
from logic import logic_main
# import motors.py
from motors import motors_main

if __name__ == '__main__':

    # configure logging to display info messages
    logging.basicConfig(level=logging.INFO, format='(%(threadName)-10s) %(message)s',)

    # create threads
    logic_thread = threading.Thread(target=logic_main)
    motors_thread = threading.Thread(target=motors_main)

    # start the threads
    motors_thread.start()
    logic_thread.start()

    # create REQ zmq socket to send start signal to logic thread
    logic_context = zmq.Context()
    logic_socket = logic_context.socket(zmq.REQ)
    logic_socket.connect("tcp://localhost:5557")

    # while true wait for terminal input, if the input is "START" send a start signal to the logic thread
    # if the input is "STOP" send a stop signal to the logic thread
    while True:
        message = input("Enter START/STOP command: ")
        if message == "START":
            logic_socket.send(b"START")
            reply = logic_socket.recv()
            logging.info("Received reply: %s" % reply)

        # wait for calibration
            calibration_reply = logic_socket.recv()
            if(calibration_reply == b"Calibratted"):
                logging.info("Received reply: %s" % calibration_reply)
                # wait for input GO
                while True:
                    message = input("Enter GO command: ")
                    if message == "GO":
                        logic_socket.send(b"GO")
                        reply = logic_socket.recv()
                        logging.info("Received reply: %s" % reply)
                        break
            break
        elif message == "STOP":
            logic_socket.send(b"STOP")
            logic_socket.recv()
            break
        else:
            logging.info("Unknown command: %s" % message)

    '''
    # create REQ/REP socket to communicate with server
    server_context = zmq.Context()
    server_socket = server_context.socket(zmq.REQ)
    server_socket.connect("tcp://localhost:5558")

    # wait for server to send message to start then send start signal to logic thread
    while True:
        message = server_socket.recv()
        logging.info("Received start signal from server: %s" % message)
        if message == b"START":
            logic_socket.send(b"START")
            logic_socket.recv()
            break
        if message == b"STOP":
            logic_socket.send(b"STOP")
            logic_socket.recv()
            break

        # Reply to server
        server_socket.send(b"ACK")
    '''

    # join threads
    logic_thread.join()
    motors_thread.join()

    # close sockets
    # server_socket.close()
    logic_socket.close()
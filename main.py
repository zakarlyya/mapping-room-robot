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
# import sensor.py


if __name__ == '__main__':
    # create threads
    logic_thread = threading.Thread(target=logic_main, args=(q,))
    motors_thread = threading.Thread(target=motors_main, args=(q,))
    #server_thread = threading.Thread(target=server_main, args=(q,))
    #sensor_thread = threading.Thread(target=sensor_main, args=(q,))

    # start the threads
    motors_thread.start()
    logic_thread.start()


    # create REQ/REP socket to communicate with server
    server_context = zmq.Context()
    server_socket = server_context.socket(zmq.REQ)
    server_socket.connect("tcp://localhost:5558")

    # create REQ zmq socket to send start signal to logic thread
    start_context = zmq.Context()
    start_socket = start_context.socket(zmq.REQ)
    start_socket.connect("tcp://localhost:5557")

    # wait for server to send message to start then send start signal to logic thread
    while True:
        message = server_socket.recv()
        logging.info("Received start signal from server: %s" % message)
        if message == b"START":
            start_socket.send(b"START")
            start_socket.recv()
            break
        if message == b"STOP":
            start_socket.send(b"STOP")
            start_socket.recv()
            break

    # join threads
    logic_thread.join()
    motors_thread.join()


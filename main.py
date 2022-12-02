#
# main.py
#
# Launches threads for logic, motors, server, sensor
# Waits for the threads
# Terminates

# import necessary libraries
import threading
import logging
import zmq

# import logic.py and motors.py
from logic import logic_main
from motors import motors_main

if __name__ == '__main__':

    # configure logging to display info messages
    logging.basicConfig(level=logging.INFO, format='(%(threadName)-10s) %(message)s',)

    # create threads
    logic_thread = threading.Thread(target=logic_main, daemon=True)
    motors_thread = threading.Thread(target=motors_main, daemon=True)

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

            # wait for calibration
            calibration_reply = logic_socket.recv()
            if(calibration_reply == b"GO"):
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
            break
        else:
            logging.info("Unknown command: %s" % message)

    # join threads
    logic_thread.join()

    # close sockets
    logic_socket.close()
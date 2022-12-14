#
# server.py
# Create a network connection which communicates with the robot 
# 
# Implement a PUB/SUB scheme which subscribes to points published by
# the robot and displays them on a live scatter plot in real time
#

# import Qt for graph
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np

# import ZMQ for communication
import zmq

# Create class for graph which is a PqQt Graph Widget
class MyWidget(pg.GraphicsLayoutWidget):
    # Constructor for graph widget
    def __init__(self, socket, parent=None):
        super().__init__(parent=parent)

        # set the main window layout and title 
        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)
        self.setWindowTitle("Autonomous Robot Room Mapping")

        # set graph variables including list of points and robot position
        self.x = []
        self.y = []
        self.robotX = 0
        self.robotY = 0

        # capture PUB/SUB socket
        self.socket = socket

        # add scatter plot to window along with grid and label for axis
        self.plotItem = self.addPlot(title="Ultrasonic points")
        self.plotItem.showGrid(x=True, y=True)
        self.plotItem.setLabel('left', units="m")
        self.plotItem.setLabel('bottom', units="m")
        self.plotItem.setAspectLocked()
        self.plotItem.setAutoVisible(x=True, y=True)

        # add plot item for points and robot
        self.plotPoint = self.plotItem.plot([], pen=None, symbolBrush=(255,0,0), symbolSize=5, symbolPen=None)
        self.plotRobot = self.plotItem.plot([], pen=None, symbolBrush=(0,255,0), symbolSize=20, symbolPen=None)
        self.plotRobot.setData([self.robotX], [self.robotY])
  
        # schedule the plot refreshing
        self.timer = QtCore.QTimer(self)            # create timer
        self.timer.setInterval(1)                   # set timer in milliseconds
        self.timer.start()                          # start the timer
        self.timer.timeout.connect(self.getNewData) # call function on timeout

    # function to get data from the robot and update plot
    def getNewData(self):
        try:
            str = self.socket.recv_string() # get data from PUB/SUB socket
            data = str.split(",")           # split the string up
            print(data)                     # print the data

            # add data to set of points or update robot position then update the plot
            if(data[2] == "point"):
                self.x = np.append(self.x, float(data[0])/100)
                self.y = np.append(self.y, float(data[1])/100)
                self.plotPoint.setData(self.x, self.y)  
            elif(data[2] == "robot"):
                self.robotX = float(data[0])/100
                self.robotY = float(data[1])/100
                self.plotRobot.setData([self.robotX], [self.robotY])
            # if robot has finished mapping, prompt user to exit
            elif(data[2] == "done"):
                self.timer.killTimer(self.timer.timerId())
                self.socket.close()
                print("Done mapping")
                input("Press Enter to exit")
                self.close()
                return
                
        except KeyboardInterrupt:
            self.timer.stop()
            self.timer.killTimer(self.timer.timerId())
            self.socket.close()
            print("Exiting")
            self.close()
            exit()
            

if __name__ == "__main__":
    # create a ZMQ context and connect to PUB/SUB socket and subscribe to all messages
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    robot_ip_address = input("Enter robot IP address: ")
    socket.connect ("tcp://%s:5558" % robot_ip_address)
    socket.subscribe("")
    print("Listening for points")

    try:
        # create graph widget, show, and start timer execution
        app = QtWidgets.QApplication([])
        pg.setConfigOptions(antialias=False)
        win = MyWidget(socket)
        win.show()
        win.resize(600,600) 
        win.raise_()
        app.exec_()

    except KeyboardInterrupt:
        print("Exiting")

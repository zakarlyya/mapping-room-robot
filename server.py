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

class MyWidget(pg.GraphicsLayoutWidget):

    def __init__(self, socket, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)
        self.setWindowTitle("Robot Mapping")

        self.x = []
        self.y = []
        self.robotX = 0
        self.robotY = 0
        self.socket = socket

        self.plotItem = self.addPlot(title="Ultrasonic points")
        self.plotDataItem = self.plotItem.plot([], pen=None, 
            symbolBrush=(255,0,0), symbolSize=5, symbolPen=None)
        self.plotDataItem2 = self.plotItem.plot([], pen=None, 
            symbolBrush=(0,255,0), symbolSize=25, symbolPen=None)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(10) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.getNewData)

    def setData(self, x, y, robotX, robotY):
        self.plotDataItem.setData(x, y)
        self.plotDataItem2.setData([robotX], [robotY])

    def getNewData(self):
        str = self.socket.recv_string()
        data = str.split(",")
        print(data)

        if(data[2] is "point"):
            self.x = np.append(self.x, float(data[0]))
            self.y = np.append(self.y, float(data[1]))
        elif(data[2] is "robot"):
            self.robotX = float(data[0])
            self.robotY = float(data[1])

        self.setData(self.x, self.y, self.robotX, self.robotY)

def main():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    #robot_ip_address = "192.168.86.227"
    #robot_ip_address = "localhost"
    robot_ip_address = input("Enter robot IP address: ")
    socket.connect ("tcp://%s:5558" % robot_ip_address)
    socket.subscribe("")

    app = QtWidgets.QApplication([])
    pg.setConfigOptions(antialias=False)
    win = MyWidget(socket)
    win.show()
    win.resize(800,600) 
    win.raise_()
    app.exec_()

if __name__ == "__main__":
    main()
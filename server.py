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

# import and set up ZMQ socket
import zmq
context = zmq.Context()
socket = context.socket(zmq.SUB)
robot_address = "127.0.0.1"
#socket.connect ("tcp://%s:5557" % robot_address)

class MyWidget(pg.GraphicsLayoutWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)
        # setting title
        self.setWindowTitle("Robot Mapping")

        self.x = []
        self.y = []

        self.plotItem = self.addPlot(title="Ultrasonic points")

        self.plotDataItem = self.plotItem.plot([], pen=None, 
            symbolBrush=(255,0,0), symbolSize=5, symbolPen=None)


    def setData(self, x, y):
        self.plotDataItem.setData(x, y)


    def onNewData(self):
        numPoints = 10 
        self.x = np.append(self.x, np.random.normal(size=numPoints))
        self.y = np.append(self.y, np.random.normal(size=numPoints))
        self.setData(self.x, self.y)


def main():
    app = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=False) # True seems to work as well

    win = MyWidget()
    win.show()
    win.resize(800,600) 
    win.raise_()
    app.exec_()

if __name__ == "__main__":
    main()
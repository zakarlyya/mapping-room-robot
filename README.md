# EECE 4376 Project - Autonomous Room Mapping Robot

## Group Members:
- Zakariyya Al-Quran
- Michael Delaney
- Edwin Campbell

## Project Description
This repository contains the code for an autonomous robot that maps a room by following the walls. The robot scans the room using an ultrasonic sensor that can pan and traverses the wall turning as needed. The walls are detected as points which are saved to the robot and published to any listeners including a server which displays the points in real-time as shown in the example below:
<br/><br/>
![Example output](/images/mapping.png)

## Operating Instructions
1. Connect to the Raspberry Pi robot
2. Download the files in src/ to the Pi
3. Launch the main program file using the command below
    > python main.py
4. (optional) Launch the server program on any clients using the command below and enter the Pi's IP address when prompted
    > python server.py
5. Position the robot facing a wall about 1-3 meters away
6. On the Pi, type START and hit enter to begin calibration
7. Then, on the Pi, type GO to start the mapping
8. Allow the robot to traverse the room. The points are displayed on the server in real time
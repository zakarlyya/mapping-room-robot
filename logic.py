#
# logic.py
#
# Retrieves sensor data. While conducing "wall following" mode, it first 
# calculates the coordinate location all new points (if any). 

# If any points are in front of the robot (within one robot's length and 
# to the right, it turns left. If no object in front or right, it moves 
# forward and turns right until it detects an object on the right.
#
# Transmits motor movements as requests to motors
# Waits for motor reply
# Requests sensor data after motor movement
# Wait for sensor data
# Calculate next move
# Repeat
#
# Terminate when robot comes back to 0,0

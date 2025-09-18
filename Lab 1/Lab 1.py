import sys
from HamBot.src.robot_systems import HamBot
#sys.path.append
import time
import math
def StraightLineFormula(x1, x2, y1, y2):
    #the starting point is x1, y1
    #the next point is x2, y2
    return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2 - y1, 2) )
def Waypoints():
    hey = 0
# kinematics calculations
# using 50 rpms
AxisLength = 4
WheelDiameter = 2.6 #in
WheelRadius = WheelDiameter /2
RPM = 50
LeftWheelRadius = 0
RightWheelRadius = 0
TotalPathLength = 0
TotalTravelTime = 0
#use time.time()  0r time.perf_counter for the travel time
p0 = [2.0,-2.0,math.pi]
p1 = [-1.5, -2.0, math.pi]
p2 = [-2.0, -1.5, math.pi/2]
p3 = [-2.0, -0.5, math.pi/2]
p4 = [-1.0, -0.5, (3 * math.pi)/2]
p5 = [-0.5, -1.0, (7 * math.pi)/4]
p6 = [2.0, -1.0, 0]
p7 = [2.0, 0.0, math.pi/2]
P8 = [0.0, 0.0, math.pi]
P9 = [0.0, 1.0, math.pi/2]
P10 = [-2.0, 1.0, math.pi]
P11 = [-1.0, 2.0, 0]
P12 = [1.5, 2.0, 0]
P13 = [0, 0, 0]
# UNKOWN VALUE

# robot movement
Bot = HamBot(lidar_enabled=False, camera_enabled=False)

P1toP2LeftWheelRadius = StraightLineFormula(p0[0],p1[0],p0[1], p1[1])
print(P1toP2LeftWheelRadius)






# 7  seconds is the end of the 4 *4 maze
# one motor turns negative to turn it 
# left positive and right negative makes it turn clockwise
# 1.19 time is perfect for turning 90 degrees
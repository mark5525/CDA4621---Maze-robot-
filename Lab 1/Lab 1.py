import sys
from HamBot.src.robot_systems import HamBot
sys.path.append
import time
import math

# kinematics calculations
# using 50 rpms
AxisLength = 4
WheelDiameter = 2.6 #in
WheelRadius = WheelDiameter /2
RPM = 50
LeftWheelRadius = (RPM * 2 * math.pi) / 60

#robot movement
Bot = HamBot(lidar_enabled = False, camera_enabled = False)
end = 0
if  end != 7:
    Bot.set_left_motor_speed(50)  
    Bot.set_right_motor_speed(50) 
    time.sleep(6)
    Bot.stop_motors()
    end = 7
# 7  seconds is the end of the 4 *4 maze 
turn = 0
if turn != 1.19:
    Bot.set_left_motor_speed(50)
    Bot.set_right_motor_speed(-50)
    time.sleep(1.19)
    Bot.stop_motors()
    turn = 1.19
# one motor turns negative to turn it 
# left positive and right negative makes it turn clockwise
# 1.19 time is perfect for turning 90 degrees 
turnaround = 0
if turnaround != 7:
    Bot.set_left_motor_speed(50)  
    Bot.set_right_motor_speed(50) 
    time.sleep(6)
    Bot.stop_motors()

x = 0
match(x):
    case 1:
        print("hello world")

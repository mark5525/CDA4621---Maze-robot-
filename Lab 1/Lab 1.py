from robot_systems.robot import HamBot
import time
Bot = HamBot(lidar_enabled = False, camera_enabled = False)
end = 0
if  end != 7:
    Bot.set_left_motor_speed(50)  
    Bot.set_right_motor_speed(50) 
    time.sleep(7)
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
    time.sleep(7)
    Bot.stop_motors()


match(x):
    case 1:
        print("hello world")

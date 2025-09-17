from robot_systems.robot import HamBot
import time
Bot = HamBot(lidar_enabled = False, camera_enabled = False)

'''Bot.set_left_motor_speed(50)  
Bot.set_right_motor_speed(50) 
time.sleep(7)
'''
# 7  seconds is the end of the 4 *4 maze 
Bot.set_left_motor_speed(50)
Bot.set_right_motor_speed(-50)
time.sleep(1.2)
Bot.stop_motors()
x= 1
match(x):
    case 1:
        print("hello world")

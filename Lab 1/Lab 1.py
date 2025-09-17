from robot_systems.robot import HamBot
import time
Bot = HamBot(lidar_enabled = False, camera_enabled = False)
end = 0

Bot.set_left_motor_speed(50)  
Bot.set_right_motor_speed(50) 
time.sleep(6)
Bot.stop_motors()

# 7  seconds is the end of the 4 *4 maze 

Bot.set_left_motor_speed(50)
Bot.set_right_motor_speed(-50)
time.sleep(1.19)
Bot.stop_motors()
turn = 1.19
# one motor turns negative to turn it 
# left positive and right negative makes it turn clockwise
# 1.19 time is perfect for turning 90 degrees 

Bot.set_left_motor_speed(50)  
Bot.set_right_motor_speed(50) 
time.sleep(6)
Bot.stop_motors()

x = 0
match(x):
    case 1:
        print("hello world")

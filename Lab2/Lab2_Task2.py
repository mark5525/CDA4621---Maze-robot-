from HamBot.src.robot_systems.robot import HamBot
from HamBot.src.robot_systems.lidar import Lidar
import time
import math

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)

    wall_reading = input("What wall reading will we be using? ('right' or 'left')")
    #if wall_reading == 'right':


    #if wall_reading == 'left':
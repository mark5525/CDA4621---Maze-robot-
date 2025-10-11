from HamBot.src.robot_systems.robot import HamBot
import math

#def saturation(Bot,v):

#def forward_PID(Bot, forward_distance = 0.6, kp = 3):


if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    wheel_radius = 0.03
    i = 0
    while i != 10:
        forward_distance = min(Bot.get_range_image()[175: 180])
        print("Forward distance: ", forward_distance)
        i = i + 1
        #forward_velocity = forward_PID(kp=3)
        # too close
        #if forward_distance :




from HamBot.src.robot_systems.robot import HamBot
from HamBot.src.robot_systems.lidar import Lidar
import math

def forward_PID(Bot.forward_distance = 0.5, kp = 3):
    actual = min(Bot.get_lidar_range_image())

def side_PID(Bot.side_distance = 0.1, kp = 3, side = "left"):



if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)

    while Bot.experiment_supervisor.step(Bot.timestep) != -1:
        forward_distance = Bot.get_front_distance()
        if forward_distance < Bot.min_forward_wall_distance:
            Bot.rotate(-45)
            forward_velocity = Bot.forward_PID(
            right_v = forward_velocity
            left_v = forward_velocity
            if side == "left":
                delta_velocity = Bot.side_PID(side = "left")
                side_distance = Bot.get_left_side_distance()
                if side_distance < Bot.desired_wall_follow_distance():
                    left_v = Bot.set(left_v * delta_velocity)
                    right_v = Bot.set(right_v * delta_velocity)
                elif side_distance > Bot.desired_wall_follow_distance():
                    right_v = Bot.set(right_v * delta_velocity)
                    left_v = Bot.set(left_v * delta_velocity)
        else:
            pass




from HamBot.src.robot_systems.robot import HamBot
import math


def saturation(Bot,rpm):
    max_rpm = 50
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm

def forward_PID(Bot, f_distance = 600, kp = 3):
    actual = min(Bot.get_range_image()[175: 180])
    e = actual - f_distance
    forward_v = saturation(kp * e)
    return forward_v

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)

    while True:
        forward_distance = min(Bot.get_range_image()[175: 180])
        forward_velocity = forward_PID(Bot, f_distance = 600, kp=3)

        if forward_distance > 600:
            Bot.set_left_motor_speed(forward_velocity)
            Bot.set_right_motor_speed(forward_velocity)
        else:
            Bot.stop_motors()
            break



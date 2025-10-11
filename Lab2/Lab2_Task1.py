from HamBot.src.robot_systems.robot import HamBot
import math


def saturation(Bot,rpm):
    max_rpm = getattr(Bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm

def forward_PID(Bot, f_distance = 600, kp = 0.2):
    scan = Bot.get_range_image()
    d_range = [a for a in scan[175:180] if a > 0]
    if not d_range:
        return 0.0
    actual = min(d_range)
    e = actual - f_distance
    rpm_v = kp * e
    forward_v = saturation(Bot, rpm_v)
    return forward_v

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        forward_velocity = forward_PID(Bot, f_distance = 600, kp=3)

        if forward_distance > 600:
            Bot.set_left_motor_speed(forward_velocity)
            Bot.set_right_motor_speed(forward_velocity)
        else:
            Bot.stop_motors()
            break



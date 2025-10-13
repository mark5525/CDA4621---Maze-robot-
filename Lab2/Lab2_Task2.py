from HamBot.src.robot_systems.robot import HamBot
import math
#

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm


def forward_PID(Bot, f_distance = 300, kp = 0.2):
    scan = Bot.get_range_image()
    d_range = [a for a in scan[175:180] if a > 0]
    if not d_range:
        return 0.0
    actual = min(d_range)
    e = actual - f_distance
    rpm_v = kp * e
    forward_v = saturation(Bot, rpm_v)
    return forward_v

def side_PID(Bot, side_distance = 300, kp = 3, side_follow):
    if side_follow == "left":
        actual = min(Bot.get_range_image()[90:115])
        e = actual - side_distance
        rpm_v = kp * e
        forward_v = saturation(Bot, rpm_v)
        return forward_v
    elif side_follow == "right":
        actual = min(Bot.get_range_image()[270:285])
        e = actual - side_distance
        rpm_v = kp * e
        forward_v = saturation(Bot,rpm_v)
        return forward_v

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    side_follow = "left"
    desired_front_distance = 300
    desired_side_distance = 300

    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        if forward_distance < desired_front_distance:
            rotate(-45)
        forward_velocity = forward_PID(Bot, f_distance=600, kp=3)
        right_v = forward_velocity
        left_v = forward_velocity
        if side_follow == "left":
            delta_velocity = side_PID(side_follow = "left")
            side_distance = min([a for a in Bot.get_range_image()[90:115] if a > 0] or [float("inf")])
            if side_distance < desired_side_distance:
                add_v = left_v + delta_velocity
                left_v = saturation(Bot, add_v)
                sub_v = right_v - delta_velocity
                right_v = saturation(Bot, sub_v)
            elif side_distance > desired_side_distance:
                add_v = right_v + delta_velocity
                right_v = saturation(Bot, add_v)
                sub_v = left_v - delta_velocity
                left_v = saturation(Bot, sub_v)

        if side_follow == "right":
            delta_velocity = side_PID(side_follow = "right")
            side_distance = min([a for a in Bot.get_range_image()[270:285] if a > 0] or [float("inf")])
            if side_distance < desired_side_distance:
                add_v = right_v + delta_velocity
                right_v = saturation(Bot, add_v)
                sub_v = left_v - delta_velocity
                left_v = saturation(Bot, sub_v)
            elif side_distance > desired_side_distance:
                add_v = left_v + delta_velocity
                left_v = saturation(Bot, add_v)
                sub_v = right_v - delta_velocity
                right_v = saturation(Bot, sub_v)

        Bot.set_left_motor_speed(left_v)
        Bot.set_right_motor_speed(right_v)






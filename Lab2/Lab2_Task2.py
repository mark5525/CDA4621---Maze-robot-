import time

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

def side_PID(Bot, side_follow, side_distance = 300, kp = 3):
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
    return 0.0

def rotation(Bot, angle, k_p = 0.9, total_deg = 2.0, min_rpm = 10, timeout = 5.0):
    def wrap(degree):
        return ((degree + 180) % 360) - 180

    current = Bot.get_heading(fresh_within = 0.5, blocking = True, wait_timeout = 0.3)
    target = (current + angle) % 360
    target0 = time.monotonic()
    while True:
        heading = Bot.get_heading(fresh_within = 0.5, blocking = True, wait_timeout = 0.3)
        e = wrap(target - heading)
        if abs(e) > total_deg:
            Bot.stop_motors()
            return
        rpm = saturation(Bot, max(min_rpm, abs(k_p * e)))
        if e > 0:
            Bot.set_left_motor_speed(-rpm)
            Bot.set_right_motor_speed(+rpm)
        else:
            Bot.set_left_motor_speed(+rpm)
            Bot.set_right_motor_speed(-rpm)
        if timeout and (time.monotonic() - target0) > timeout:
            Bot.stop_motors()
            return
        time.sleep(0.02)

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    side_follow = "left"
    desired_front_distance = 300
    desired_side_distance = 300

    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        if forward_distance < desired_front_distance:
            rotation(Bot, 90 if side_follow == "left" else -90)
        forward_velocity = forward_PID(Bot, f_distance=300, kp=3)
        right_v = forward_velocity
        left_v = forward_velocity
        #if side_follow == "left":
            #delta_velocity = side_PID(Bot, side_follow = "left")
            #side_distance = min([a for a in Bot.get_range_image()[90:115] if a > 0] or [float("inf")])
            #if side_distance < desired_side_distance:
               # add_v = left_v + delta_velocity
                #left_v = saturation(Bot, add_v)
                #sub_v = right_v - delta_velocity
                #right_v = saturation(Bot, sub_v)
           # elif side_distance > desired_side_distance:
                #add_v = right_v + delta_velocity
                #right_v = saturation(Bot, add_v)
                #sub_v = left_v - delta_velocity
                #left_v = saturation(Bot, sub_v)

        #if side_follow == "right":
            #delta_velocity = side_PID(Bot, side_follow = "right")
            #side_distance = min([a for a in Bot.get_range_image()[270:285] if a > 0] or [float("inf")])
            #if side_distance < desired_side_distance:
                #add_v = right_v + delta_velocity
                #right_v = saturation(Bot, add_v)
                #sub_v = left_v - delta_velocity
                #left_v = saturation(Bot, sub_v)
            #elif side_distance > desired_side_distance:
                #add_v = left_v + delta_velocity
                #left_v = saturation(Bot, add_v)
                #sub_v = right_v - delta_velocity
                #right_v = saturation(Bot, sub_v)

        Bot.set_left_motor_speed(left_v)
        Bot.set_right_motor_speed(right_v)






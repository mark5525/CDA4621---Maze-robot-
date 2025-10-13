#include <iostream>
from HamBot.src.robot_systems.robot import HamBot
import math
import time

def saturation(Bot,rpm):
    max_rpm = getattr(Bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm

def sector_min(scan, a0, a1):
    values = [c for c in scan[a0:a1 + 1] if c and c > 0]
    return min(values) if values else float('inf')

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

def wrap(degree):
    return ((degree + 180) % 360) - 180

def side_PID(Bot, s_distance = 300, kp = 0.10, side = "left"):
    scan = Bot.get_range_image()
    if side == "left":
        actual = sector_min(scan, 90, 115)
    else:
        actual =  sector_min(scan, 265, 290)
    if actual is float('inf'):
        return 0.0
    e = abs(actual - s_distance)
    return kp * e

def rotate(Bot, end_bearing, margin_error = .000001, print_pose= False):
    starting_encoder_p = getattr(Bot, "start_encoder_p", lambda: None)
    previous_encoder_p = starting_encoder_p
    while True:
        rotation_PID(Bot, end_bearing)
        if end_bearing - margin_error <= Bot.get_heading(fresh_within = 0.5, blocking = True, wait_timeout = 0.3) <= end_bearing + margin_error:
            Bot.stop_motors()
            return

def rotation_PID(Bot, end_bearing, k_p = 0.9, min_rpm = 10):
    current = Bot.get_heading(fresh_within = 0.5, blocking = True, wait_timeout = 0.3)
    delta = wrap(end_bearing - current)
    rpm = max(min_rpm, abs(k_p * delta))
    rpm = saturation(Bot, rpm)
    if delta > 0:
        Bot.set_left_motor_speed(+rpm)
        Bot.set_right_motor_speed(-rpm)
    else:
        Bot.set_left_motor_speed(-rpm)
        Bot.set_right_motor_speed(+rpm)

def curve_rotation(Bot, forward_rpm, follow_side, s_set_mm, band_mm, kp_side):
    scan = Bot.get_range_image()
    if follow_side == "left":
        side_d = sector_min(scan, 90, 115)
    else:
        side_d = sector_min(scan, 265, 290)

    if side_d is float('inf'):
        delta_v = kp_side * (s_set_mm + 3 * band_mm)
    else:
        delta_v = kp_side * abs(side_d - s_set_mm)

    if side_d < (s_set_mm + band_mm):
        if follow_side == "left":
            left_v = saturation(Bot, forward_rpm - delta_v)
            right_v = saturation(Bot,forward_rpm)
        else:
            left_v = saturation(Bot, forward_rpm)
            right_v = saturation(Bot, forward_rpm - delta_v)

    elif side_d > (s_set_mm + band_mm):
        if follow_side == "left":
            left_v = saturation(Bot, forward_rpm)
            right_v = saturation(Bot,forward_rpm - delta_v)
        else:
            left_v = saturation(Bot, forward_rpm - delta_v)
            right_v = saturation(Bot, forward_rpm)
    else:
        left_v = saturation(Bot, forward_rpm)
        right_v = saturation(Bot, forward_rpm)

    return left_v, right_v, side_d, delta_v


if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    follow_mode = "left"
    desired_side = 300
    side_band = 25
    corner_trigger = 350
    loop_hz = 20.0
    dt = 1.0 / loop_hz


    while True:
        scan = Bot.get_range_image()
        if scan in (-1, None):
            Bot.stop_motors(); time.sleep(dt)

        forward_distance = sector_min(scan, 175, 181)

        if forward_distance <= corner_trigger:
            curr = Bot.get_heading(fresh_within = 0.5, blocking = True, wait_timeout = 0.3)
            end = (curr + 90.0) % 360.0 if follow_mode == "left" else (curr - 90.0) % 360.0
            rotate(Bot, end_bearing=end, margin_error = 2.0)
            Bot.stop_motors()
            time.sleep(0.1)
            continue

        forward_velocity = forward_PID(Bot, f_distance = corner_trigger, kp = 0.3)

        left_velocity, right_velocity, side_d, delta_v = curve_rotation(Bot, forward_rpm = forward_velocity, follow_side= follow_mode, s_set_mm = desired_side, kp_side = side_band, kp = 0.10)

        Bot.set_left_motor_speed(left_velocity)
        Bot.set_right_motor_speed(right_velocity)
        time.sleep(dt)




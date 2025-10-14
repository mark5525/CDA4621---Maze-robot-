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
        values = [d for d in Bot.get_range_image()[90:115] if d and d > 0]
        if not values:
            return 0.0
        actual = min(values)
    elif side_follow == "right":
        values = [d for d in Bot.get_range_image()[270:285] if d and d > 0]
        if not values:
            return 0.0
        actual = min(values)
    else:
        return 0.0
    e = actual - side_distance
    rpm_v = kp * e
    return saturation(Bot, rpm_v)

def _front_mm():
    scan = Bot.get_range_image()
    vals = [d for d in scan[175:191] if d and d > 0]
    return min(vals) if vals else float("inf")
def rotation(Bot, angle, pivot_rpm=12, timeout_s=6.0, desired_front_distance=300, extra_clear=150, consecutive_clear=2):
    clear_thresh = desired_front_distance + extra_clear
    clear_hits = 0

    rpm = saturation(Bot, abs(pivot_rpm))

    t0 = time.monotonic()
    while True:
        fwd = _front_mm()
        if fwd >= clear_thresh:
            clear_hits += 1
            if clear_hits >= consecutive_clear:
                Bot.stop_motors()
                return
        else:
            clear_hits = 0
        if angle < 0:
            Bot.set_left_motor_speed(+rpm)
            Bot.set_right_motor_speed(-rpm)
        else:
            Bot.set_left_motor_speed(-rpm)
            Bot.set_right_motor_speed(+rpm)
        if timeout_s and (time.monotonic() - t0) > timeout_s:
            Bot.stop_motors()
            return
        time.sleep(0.02)


if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 50
    side_follow = "left"
    desired_front_distance = 300
    desired_side_distance = 300

    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        if forward_distance < desired_front_distance:
            rotation(Bot, -90 if side_follow == "left" else 90, pivot_rpm = 12)
            continue

        forward_velocity = forward_PID(Bot, f_distance=300, kp=3)
        right_v = forward_velocity
        left_v = forward_velocity
        if side_follow == "left":
            delta_velocity = side_PID(Bot, side_follow = "left")
            delta_velocity = max(-abs(forward_velocity) * 0.9, min(abs(forward_velocity) * 0.9, delta_velocity))
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
            delta_velocity = side_PID(Bot, side_follow = "right")
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






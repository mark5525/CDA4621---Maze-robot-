import time
from HamBot.src.robot_systems.robot import HamBot
import math

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm


def forward_PID(Bot, f_distance = 300, kp = 0.8):
    scan = Bot.get_range_image()
    d_range = [a for a in scan[175:180] if a > 0]
    if not d_range:
        return 0.0
    actual = min(d_range)
    e = actual - f_distance
    rpm_v = kp * e
    forward_v = saturation(Bot, rpm_v)
    return forward_v

def side_PID(Bot, side_follow, side_distance = 300, kp = 0.10):
    if side_follow == "left":
        values = [d for d in Bot.get_range_image()[90:115] if d and d > 0]
    else:
        values = [d for d in Bot.get_range_image()[270:285] if d and d > 0]
    if not values:
        return 0.0
    actual = min(values)
    e = actual - side_distance
    rpm_v = kp * e
    return saturation(Bot, rpm_v)


def rotation(Bot, angle, pivot_rpm = 6, timeout_s = 4.0, desired_front_distance = 300, extra_clear = 40, consecutive_clear = 1):
    def _front_mm():
        scan = Bot.get_range_image()
        vals = [d for d in scan[178:183] if d and d > 0]
        return min(vals) if vals else float("inf")

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
        time.sleep(0.01)



if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 40
    side_follow = "right"
    desired_front_distance = 300
    desired_side_distance = 300

    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        if forward_distance < desired_front_distance:
            rotation(Bot, -90 if side_follow == "left" else 90, pivot_rpm = 12)
            continue
        forward_velocity = forward_PID(Bot, f_distance=300, kp=0.8)
        right_v = forward_velocity
        left_v = forward_velocity
        delta_velocity = side_PID(Bot, side_follow=side_follow, side_distance = desired_side_distance, kp = 0.1)

        lim = abs(forward_velocity) * 0.8
        if delta_velocity > lim: delta_velocity = lim
        if delta_velocity < -lim: delta_velocity = -lim

        if side_follow == "left":
            left_v = saturation(Bot, left_v - delta_velocity)
            right_v = saturation(Bot, right_v + delta_velocity)
        else:
            left_v = saturation(Bot, left_v + delta_velocity)
            right_v = saturation(Bot, right_v - delta_velocity)

        Bot.set_right_motor_speed(right_v)
        Bot.set_left_motor_speed(left_v)

        time.sleep(0.05)  # ~20 Hz








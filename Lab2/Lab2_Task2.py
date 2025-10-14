import time
from HamBot.src.robot_systems.robot import HamBot
import math
import sys

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    return max(-max_rpm, min(max_rpm, rpm))

# --- helpers ---------------------------------------------------------------

def _front_mm(Bot):
    scan = Bot.get_range_image()
    # 180° is front; use a slightly wider 172..188 window and ignore zeros
    vals = [d for d in scan[172:188] if d and d > 0]
    return min(vals) if vals else float("inf")

def _side_vals(Bot, follow):
    scan = Bot.get_range_image()
    # 90° left, 270° right. Slightly wider windows help.
    if follow == "left":
        return [d for d in scan[84:100]  if d and d > 0]
    else:
        return [d for d in scan[260:276] if d and d > 0]

def _shortest_delta_deg(target, current):
    # returns signed delta in (-180, 180]
    return (target - current + 180) % 360 - 180

def rotate_by_degrees(Bot, delta_deg, pivot_rpm=10, tol_deg=3, timeout_s=3.0):
    """Turn the robot in place by +delta_deg (CCW positive), independent of lidar."""
    rpm = saturation(Bot, abs(pivot_rpm))
    start = Bot.get_heading()
    target = (start + delta_deg) % 360
    t0 = time.monotonic()

    while True:
        err = _shortest_delta_deg(target, Bot.get_heading())
        if abs(err) <= tol_deg:
            Bot.stop_motors()
            return
        s = 1 if err > 0 else -1   # + means need CCW
        # CCW: left +, right -
        Bot.set_left_motor_speed( s * rpm)
        Bot.set_right_motor_speed(-s * rpm)

        if timeout_s and (time.monotonic() - t0) > timeout_s:
            Bot.stop_motors()
            return
        time.sleep(0.01)

def creep_forward(Bot, rpm=10, duration=0.20):
    rpm = saturation(Bot, rpm)
    Bot.set_left_motor_speed(rpm)
    Bot.set_right_motor_speed(rpm)
    time.sleep(duration)
    Bot.stop_motors()

# --- controllers -----------------------------------------------------------

def forward_PID(Bot, f_distance=150, kp=0.4):
    scan = Bot.get_range_image()
    d_range = [a for a in scan[172:188] if a > 0]
    if not d_range:
        return 0.0
    actual = min(d_range)
    e = actual - f_distance
    rpm_v = kp * e
    return saturation(Bot, rpm_v)

def side_PID(Bot, side_follow, side_distance=150, kp=0.08):
    vals = _side_vals(Bot, side_follow)
    if not vals:
        return 0.0
    # use a low-quantile to be less jumpy than pure min
    actual = sorted(vals)[max(0, len(vals)//8 - 1)]
    e = actual - side_distance
    rpm_v = kp * e
    return saturation(Bot, rpm_v)

# --- main loop -------------------------------------------------------------

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 40

    # choose which wall to follow; pass as CLI arg if you want (left/right)
    # e.g. python wall_follow.py right
    side_follow = sys.argv[1].lower() if len(sys.argv) > 1 else "left"
    assert side_follow in ("left", "right")

    desired_front_distance = 150   # mm
    desired_side_distance  = 150   # mm

    # debouncing for "side lost"
    side_lost_count = 0
    side_lost_needed = 3   # consecutive frames

    while True:
        scan = Bot.get_range_image()
        forward_distance = min([a for a in scan[172:188] if a > 0] or [float("inf")])
        side_values = _side_vals(Bot, side_follow)

        # ---- 1) Wrap-around when the side wall disappears (corner or opening)
        if not side_values:
            side_lost_count += 1
        else:
            side_lost_count = 0

        if side_lost_count >= side_lost_needed:
            Bot.stop_motors()
            # Turn TOWARD the followed wall by 90°:
            delta = +90 if side_follow == "left" else -90
            rotate_by_degrees(Bot, delta, pivot_rpm=10, tol_deg=3, timeout_s=3.0)
            creep_forward(Bot, rpm=10, duration=0.25)  # reacquire the wall
            side_lost_count = 0
            continue

        # ---- 2) Frontal obstacle: rotate 90° and keep same wall
        if forward_distance < (desired_front_distance):
            Bot.stop_motors()
            delta = +90 if side_follow == "left" else -90  # left-wall => CCW
            rotate_by_degrees(Bot, delta, pivot_rpm=10, tol_deg=3, timeout_s=3.0)
            creep_forward(Bot, rpm=10, duration=0.25)
            continue

        # ---- 3) Normal wall following
        forward_velocity = forward_PID(Bot, f_distance=desired_front_distance, kp=0.4)
        delta_velocity = side_PID(Bot, side_follow=side_follow,
                                  side_distance=desired_side_distance, kp=0.08)

        # clamp steering vs forward speed
        lim = max(abs(forward_velocity) * 0.8, 12)
        delta_velocity = max(-lim, min(lim, delta_velocity))

        left_v = forward_velocity
        right_v = forward_velocity
        if side_follow == "left":
            left_v  = saturation(Bot, left_v  - delta_velocity)
            right_v = saturation(Bot, right_v + delta_velocity)
        else:
            left_v  = saturation(Bot, left_v  + delta_velocity)
            right_v = saturation(Bot, right_v - delta_velocity)

        Bot.set_right_motor_speed(right_v)
        Bot.set_left_motor_speed(left_v)
        time.sleep(0.06)  # ~17 Hz









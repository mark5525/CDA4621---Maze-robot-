import time
from HamBot.src.robot_systems.robot import HamBot
import math
import statistics as stats

# ----------------- helpers -----------------
def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 40)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

def _median_positive(seq):
    vals = [d for d in seq if d and d > 0]
    if not vals:
        return None
    return stats.median(vals)

def _wrap_deg(x):
    # returns angle in (-180, 180]
    return ((x + 180.0) % 360.0) - 180.0

# ----------------- controllers -----------------
def forward_PID(Bot, f_distance=300, kp=0.25):
    scan = Bot.get_range_image()
    # widen window a touch and use median for stability
    d_med = _median_positive(scan[173:187])
    if d_med is None:
        return 0.0
    e = d_med - f_distance
    rpm_v = kp * e
    rpm_v = saturation(Bot, rpm_v)
    return rpm_v

def side_PID(Bot, side_follow, side_distance=300, kp=0.05, deadband=15):
    scan = Bot.get_range_image()
    if side_follow == "left":
        s_med = _median_positive(scan[90:108])
    else:
        s_med = _median_positive(scan[252:270])
    if s_med is None:
        return 0.0
    e = s_med - side_distance
    if abs(e) < deadband:
        return 0.0
    rpm_v = kp * e
    return saturation(Bot, rpm_v)

def rotation(Bot, angle_deg, pivot_rpm=8, timeout_s=1.8, tol_deg=2.0):
    """
    Pivot by a relative angle using IMU (positive = CCW, negative = CW).
    Tapers rpm as we get close to the target to avoid overshoot.
    """
    start = Bot.get_heading()
    target = (start + angle_deg) % 360.0
    t0 = time.monotonic()

    while True:
        now = Bot.get_heading()
        err = _wrap_deg(target - now)

        if abs(err) <= tol_deg:
            Bot.stop_motors()
            return

        # Proportional taper: 0.25 rpm/deg, clamped [4, pivot_rpm]
        rpm_cmd = max(4.0, min(pivot_rpm, 0.25 * abs(err)))
        if err > 0:  # need CCW
            Bot.set_left_motor_speed(-rpm_cmd)
            Bot.set_right_motor_speed(+rpm_cmd)
        else:        # need CW
            Bot.set_left_motor_speed(+rpm_cmd)
            Bot.set_right_motor_speed(-rpm_cmd)

        if timeout_s and (time.monotonic() - t0) > timeout_s:
            Bot.stop_motors()
            return

        time.sleep(0.01)

# ----------------- main -----------------
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 40

    side_follow = "right"           # "left" or "right"
    desired_front_distance = 300    # mm
    desired_side_distance  = 300    # mm

    # Small hysteresis so we don't false-trigger on side gaps
    no_side_counter = 0
    NO_SIDE_THRESH = 2  # need 2 consecutive frames missing the wall

    while True:
        scan = Bot.get_range_image()

        # FRONT distance (median for stability)
        front_med = _median_positive(scan[173:187])
        forward_distance = front_med if front_med is not None else float("inf")

        # SIDE visibility
        if side_follow == "left":
            side_med = _median_positive(scan[90:108])
        else:
            side_med = _median_positive(scan[252:270])

        # Corner detection: side wall disappears consecutively
        if side_med is None:
            no_side_counter += 1
        else:
            no_side_counter = 0

        # --- Corner turn when side truly disappears ---
        if no_side_counter >= NO_SIDE_THRESH:
            Bot.stop_motors()
            time.sleep(0.08)
            rotation(Bot, -90 if side_follow == "left" else +90, pivot_rpm=8)
            no_side_counter = 0
            continue

        # --- Emergency turn if too close to front wall ---
        if forward_distance < desired_front_distance:
            Bot.stop_motors()
            rotation(Bot, -90 if side_follow == "left" else +90, pivot_rpm=8)
            continue

        # --- Forward slowdown zone to avoid diving into corners ---
        # Start easing off as we approach the wall (linear scale)
        slow_zone = desired_front_distance + 200  # mm
        base_fwd = forward_PID(Bot, f_distance=desired_front_distance, kp=0.25)
        if forward_distance < slow_zone:
            # scale in [0.3, 1.0] across the slow zone to maintain some motion but avoid rush
            alpha = max(0.3, (forward_distance - desired_front_distance) / max(1.0, slow_zone - desired_front_distance))
            base_fwd *= alpha

        # --- Side correction (gentle) ---
        delta = side_PID(Bot, side_follow=side_follow,
                         side_distance=desired_side_distance,
                         kp=0.05, deadband=15)

        # Rate-limit steering so it can't yank the robot
        # Cap tied to speed, but with conservative bounds
        steer_cap = min(12.0, max(8.0, 0.5 * abs(base_fwd)))
        if delta >  steer_cap: delta =  steer_cap
        if delta < -steer_cap: delta = -steer_cap

        # Compose wheel speeds
        left_v  = base_fwd
        right_v = base_fwd
        if side_follow == "left":
            left_v  = saturation(Bot, left_v  - delta)
            right_v = saturation(Bot, right_v + delta)
        else:
            left_v  = saturation(Bot, left_v  + delta)
            right_v = saturation(Bot, right_v - delta)

        Bot.set_right_motor_speed(right_v)
        Bot.set_left_motor_speed(left_v)

        time.sleep(0.05)  # ~20 Hz, a hair faster but still stable









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
    return stats.median(vals) if vals else None

def _wrap_deg(x):
    return ((x + 180.0) % 360.0) - 180.0

# ----------------- controllers -----------------
def forward_PID(Bot, f_distance=300, kp=0.20):
    # Median over a wider window = steadier forward speed
    scan = Bot.get_range_image()
    d_med = _median_positive(scan[173:187])
    if d_med is None:
        return 0.0
    e = d_med - f_distance
    rpm_v = kp * e
    return saturation(Bot, rpm_v)

def side_PD(Bot, side_follow, side_distance, kp=0.045, kd=0.18, deadband=18, state=None, dt=0.05):
    scan = Bot.get_range_image()
    if side_follow == "left":
        s_med = _median_positive(scan[90:108])
    else:
        s_med = _median_positive(scan[252:270])

    # keep previous EMA if nothing visible this frame
    if s_med is None:
        return 0.0, state

    # EMA smoothing of side distance
    alpha = 0.70  # higher = smoother
    s_ema = s_med if state["s_ema"] is None else (alpha * state["s_ema"] + (1 - alpha) * s_med)

    e = s_ema - side_distance
    if abs(e) < deadband:
        e = 0.0

    # derivative of error (per second), clipped to avoid spikes
    de = (e - state["prev_e"]) / max(dt, 1e-3)
    de = max(-80.0, min(80.0, de))  # clip mm/s

    # PD with derivative *damping*: kp*e - kd*de
    rpm = kp * e - kd * de

    # update state
    state["prev_e"] = e
    state["s_ema"]  = s_ema

    return saturation(Bot, rpm), state

def rotation(Bot, angle_deg, pivot_rpm=7, timeout_s=2.2, tol_deg=1.5):
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

        # proportional taper (rpm per deg), clamped
        rpm_cmd = max(4.0, min(pivot_rpm, 0.22 * abs(err)))
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

    # corner hysteresis
    no_side_counter = 0
    NO_SIDE_THRESH  = 2  # need 2 consecutive frames w/ no side before turning

    # PD state
    pd_state = {"prev_e": 0.0, "s_ema": None}

    # loop timing
    last_t = time.monotonic()

    while True:
        now = time.monotonic()
        dt = max(0.01, now - last_t)
        last_t = now

        scan = Bot.get_range_image()

        # FRONT (median)
        front_med = _median_positive(scan[173:187])
        forward_distance = front_med if front_med is not None else float("inf")

        # SIDE presence (reuse windows)
        if side_follow == "left":
            side_med_for_presence = _median_positive(scan[90:108])
        else:
            side_med_for_presence = _median_positive(scan[252:270])

        if side_med_for_presence is None:
            no_side_counter += 1
        else:
            no_side_counter = 0

        # --- Corner: side disappears consistently -> turn into the followed wall ---
        if no_side_counter >= NO_SIDE_THRESH:
            Bot.stop_motors()
            time.sleep(0.08)
            # correct directions:
            # follow LEFT -> +90 (CCW); follow RIGHT -> -90 (CW)
            rotation(Bot, +90 if side_follow == "left" else -90, pivot_rpm=7)
            no_side_counter = 0
            pd_state["s_ema"] = None  # reset smoother after big motion
            continue

        # --- Emergency: front wall too close -> same turn as above ---
        if forward_distance < desired_front_distance:
            Bot.stop_motors()
            rotation(Bot, +90 if side_follow == "left" else -90, pivot_rpm=7)
            pd_state["s_ema"] = None
            continue

        # --- Forward speed with broader slow-down ---
        slow_zone = desired_front_distance + 260  # mm
        base_fwd = forward_PID(Bot, f_distance=desired_front_distance, kp=0.20)

        if forward_distance < slow_zone:
            # ease off more aggressively near walls
            alpha = max(0.25, (forward_distance - desired_front_distance) /
                               max(1.0, slow_zone - desired_front_distance))
            base_fwd *= alpha

        # --- Side PD correction (gentle, damped) ---
        delta, pd_state = side_PD(Bot,
                                  side_follow=side_follow,
                                  side_distance=desired_side_distance,
                                  kp=0.045, kd=0.18, deadband=18,
                                  state=pd_state, dt=dt)

        # Cap steering to avoid yanking; scale a little with speed
        steer_cap = min(11.0, max(7.0, 0.45 * abs(base_fwd)))
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

        time.sleep(0.055)  # ~18 Hz (stable)









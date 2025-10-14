import time
import math
import statistics
from HamBot.src.robot_systems.robot import HamBot

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm >  max_rpm: return  max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

def forward_PID(Bot, f_distance = 300, kp = 0.8):
    scan = Bot.get_range_image()
    d_range = [a for a in scan[175:181] if a and a > 0]   # include 180°, ignore invalid
    if not d_range:
        return 0.0
    actual = min(d_range)
    e = actual - f_distance
    rpm_v = kp * e
    return saturation(Bot, rpm_v)

def side_PID(Bot, side_follow, side_distance = 300, kp = 0.08,
             deadband_mm=20, ema_alpha=0.3):
    """Signed magnitude; small deadband + smoothing to reduce oscillation."""
    if side_follow == "left":
        vals = [d for d in Bot.get_range_image()[90:115] if d and d > 0]
        ema_key = "_side_filt_left"
    else:
        vals = [d for d in Bot.get_range_image()[270:285] if d and d > 0]
        ema_key = "_side_filt_right"
    if not vals:
        return 0.0

    # less jitter than min
    raw = statistics.median(vals)

    # EMA smoothing (persists on Bot)
    prev = getattr(Bot, ema_key, raw)
    filt = ema_alpha * raw + (1.0 - ema_alpha) * prev
    setattr(Bot, ema_key, filt)

    e = filt - side_distance
    if abs(e) < deadband_mm:
        return 0.0

    rpm_v = kp * abs(e)           # magnitude only; you apply sign in main
    return saturation(Bot, rpm_v)

def rotation(Bot, angle, pivot_rpm = 6, timeout_s = 4.0,
             desired_front_distance = 300, extra_clear = 40, consecutive_clear = 1):
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

        # keep your original signs
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
    side_follow = "left"              # "left" or "right"
    desired_front_distance = 300      # mm
    desired_side_distance  = 300      # mm

    # curve tuning (small knobs)
    OPENING_MARGIN   = 90    # side > setpoint+margin counts as opening
    LEAD_MARGIN      = 60    # lead sector early-opening sensitivity
    CURVE_GAIN       = 1.4   # boost side correction when curving
    CURVE_FWD_SCALE  = 0.7   # forward scale during curve
    CREEP_MIN_RPM    = 6

    while True:
        scan = Bot.get_range_image()
        forward_distance = min([a for a in scan[175:181] if a and a > 0] or [float("inf")])

        # follow-side slices
        if side_follow == "left":
            side_vals = [d for d in scan[90:115] if d and d > 0]
            lead_vals = [d for d in scan[115:150] if d and d > 0]   # ahead-left (turn sooner)
        else:
            side_vals = [d for d in scan[270:285] if d and d > 0]
            lead_vals = [d for d in scan[240:265] if d and d > 0]   # ahead-right

        # Emergency corner (front)
        if forward_distance < desired_front_distance:
            rotation(Bot, -90 if side_follow == "left" else 90, pivot_rpm = 12)
            continue

        # ---- Curve mode: earlier trigger using LEAD sector ----
        side_open = (not side_vals) or (min(side_vals) > desired_side_distance + OPENING_MARGIN)
        lead_open = (lead_vals and (min(lead_vals) > desired_side_distance + LEAD_MARGIN))
        if side_open or lead_open:
            base_fwd = forward_PID(Bot, f_distance=desired_front_distance, kp=0.8)
            base_fwd = saturation(Bot, max(CREEP_MIN_RPM, abs(base_fwd) * CURVE_FWD_SCALE))

            delta = side_PID(Bot, side_follow=side_follow,
                             side_distance=desired_side_distance, kp=0.08,
                             deadband_mm=20, ema_alpha=0.3)
            delta = saturation(Bot, abs(delta) * CURVE_GAIN)

            lim = abs(base_fwd) * 0.6          # tighter clamp ⇒ straighter arc
            if delta >  lim: delta =  lim
            if delta < -lim: delta = -lim

            left_v = right_v = base_fwd
            if side_follow == "left":
                left_v  = saturation(Bot, left_v  - delta)
                right_v = saturation(Bot, right_v + delta)
            else:
                left_v  = saturation(Bot, left_v  + delta)
                right_v = saturation(Bot, right_v - delta)

            Bot.set_right_motor_speed(right_v)
            Bot.set_left_motor_speed(left_v)
            time.sleep(0.05)
            continue
        # -------------------------------------------------------

        # Normal follow
        forward_velocity = forward_PID(Bot, f_distance=desired_front_distance, kp=0.8)
        left_v = right_v = forward_velocity

        delta_velocity = side_PID(Bot, side_follow=side_follow,
                                  side_distance=desired_side_distance, kp=0.08,
                                  deadband_mm=20, ema_alpha=0.3)

        # keep it straight: steering <= 60% of forward
        lim = abs(forward_velocity) * 0.6
        if delta_velocity >  lim: delta_velocity =  lim
        if delta_velocity < -lim: delta_velocity = -lim

        if side_follow == "left":
            left_v  = saturation(Bot, left_v  - delta_velocity)
            right_v = saturation(Bot, right_v + delta_velocity)
        else:
            left_v  = saturation(Bot, left_v  + delta_velocity)
            right_v = saturation(Bot, right_v - delta_velocity)

        Bot.set_right_motor_speed(right_v)
        Bot.set_left_motor_speed(left_v)
        time.sleep(0.05)  # ~20 Hz








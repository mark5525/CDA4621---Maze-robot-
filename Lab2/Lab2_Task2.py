import time
from HamBot.src.robot_systems.robot import HamBot
import math
import statistics  # <<< for a less noisy side reading

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm: return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

def forward_PID(Bot, f_distance = 300, kp = 0.8):
    scan = Bot.get_range_image()
    d_range = [a for a in scan[175:181] if a and a > 0]  # include 180°, ignore invalids  <<< small widen
    if not d_range:
        return 0.0
    actual = min(d_range)
    e = actual - f_distance
    rpm_v = kp * e
    forward_v = saturation(Bot, rpm_v)
    return forward_v

def side_PID(Bot, side_follow, side_distance = 300, kp = 0.10):
    # widen a bit and use median to reduce jitter  <<<
    if side_follow == "left":
        vals = [d for d in Bot.get_range_image()[90:120] if d and d > 0]   # <<< was 90:105
    else:
        vals = [d for d in Bot.get_range_image()[260:290] if d and d > 0]  # <<< a touch wider
    if not vals:
        return 0.0
    actual = statistics.median(vals)  # <<< was min(...)

    # Correct sign: positive when TOO CLOSE, negative when TOO FAR  <<<
    e = side_distance - actual        # <<< was (actual - side_distance)
    # Tiny deadband keeps it straighter on long walls  <<<
    if abs(e) < 15:
        return 0.0

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
    side_follow = "left"
    desired_front_distance = 300
    desired_side_distance = 300

    # Early-curve knobs (tiny)
    OPENING_MARGIN = 90     # side > setpoint+margin → treat as opening
    LEAD_MARGIN    = 60     # lead sector opens a bit → start curve sooner
    CURVE_GAIN     = 1.4    # boost steering during curve
    CURVE_FWD_SCALE= 0.7    # slow forward slightly during curve
    CREEP_MIN_RPM  = 6

    while True:
        scan = Bot.get_range_image()
        forward_distance = min([a for a in scan[175:181] if a and a > 0] or [float("inf")])

        # Follow-side + a small lead-ahead sector  <<<
        if side_follow == "left":
            side_values = [d for d in scan[90:120]  if d and d > 0]
            lead_values = [d for d in scan[115:150] if d and d > 0]  # ahead-left
        else:
            side_values = [d for d in scan[260:290] if d and d > 0]
            lead_values = [d for d in scan[230:265] if d and d > 0]  # ahead-right

        # EARLY CURVE around sharp turns (no hard rotate)  <<<
        side_open = (not side_values) or (min(side_values) > desired_side_distance + OPENING_MARGIN)
        lead_open = (lead_values and (min(lead_values) > desired_side_distance + LEAD_MARGIN))
        if side_open or lead_open:
            base_fwd = forward_PID(Bot, f_distance=desired_front_distance, kp=0.6)  # slightly softer when curving
            base_fwd = saturation(Bot, max(CREEP_MIN_RPM, abs(base_fwd) * CURVE_FWD_SCALE))

            delta_velocity = side_PID(Bot, side_follow=side_follow,
                                      side_distance=desired_side_distance, kp=0.08)
            delta_velocity = saturation(Bot, delta_velocity * CURVE_GAIN)

            # keep both wheels forward for a smooth arc
            lim = max(abs(base_fwd) * 0.6, 10)  # tighter clamp ⇒ straighter
            if delta_velocity >  lim: delta_velocity =  lim
            if delta_velocity < -lim: delta_velocity = -lim

            left_v = right_v = base_fwd
            if side_follow == "left":
                left_v  = saturation(Bot, left_v  - delta_velocity)
                right_v = saturation(Bot, right_v + delta_velocity)
            else:
                left_v  = saturation(Bot, left_v  + delta_velocity)
                right_v = saturation(Bot, right_v - delta_velocity)

            Bot.set_right_motor_speed(right_v)
            Bot.set_left_motor_speed(left_v)
            time.sleep(0.05)
            continue

        # Front safety rotate (unchanged)
        if forward_distance < desired_front_distance:
            rotation(Bot, -90 if side_follow == "left" else 90, pivot_rpm = 12)
            continue

        # Normal straight follow
        forward_velocity = forward_PID(Bot, f_distance=300, kp=0.4)   # keep modest
        left_v = right_v = forward_velocity

        delta_velocity = side_PID(Bot, side_follow=side_follow,
                                  side_distance=desired_side_distance, kp=0.08)

        # STRAIGHTER: clamp steering tighter  <<<
        lim = max(abs(forward_velocity) * 0.6, 10)  # was 0.8 → 0.6
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
        time.sleep(0.05)








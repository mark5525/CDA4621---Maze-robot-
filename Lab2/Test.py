from HamBot.src.robot_systems.robot import HamBot
import math, time

# ---------- small helpers ----------
def _sector_min_mm(scan, a0, a1):
    """Min positive distance in [a0..a1]; inf if none."""
    vals = [d for d in scan[a0:a1+1] if d and d > 0]
    return min(vals) if vals else float("inf")

def _wrap180(deg):
    return ((deg + 180.0) % 360.0) - 180.0

# ---------- your functions (kept) ----------
def saturation(Bot, rpm):
    max_rpm = getattr(Bot, "max_motor_speed", 60)
    if rpm >  max_rpm: return  max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

def forward_PID(Bot, f_distance=300, kp=0.3):
    """
    Forward-only P controller (outputs RPM). Distances in mm.
    Uses SAME threshold as the corner trigger; no separate 'front set'.
    """
    scan = Bot.get_range_image()
    actual = _sector_min_mm(scan, 175, 181)   # include 180°
    if actual is float("inf"):
        return 0.0
    e_mm = actual - f_distance                # >0 ⇒ too far ⇒ forward
    rpm_cmd = kp * e_mm                       # RPM (kp in RPM/mm)
    return saturation(Bot, rpm_cmd)

# (Optional utility if you still want a side-only magnitude helper)
def side_PID(Bot, s_distance=300, kp=0.10, side="left"):
    scan = Bot.get_range_image()
    if side.lower() == "left":
        actual = _sector_min_mm(scan, 90, 115)
    else:
        actual = _sector_min_mm(scan, 265, 290)
    if actual is float("inf"):
        return 0.0
    e_mm = abs(actual - s_distance)
    return kp * e_mm

# ---------- rotation (your shape, wired) ----------
def rotation_PID(Bot, end_bearing, k_p=0.9, min_rpm=10):
    cur = Bot.get_heading(fresh_within=0.5, blocking=True, wait_timeout=0.3)
    delta = _wrap180(end_bearing - cur)
    rpm = saturation(Bot, max(min_rpm, abs(k_p * delta)))
    if delta > 0:
        Bot.set_left_motor_speed(+rpm)   # CCW
        Bot.set_right_motor_speed(-rpm)
    else:
        Bot.set_left_motor_speed(-rpm)   # CW
        Bot.set_right_motor_speed(+rpm)

def rotate(Bot, end_bearing, margin_error=.000001, print_pose=False):
    starting_encoder_p = getattr(Bot, "get_encoder_readings", lambda: None)()
    previous_encoder_p = starting_encoder_p
    while True:
        rotation_PID(Bot, end_bearing)  # <- preserved call
        h = Bot.get_heading(fresh_within=0.5, blocking=True, wait_timeout=0.3)
        if end_bearing - margin_error <= h <= end_bearing + margin_error:
            Bot.stop_motors()
            return

# ---------- MAIN (if/elif/else steering lives here) ----------
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60  # RPM cap

    FOLLOW_MODE    = "left"   # "left" or "right"
    CORNER_TRIG_MM = 300      # single front threshold
    SIDE_SET_MM    = 300      # desired wall offset
    SIDE_BAND_MM   = 25       # tolerance
    KP_FORWARD     = 0.3      # RPM per mm
    KP_SIDE        = 0.10     # RPM per mm (steering bias)
    LOOP_HZ        = 20.0
    dt             = 1.0 / LOOP_HZ

    while True:
        scan = Bot.get_range_image()
        if scan in (-1, None):
            Bot.stop_motors(); time.sleep(dt); continue

        # measurements
        forward_distance = _sector_min_mm(scan, 175, 181)
        if FOLLOW_MODE.lower() == "left":
            side_distance = _sector_min_mm(scan, 90, 115)
        else:
            side_distance = _sector_min_mm(scan, 265, 290)

        # corner behavior: rotate 90° toward the follow side when front ≤ 300 mm
        if forward_distance <= CORNER_TRIG_MM:
            cur = Bot.get_heading(fresh_within=0.5, blocking=True, wait_timeout=0.3)
            end = (cur + 90.0) % 360.0 if FOLLOW_MODE.lower() == "left" else (cur - 90.0) % 360.0
            rotate(Bot, end_bearing=end, margin_error=2.0)  # looser than 1e-6
            Bot.stop_motors()
            time.sleep(0.1)
            continue

        # base forward command (slows as you approach the same 300 mm threshold)
        forward_velocity = forward_PID(Bot, f_distance=CORNER_TRIG_MM, kp=KP_FORWARD)

        # steering magnitude from side error (RPM); treat "opening" as far
        if side_distance is float("inf"):
            delta_velocity = KP_SIDE * (SIDE_SET_MM + 3*SIDE_BAND_MM)
        else:
            delta_velocity = KP_SIDE * abs(side_distance - SIDE_SET_MM)

        # -------- IF / ELIF / ELSE steering (now in the main loop) --------
        if side_distance < (SIDE_SET_MM - SIDE_BAND_MM):
            # too close to follow-side wall → steer away (slow wheel on follow side)
            if FOLLOW_MODE.lower() == "left":
                left_v  = saturation(Bot, forward_velocity - delta_velocity)
                right_v = saturation(Bot, forward_velocity)
            else:
                right_v = saturation(Bot, forward_velocity - delta_velocity)
                left_v  = saturation(Bot, forward_velocity)

        elif side_distance > (SIDE_SET_MM + SIDE_BAND_MM):
            # too far from wall → steer toward it (slow opposite wheel)
            if FOLLOW_MODE.lower() == "left":
                left_v  = saturation(Bot, forward_velocity)
                right_v = saturation(Bot, forward_velocity - delta_velocity)
            else:
                right_v = saturation(Bot, forward_velocity)
                left_v  = saturation(Bot, forward_velocity - delta_velocity)

        else:
            # in tolerance band → go straight
            left_v  = saturation(Bot, forward_velocity)
            right_v = saturation(Bot, forward_velocity)
        # -------------------------------------------------------------------

        Bot.set_left_motor_speed(left_v)
        Bot.set_right_motor_speed(right_v)

        time.sleep(dt)

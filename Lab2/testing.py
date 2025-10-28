"""
HamBot Wall Following (simplified + smooth corner wrap, mm units)
- Wheel velocities come from side_PID only.
- forward_PID rotates at 300 mm (immediate).
- Corner wrap: anticipates sharp bends and arcs around them smoothly,
  with base-speed ramping to prevent surge.
- Max RPM fixed at 60.
"""
import time
import math
from HamBot.src.robot_systems.robot import HamBot

# ========================
# Setup
# ========================
bot = HamBot(lidar_enabled=True, camera_enabled=False)
bot.max_motor_speed = 60
dt = 0.032

SENSOR_MAX_MM  = 9500
NO_WALL_THRESH = 2500  # treat >= 2.5 m as "no wall" on the chosen side

def safe_distance(mm, max_range=SENSOR_MAX_MM):
    try:
        v = float(mm)
        if v <= 0 or math.isnan(v) or math.isinf(v):
            return max_range
        return min(v, max_range)
    except:
        return max_range

def median_valid_mm(vals):
    """Median of only valid (>0, finite) mm values; else SENSOR_MAX_MM."""
    cleaned = []
    for v in vals:
        try:
            x = float(v)
            if x > 0 and not math.isnan(x) and not math.isinf(x):
                cleaned.append(x)
        except:
            pass
    if not cleaned:
        return SENSOR_MAX_MM
    s = sorted(cleaned)
    n = len(s)
    return s[n//2] if n % 2 else 0.5*(s[n//2-1] + s[n//2])

def median_mm(vals):
    vals = [safe_distance(v) for v in vals if v is not None]
    if not vals: return SENSOR_MAX_MM
    s = sorted(vals); n = len(s)
    return s[n//2] if n % 2 else 0.5*(s[n//2-1] + s[n//2])

def sat(rpm, limit=None):
    lim = bot.max_motor_speed if limit is None else limit
    return max(min(rpm, lim), -lim)

# ========================
# Controller
# ========================
class Defintions:
    def __init__(self):
        # --- Side PD (calm) ---
        self.Kp_side = 0.85
        self.Kd_side = 8.5
        self.StopBand = 15.0

        # Asymmetric shaping (gentle when far; firmer when close)
        self.Kp_far_scale   = 0.55
        self.Kp_close_scale = 1.10
        self.Kd_far_scale   = 0.70
        self.Kd_close_scale = 1.00

        # --- Speed profile (base depends on |side error|) ---
        self.BaseMin  = 20.0
        self.BaseMax  = 55.0
        self.BaseGain = 0.14

        # Base ramp & smoothing (prevents surge at wraps)
        self.BaseLPAlpha = 0.25   # low-pass weight for base
        self.BaseSlew    = 3.0    # max base rpm change per cycle
        self.prev_base   = self.BaseMin

        # Steering limits/smoothing
        self.SteerFrac    = 0.55  # |steer| <= 0.55 * base
        self.SteerMax     = 16.0  # absolute steer cap (rpm)
        self.ForwardFloor = 8.0   # keep both wheels moving forward
        self.SteerLPAlpha = 0.35  # steering low-pass
        self.steer_lp     = 0.0

        # Robust "no wall" persistence
        self.no_wall_count           = 0
        self.no_wall_frames_required = 3
        self.no_wall_clear_frames    = 2

        # Corner wrap parameters (smooth bias + tamed base during wrap)
        self.WrapDiagClear   = 450    # diag must exceed target + 450 mm
        self.WrapSideRiseMM  = 60     # side increases by ~6 cm
        self.WrapBiasMax     = 14.0   # max wrap bias (rpm)
        self.WrapBiasGain    = 0.03   # rpm per mm of (diag - target)
        self.WrapHoldSec     = 0.35   # hold bias briefly
        self.wrap_until_ts   = 0.0
        self.prev_side_meas  = 300.0

        self.WrapBaseScale   = 0.55   # scale base while wrapping
        self.WrapBaseMax     = 32.0   # cap base during wrap
        self.WrapBiasLPAlpha = 0.35   # ramp-in for wrap bias
        self.wrap_bias_lp    = 0.0

        self.prev_err_side = 0.0
        self.Timestep = dt

    # -------- side PID → wheel velocities (with smooth corner wrap) --------
    def side_PID(self, wall="left", target_mm=300):
        scan = bot.get_range_image()
        if not isinstance(scan, list) or len(scan) < 300:
            return 0.0, 0.0

        # Side & forward-diagonal windows
        if wall == "left":
            d_primary  = median_valid_mm(scan[85:95])
            d_fallback = median_valid_mm(scan[95:105])
            d_diag     = median_valid_mm(scan[100:120])  # ahead-left
        else:
            d_primary  = median_valid_mm(scan[265:275])
            d_fallback = median_valid_mm(scan[255:265])
            d_diag     = median_valid_mm(scan[240:260])  # ahead-right

        d_side = min(d_primary, d_fallback)

        # --- No-wall persistence -> gentle search arc (avoid tight circles)
        if d_side >= NO_WALL_THRESH:
            self.no_wall_count = min(self.no_wall_frames_required, self.no_wall_count + 1)
        else:
            self.no_wall_count = max(0, self.no_wall_count - self.no_wall_clear_frames)

        if self.no_wall_count >= self.no_wall_frames_required:
            base = 24.0
            bias = 10.0
            if wall == "left":
                left = base + bias; right = base - bias
            else:
                left = base - bias; right = base + bias
            # decay steer + ease base toward search base
            self.steer_lp = 0.8 * self.steer_lp
            self.prev_side_meas = d_side
            self.prev_base = 0.8 * self.prev_base + 0.2 * base
            return sat(left), sat(right)

        # --- Corner wrap detection (engage short bias if opening & clear)
        now = time.time()
        side_rising = (d_side - self.prev_side_meas) > self.WrapSideRiseMM
        diag_clear  = d_diag > (target_mm + self.WrapDiagClear)
        if (side_rising and diag_clear) or (d_side > target_mm + 80 and diag_clear):
            self.wrap_until_ts = now + self.WrapHoldSec
        wrap_active = now < self.wrap_until_ts

        # --- PD steering on side error (asymmetric gains)
        err = d_side - target_mm
        if abs(err) <= self.StopBand:
            err = 0.0

        if err > 0:   # too far -> gentle pull toward wall
            kp = self.Kp_side * self.Kp_far_scale
            kd = self.Kd_side * self.Kd_far_scale
        else:         # too close -> firmer push away
            kp = self.Kp_side * self.Kp_close_scale
            kd = self.Kd_side * self.Kd_close_scale

        derr = (err - self.prev_err_side) / self.Timestep
        self.prev_err_side = err

        steer_raw = kp * err + kd * derr

        # Wrap bias with smooth ramp-in & correct direction
        if wrap_active:
            raw_bias = min(self.WrapBiasMax, self.WrapBiasGain * max(0.0, d_diag - target_mm))
        else:
            raw_bias = 0.0
        self.wrap_bias_lp = (1 - self.WrapBiasLPAlpha) * self.wrap_bias_lp + self.WrapBiasLPAlpha * raw_bias
        steer_raw += (self.wrap_bias_lp if wall == "left" else -self.wrap_bias_lp)

        # Low-pass steer to kill residual wiggle
        self.steer_lp = (1 - self.SteerLPAlpha) * self.steer_lp + self.SteerLPAlpha * steer_raw
        steer = self.steer_lp

        # Desired base from |err|
        desired_base = min(self.BaseMax, max(self.BaseMin, self.BaseGain * abs(err)))

        # Tame base during wrap (scale + cap), then ramp/slew to avoid surge
        if wrap_active:
            desired_base = min(self.WrapBaseMax, self.WrapBaseScale * desired_base)

        base_lp = (1 - self.BaseLPAlpha) * self.prev_base + self.BaseLPAlpha * desired_base
        delta   = max(-self.BaseSlew, min(self.BaseSlew, base_lp - self.prev_base))
        base    = self.prev_base + delta
        self.prev_base = base

        # Steering clamps relative to (possibly reduced) base
        steer_cap = min(self.SteerFrac * max(base, 1.0), self.SteerMax)
        steer = math.copysign(min(abs(steer), steer_cap), steer)

        # Mix signed steer into wheels
        if wall == "left":
            left_rpm  = base - steer
            right_rpm = base + steer
        else:
            left_rpm  = base + steer
            right_rpm = base - steer

        # Forward floor (avoid pivot-in-place)
        left_rpm  = max(left_rpm,  self.ForwardFloor)
        right_rpm = max(right_rpm, self.ForwardFloor)

        self.prev_side_meas = d_side
        return sat(left_rpm), sat(right_rpm)

    # -------- forward "PID": immediate rotate at 300 mm --------
    def forward_PID(self, wall="left", target_mm=300):
        scan = bot.get_range_image()
        if not isinstance(scan, list) or len(scan) < 185:
            return
        front_med = median_mm(scan[175:185])
        if front_med <= target_mm:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            self.rotate(math.pi/2 if wall == "left" else -math.pi/2)

    # -------- simple IMU-based 90° rotate --------
    def rotate(self, rad_angle, rpm=16):
        left_sign  =  1 if rad_angle > 0 else -1
        right_sign = -left_sign

        def ang_delta(a, b):
            return (a - b + math.pi) % (2*math.pi) - math.pi

        start = bot.get_heading()
        while True:
            curr = bot.get_heading()
            if abs(ang_delta(curr, start)) >= abs(rad_angle):
                break
            bot.set_left_motor_speed( left_sign  * rpm)
            bot.set_right_motor_speed(right_sign * rpm)
            time.sleep(self.Timestep)

        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        time.sleep(0.05)

# ========================
# Main loop (simple)
# ========================
pp = Defintions()
wall = "left"         # or "right"
SIDE_TARGET  = 300    # mm
FRONT_TARGET = 300    # mm

while True:
    # 1) Wheel speeds come ONLY from side controller
    left_rpm, right_rpm = pp.side_PID(wall=wall, target_mm=SIDE_TARGET)
    bot.set_left_motor_speed(left_rpm)
    bot.set_right_motor_speed(right_rpm)

    # 2) Front check: rotate at ~300 mm
    pp.forward_PID(wall=wall, target_mm=FRONT_TARGET)

    time.sleep(dt)



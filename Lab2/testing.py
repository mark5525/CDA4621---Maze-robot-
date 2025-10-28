"""
HamBot Wall Following (simplified, mm units)
- Wheel velocities come from side_PID only.
- forward_PID only checks the front and rotates at 300 mm (immediate).
- Max RPM fixed at 60.
- Fixes: asymmetric steer (gentle when far, stronger when close), robust side read,
  no-wall persistence, and unconditional front rotate.
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

class Defintions:
    def __init__(self):
        # --- Side PD (asymmetric) ---
        self.Kp_side = 1.0    # base proportional (rpm per mm)
        self.Kd_side = 7.0    # derivative (rpm per (mm/s))
        self.StopBand = 12.0  # mm deadband around 300 mm

        # Asymmetric scaling: gentler when far (err>0), stronger when close (err<0)
        self.Kp_far_scale = 0.55   # pull toward wall softly
        self.Kp_close_scale = 1.20 # push away from wall more decisively
        self.Kd_far_scale = 0.6
        self.Kd_close_scale = 1.0

        # --- Speed profile ---
        self.BaseMin  = 20.0   # rpm
        self.BaseMax  = 55.0   # rpm
        self.BaseGain = 0.14   # rpm per mm of |error|

        # Steering limits
        self.SteerFrac = 0.55  # |steer| <= 0.55 * base (tamer curvature)
        self.SteerMax  = 16.0  # absolute steer cap (rpm)
        self.ForwardFloor = 8.0  # ensure both wheels have forward motion

        self.prev_err_side = 0.0
        self.Timestep = dt

        # --- Front rotate: immediate at 300 mm (no gating) ---
        # (kept minimal to guarantee it triggers)

        # --- No-wall persistence to avoid big circles ---
        self.no_wall_count           = 0
        self.no_wall_frames_required = 3   # need 3 frames of "no wall" to arc
        self.no_wall_clear_frames    = 2   # need 2 frames of wall to clear

    # -------- side PID → wheel velocities --------
    def side_PID(self, wall="left", target_mm=300):
        scan = bot.get_range_image()
        if not isinstance(scan, list) or len(scan) < 300:
            return 0.0, 0.0  # safe default

        # Side distance (robust): primary window 85:95, fallback neighbor
        if wall == "left":
            d_primary  = median_valid_mm(scan[85:95])
            d_fallback = median_valid_mm(scan[95:105])
        else:
            d_primary  = median_valid_mm(scan[265:275])
            d_fallback = median_valid_mm(scan[255:265])

        d_side = min(d_primary, d_fallback)

        # "No wall" persistence
        if d_side >= NO_WALL_THRESH:
            self.no_wall_count = min(self.no_wall_frames_required,
                                     self.no_wall_count + 1)
        else:
            self.no_wall_count = max(0, self.no_wall_count - self.no_wall_clear_frames)

        if self.no_wall_count >= self.no_wall_frames_required:
            # Gentle search arc (not a tight circle)
            base = 24.0
            bias = 10.0
            if wall == "left":
                left = base + bias
                right = base - bias
            else:
                left = base - bias
                right = base + bias
            return sat(left), sat(right)

        # Error relative to 300 mm
        err = d_side - target_mm

        # Deadband
        if abs(err) <= self.StopBand:
            err = 0.0

        # Asymmetric gain scheduling
        if err > 0:   # too far from the wall → gentle pull
            kp = self.Kp_side * self.Kp_far_scale
            kd = self.Kd_side * self.Kd_far_scale
        else:         # too close to the wall → stronger push
            kp = self.Kp_side * self.Kp_close_scale
            kd = self.Kd_side * self.Kd_close_scale

        # PD steering
        derr = (err - self.prev_err_side) / self.Timestep
        self.prev_err_side = err
        steer = kp * err + kd * derr  # rpm, signed

        # Base forward speed from |err|
        base = min(self.BaseMax, max(self.BaseMin, self.BaseGain * abs(err)))

        # Steering clamps
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

        return sat(left_rpm), sat(right_rpm)

    # -------- forward "PID": immediate rotate at 300 mm (no gating) --------
    def forward_PID(self, wall="left", target_mm=300):
        scan = bot.get_range_image()
        if not isinstance(scan, list) or len(scan) < 185:
            return
        front_med = median_mm(scan[175:185])
        if front_med <= target_mm:
            # Stop, rotate 90°, resume
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
# Main loop (kept simple)
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

    # 2) Front check: rotate immediately at ~300 mm
    pp.forward_PID(wall=wall, target_mm=FRONT_TARGET)

    time.sleep(dt)



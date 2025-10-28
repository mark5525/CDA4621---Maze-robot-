"""
HamBot Wall Following (simplified, mm units)
- Wheel velocities come from side_PID only.
- forward_PID only checks the front and rotates at 300 mm (with tiny gating).
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
        # --- Side PD (keep it simple) ---
        self.Kp_side = 1.0    # rpm per mm
        self.Kd_side = 6.0    # rpm per (mm/s)
        self.StopBand = 12.0  # mm deadband around 300 mm

        # --- Speed profile (faster than before) ---
        self.BaseMin  = 18.0  # rpm (was 10)
        self.BaseMax  = 55.0  # rpm (was 40)
        self.BaseGain = 0.14  # rpm per mm of |error| (was 0.08)

        # Keep steer from overpowering base
        self.SteerFrac = 0.85  # |steer| <= 0.85 * base

        self.prev_err_side = 0.0
        self.last_steer = 0.0
        self.Timestep = dt

        # --- Front rotate gating (still “simple”) ---
        self.front_frames_required = 2     # need 2 consecutive frames
        self.front_hysteresis_mm   = 20    # must rise above 320 to reset
        self.front_hit_count       = 0
        self.rotate_gate_rpm       = 10.0  # only rotate if |steer| <= 10 rpm
        self.rotate_refractory_s   = 1.0   # seconds between rotates
        self.last_rotate_time      = time.time() - 5.0

    # -------- side PID → wheel velocities --------
    def side_PID(self, wall="left", target_mm=300):
        scan = bot.get_range_image()
        if not isinstance(scan, list) or len(scan) < 300:
            return 0.0, 0.0  # safe default

        # Side windows (you asked for these)
        if wall == "left":
            d_side = median_mm(scan[85:95])
        else:
            d_side = median_mm(scan[265:275])

        # If no wall, do a gentle search arc (make it brisker)
        if d_side >= NO_WALL_THRESH:
            base = 22.0
            bias = 12.0
            if wall == "left":
                left = base + bias
                right = base - bias
            else:
                left = base - bias
                right = base + bias
            self.last_steer = (right - left) * 0.5  # approx steer
            return sat(left), sat(right)

        # Error to target (300 mm)
        err = d_side - target_mm

        # Deadband to avoid nibbling
        if abs(err) <= self.StopBand:
            err = 0.0

        # PD steering (signed)
        derr = (err - self.prev_err_side) / self.Timestep
        self.prev_err_side = err
        steer = self.Kp_side * err + self.Kd_side * derr  # rpm, signed

        # Base forward speed from |err| (faster profile)
        base = min(self.BaseMax, max(self.BaseMin, self.BaseGain * abs(err)))

        # Clamp steer so it can't flip a wheel negative unless base is tiny
        steer = math.copysign(min(abs(steer), self.SteerFrac * max(base, 1.0)), steer)
        self.last_steer = steer

        # Mix signed steer into wheels
        if wall == "left":
            left_rpm  = base - steer
            right_rpm = base + steer
        else:
            left_rpm  = base + steer
            right_rpm = base - steer

        return sat(left_rpm), sat(right_rpm)

    # -------- forward "PID": rotate when front is close (with tiny gating) --------
    def forward_PID(self, wall="left", target_mm=300):
        scan = bot.get_range_image()
        if not isinstance(scan, list) or len(scan) < 185:
            return

        front_med = median_mm(scan[175:185])  # robust to outliers
        if front_med <= target_mm:
            self.front_hit_count += 1
        elif front_med >= (target_mm + self.front_hysteresis_mm):
            self.front_hit_count = 0  # reset only after clearing hysteresis

        # Only rotate if: confirmed close, not in a hard turn, and not repeating too fast
        now = time.time()
        if (self.front_hit_count >= self.front_frames_required
            and abs(self.last_steer) <= self.rotate_gate_rpm
            and (now - self.last_rotate_time) >= self.rotate_refractory_s):
            self.rotate(math.pi/2 if wall == "left" else -math.pi/2)
            self.last_rotate_time = time.time()
            self.front_hit_count = 0  # reset after rotate

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

    # 2) Front check: rotate at ~300 mm (robust + gated)
    pp.forward_PID(wall=wall, target_mm=FRONT_TARGET)

    time.sleep(dt)


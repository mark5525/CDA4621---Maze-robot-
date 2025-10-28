"""
HamBot Wall Following PID Controller (Real Robot Version, mm units)
Targets: front=300 mm, side=300 mm, max RPM=60
Tuned to reduce side oscillation: median windows, filtered D, milder gains,
steer mixing, slew limit, and wheel differential cap.
"""
import time
import math
from robot_systems.robot import HamBot  # HamBot 라이브러리

# ========================
# HamBot Setup
# ========================
bot = HamBot(lidar_enabled=True, camera_enabled=False)
bot.max_motor_speed = 60  # keep max RPM at 60
dt = 0.032

# ========================
# Utility functions & constants
# ========================
SENSOR_MAX_MM  = 9500   # ~9.5 m fallback (mm)
NO_WALL_THRESH = 2500   # 2.5 m in mm

def safe_distance(value_mm, max_range=SENSOR_MAX_MM):
    """Clamp bad/missing LiDAR values to a safe 'far away' distance in mm."""
    try:
        v = float(value_mm)
        if math.isinf(v) or math.isnan(v) or v <= 0:
            return max_range
        return min(v, max_range)
    except:
        return max_range

def median_mm(vals):
    """Robust central tendency for LiDAR slices (in mm)."""
    cleaned = [safe_distance(v) for v in vals if v is not None]
    if not cleaned:
        return SENSOR_MAX_MM
    s = sorted(cleaned)
    n = len(s)
    return s[n//2] if n % 2 else 0.5 * (s[n//2 - 1] + s[n//2])

def saturation(speed_rpm, max_speed=None):
    """Saturate to robot max RPM (defaults to bot.max_motor_speed=60)."""
    limit = bot.max_motor_speed if max_speed is None else max_speed
    return max(min(speed_rpm, limit), -limit)

# ========================
# Controller
# ========================
class Defintions():
    def __init__(self):
        # Forward PID gains (rpm per mm)
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5

        # Side distance + angle gains (softened to reduce oscillations)
        self.Kp = 1.2     # distance-to-wall (rpm/mm)  (was 2.0)
        self.Ki = 0.0
        self.Kd = 8.0     # derivative (rpm/mm/s)     (was 5.0)

        self.Kphi = 12.0  # wall-angle gain (rpm/rad) (was 18–30)
        self.SideDeltaDeg = 12  # separation between side beams for angle estimate

        self.Timestep = dt

        # PID state
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

        # Derivative low-pass filter for side loop ("dirty derivative")
        self.DAlpha = 0.7     # 0.0=no filter, 0.9=very smooth
        self.SideDerivFilt = 0.0

        # Helpers
        self.StopBand = 15.0       # mm (slightly wider to avoid nibbling)
        self.I_Limit  = 200.0
        self.ApproachSlope = 0.5
        self.MinApproachRPM = 6.0

        # Steering shaping
        self.SteerMix  = 0.35      # fraction of steer sent to wheels (0..1)
        self.SteerSlew = 4.0       # max steer change per cycle (rpm)
        self.prevSteerCmd = 0.0
        self.DiffCap   = 20.0      # max (right - left) instantaneous split (rpm)
        self.BaseCap   = 32.0      # cap forward base so steer has authority
        self.TurnSlowdown = 0.8    # stronger slowdown when turning hard

    def reset_pids(self):
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0
        self.SideDerivFilt = 0.0
        self.prevSteerCmd = 0.0

    # ---------- forward (front-distance) PID -> base throttle ----------
    def forward_PID(self, desired_distance_mm=300):
        lidar = bot.get_range_image()
        win = lidar[175:185] if isinstance(lidar, list) else []
        measured = median_mm(win)                 # mm (robust)
        error = measured - desired_distance_mm    # mm

        if abs(error) <= self.StopBand:
            self.ForwardIntegral = 0.0
            self.ForwardPrevError = 0.0
            return 0.0

        self.ForwardIntegral += error * self.Timestep
        self.ForwardIntegral = max(-self.I_Limit, min(self.ForwardIntegral, self.I_Limit))
        d_err = (error - self.ForwardPrevError) / self.Timestep
        self.ForwardPrevError = error

        u = (self.K_p * error) + (self.K_i * self.ForwardIntegral) + (self.K_d * d_err)

        # soft approach & base cap
        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)
        u = math.copysign(min(abs(u), self.BaseCap), u)

        return saturation(u)

    # ---------- angle-aware side control (chosen wall-distance -> steering) ----------
    def side_PID(self, wall="left", desired_distance_mm=300):
        if wall not in ("left", "right"):
            return 0.0

        lidar = bot.get_range_image()
        if not isinstance(lidar, list) or len(lidar) < 300:
            return 0.0

        # Primary distance windows as requested; angle uses a nearby offset window
        if wall == "left":
            d0 = median_mm(lidar[85:95])      # ~90°
            d1 = median_mm(lidar[100:110])    # ~90° + Δ
        else:
            d0 = median_mm(lidar[265:275])    # ~270°
            d1 = median_mm(lidar[250:260])    # ~270° - Δ

        # No wall on chosen side → let wall_follow handle search arc
        if d0 >= NO_WALL_THRESH:
            return 0.0

        # Distance error with stop-band
        e_d = d0 - desired_distance_mm
        if abs(e_d) <= self.StopBand:
            self.SideIntegral = 0.0
            self.SidePrevError = 0.0
            e_d = 0.0

        # Gain scheduling: soften Kp near setpoint (<= 100 mm)
        mag = min(abs(e_d) / 100.0, 1.0)
        Kp_eff = (0.5 + 0.5 * mag) * self.Kp  # 50%..100% of Kp

        # Local wall angle estimate φ using two beams separated by Δ
        Δ = math.radians(self.SideDeltaDeg)
        phi = math.atan2(d0 * math.cos(Δ) - d1, max(1e-6, d0 * math.sin(Δ)))
        if wall == "right":
            phi = -phi  # unify control sense

        # Filtered derivative of distance error
        d_err_raw = (e_d - self.SidePrevError) / self.Timestep
        self.SideDerivFilt = self.DAlpha * self.SideDerivFilt + (1.0 - self.DAlpha) * d_err_raw
        self.SidePrevError = e_d

        # Distance PD + Angle P
        u_dist = (Kp_eff * e_d) + (self.Kd * self.SideDerivFilt)
        u_angl = self.Kphi * phi
        u = u_dist + u_angl

        # Raw steer cap
        steer_cap = 0.6 * bot.max_motor_speed
        u = math.copysign(min(abs(u), steer_cap), u)

        # Slew-limit steering to avoid snap turns
        delta = max(-self.SteerSlew, min(self.SteerSlew, u - self.prevSteerCmd))
        u_slewed = self.prevSteerCmd + delta
        self.prevSteerCmd = u_slewed

        return saturation(u_slewed)

    # ---------- IMU-based rotate (radians; + = left/CCW) with soft finish ----------
    def rotate(self, radianAngle, base_rpm=18):
        self.reset_pids()
        left_sign  =  1 if radianAngle > 0 else -1
        right_sign = -left_sign

        def ang_delta(curr, ref):
            return (curr - ref + math.pi) % (2 * math.pi) - math.pi

        start = bot.get_heading()
        target = abs(radianAngle)
        angle_eps = math.radians(3.0)  # ~3°

        while True:
            curr = bot.get_heading()
            delta = abs(ang_delta(curr, start))
            rem = target - delta
            if rem <= angle_eps:
                break

            rpm = base_rpm
            if rem < math.radians(20):
                rpm = max(6, base_rpm * 0.5)
            if rem < math.radians(10):
                rpm = max(4, base_rpm * 0.3)

            bot.set_left_motor_speed( left_sign  * rpm)
            bot.set_right_motor_speed(right_sign * rpm)
            time.sleep(self.Timestep)

        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        self.reset_pids()
        time.sleep(self.Timestep)

    # ---------- high-level: wall following (returns right_rpm, left_rpm) ----------
    def wall_follow(self, wall="left"):
        lidar = bot.get_range_image()
        if not isinstance(lidar, list) or len(lidar) < 300:
            return 0.0, 0.0

        left_distance  = median_mm(lidar[85:95])     # robust distance readout
        right_distance = median_mm(lidar[265:275])

        print("LEFT D:", int(left_distance), "mm")
        print("RIGHT D:", int(right_distance), "mm")

        target_mm = 300  # side & front targets are 300 mm

        # 1) Base forward speed from front controller (capped)
        linear_velocity = self.forward_PID(desired_distance_mm=target_mm)

        # 2) Steering from side controller (SIGNED, smoothed)
        steer_raw = self.side_PID(wall=wall, desired_distance_mm=target_mm)
        steer_for_mix = self.SteerMix * steer_raw

        # Slow down base when turning hard
        slow_factor = max(0.35, 1.0 - self.TurnSlowdown * (abs(steer_raw) / bot.max_motor_speed))
        linear_velocity *= slow_factor

        rightv = leftv = linear_velocity

        # 3) If no wall on chosen side -> search arc (gentle)
        side_distance = right_distance if wall == "right" else left_distance
        search_sign = +1 if wall == "right" else -1
        if side_distance >= NO_WALL_THRESH:
            base = 0.5 * linear_velocity
            bias = 0.20 * bot.max_motor_speed
            rightv = base - search_sign * bias
            leftv  = base + search_sign * bias
            return saturation(rightv), saturation(leftv)

        # 4) Signed steering mix with differential cap
        if wall == "left":
            leftv  = linear_velocity - steer_for_mix
            rightv = linear_velocity + steer_for_mix
        else:
            leftv  = linear_velocity + steer_for_mix
            rightv = linear_velocity - steer_for_mix

        diff = rightv - leftv
        if abs(diff) > self.DiffCap:
            avg = 0.5 * (rightv + leftv)
            half = self.DiffCap / 2.0 * (1 if diff >= 0 else -1)
            rightv = avg + half
            leftv  = avg - half

        return saturation(rightv), saturation(leftv)

# ========================
# Instantiate controller
# ========================
pp = Defintions()

# ========================
# Main loop (same style/flow as your reference) with rotate gating
# ========================
wall = "left"
last_rotate_time = 0.0
ROTATE_REFRACTORY = 1.0          # s: wait after a rotate
STEER_ROTATE_GATE = 10.0         # rpm: only rotate if not turning hard

while True:

    lidar = bot.get_range_image()
    if isinstance(lidar, list) and len(lidar) >= 360:
        center_idx = 180  # front
        print(f"Front distance: {safe_distance(lidar[center_idx]):.0f} mm")
    else:
        print("No LiDAR data received")

    rightv, leftv = pp.wall_follow(wall)
    bot.set_left_motor_speed(leftv)
    bot.set_right_motor_speed(rightv)

    # front window for corner handling (mm)
    if isinstance(lidar, list) and len(lidar) >= 185:
        front_distance = min(safe_distance(v) for v in lidar[175:185])
    else:
        front_distance = SENSOR_MAX_MM

    print(front_distance)
    print("-" * 50)

    # Rotate if very close ahead; gate to avoid false triggers during strong steering
    now = time.time()
    if (front_distance < 450
        and abs(pp.prevSteerCmd) <= STEER_ROTATE_GATE
        and (now - last_rotate_time) > ROTATE_REFRACTORY):
        if wall == "right":
            pp.rotate(-math.pi / 2)
        else:
            pp.rotate(math.pi / 2)
        last_rotate_time = time.time()

    time.sleep(dt)

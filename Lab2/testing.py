"""
HamBot Wall Following PID Controller (Real Robot Version, mm units)
Targets: front=300 mm, side=300 mm, max RPM=60
Adds: mixing gain, steer slew-limit, rotate gating to prevent dive & spin
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
# Utility functions
# ========================
SENSOR_MAX_MM  = 9500   # ~9.5 m fallback (mm)
NO_WALL_THRESH = 2500   # 2.5 m in mm

def safe_distance(value_mm, max_range=SENSOR_MAX_MM):
    # Clamp bad values to "far" so logic treats as no wall
    try:
        v = float(value_mm)
        if math.isinf(v) or math.isnan(v) or v <= 0:
            return max_range
        return min(v, max_range)
    except:
        return max_range

def saturation(speed_rpm, max_speed=None):
    # Use robot’s max if not provided
    limit = bot.max_motor_speed if max_speed is None else max_speed
    return max(min(speed_rpm, limit), -limit)

# ========================
# Your controller class (kept)
# ========================
class Defintions():
    def __init__(self):
        # Forward PID gains (rpm per mm)
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5

        # Side distance + angle gains
        self.Kp = 2.0     # distance-to-wall (rpm/mm)
        self.Ki = 0.0
        self.Kd = 5.0     # distance derivative (rpm/mm/s)

        self.Kphi = 18.0  # angle gain (rpm/rad) — softened from 30
        self.SideDeltaDeg = 15

        self.Timestep = dt

        # PID state
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

        # Helpers
        self.StopBand = 10.0       # mm
        self.I_Limit  = 200.0      # integral clamp
        self.ApproachSlope = 0.5   # rpm per mm
        self.MinApproachRPM = 6.0  # rpm

        # NEW: steering shaping
        self.SteerMix  = 0.5       # fraction of steer sent to wheels (0..1)
        self.SteerSlew = 6.0       # max steer change per cycle (rpm)
        self.prevSteerCmd = 0.0
        self.DiffCap   = 30.0      # max left/right difference (rpm)
        self.BaseCap   = 38.0      # cap forward base so steer has authority
        self.TurnSlowdown = 0.6    # scale base when turning hard

    def reset_pids(self):
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0
        self.prevSteerCmd = 0.0

    # ---------- forward (front-distance) PID -> base throttle ----------
    def forward_PID(self, desired_distance_mm=300):
        lidar = bot.get_range_image()
        win = [safe_distance(v) for v in lidar[175:185]] if isinstance(lidar, list) else []
        if not win:
            return 0.0

        measured = min(win)                        # mm
        error = measured - desired_distance_mm     # mm

        if abs(error) <= self.StopBand:
            self.ForwardIntegral = 0.0
            self.ForwardPrevError = 0.0
            return 0.0

        # PID
        self.ForwardIntegral += error * self.Timestep
        self.ForwardIntegral = max(-self.I_Limit, min(self.ForwardIntegral, self.I_Limit))
        d_err = (error - self.ForwardPrevError) / self.Timestep
        self.ForwardPrevError = error

        u = (self.K_p * error) + (self.K_i * self.ForwardIntegral) + (self.K_d * d_err)

        # soft approach
        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        # cap base so steer can still control heading
        u = math.copysign(min(abs(u), self.BaseCap), u)
        return saturation(u)

    # ---------- angle-aware side control (chosen wall-distance -> steering) ----------
    def side_PID(self, wall="left", desired_distance_mm=300):
        if wall not in ("left", "right"):
            return 0.0

        lidar = bot.get_range_image()
        if not isinstance(lidar, list) or len(lidar) < 300:
            return 0.0

        # Distance windows you asked for
        if wall == "left":
            side_near = [safe_distance(v) for v in lidar[85:95]]     # ~90°
            side_far  = [safe_distance(v) for v in lidar[100:110]]   # offset +Δ
        else:
            side_near = [safe_distance(v) for v in lidar[265:275]]   # ~270°
            side_far  = [safe_distance(v) for v in lidar[250:260]]   # offset -Δ

        d0 = min(side_near) if side_near else SENSOR_MAX_MM
        d1 = min(side_far)  if side_far  else SENSOR_MAX_MM

        if d0 >= NO_WALL_THRESH:
            return 0.0

        e_d = d0 - desired_distance_mm
        if abs(e_d) <= self.StopBand:
            self.SideIntegral = 0.0
            self.SidePrevError = 0.0
            e_d = 0.0

        # Local wall angle estimate
        Δ = math.radians(self.SideDeltaDeg)
        phi = math.atan2(d0 * math.cos(Δ) - d1, max(1e-6, d0 * math.sin(Δ)))
        if wall == "right":
            phi = -phi

        d_err = (e_d - self.SidePrevError) / self.Timestep
        self.SidePrevError = e_d

        u_dist = (self.Kp * e_d) + (self.Kd * d_err)
        u_angl = self.Kphi * phi

        u = u_dist + u_angl

        # raw steer cap (not final wheel diff)
        steer_cap = 0.8 * bot.max_motor_speed
        u = math.copysign(min(abs(u), steer_cap), u)

        # --- NEW: steer slew limiting ---
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
        angle_eps = math.radians(3.0)

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

        left_distance  = safe_distance(min(lidar[85:95]))     # mm
        right_distance = safe_distance(min(lidar[265:275]))   # mm
        print("LEFT D:", int(left_distance), "mm")
        print("RIGHT D:", int(right_distance), "mm")

        target_mm = 300

        # 1) Base forward speed from front controller (capped)
        linear_velocity = self.forward_PID(desired_distance_mm=target_mm)

        # Slow down base when steering is large (prevents nose-planting into walls)
        # We’ll scale after we compute the current steering.
        steer_raw = self.side_PID(wall=wall, desired_distance_mm=target_mm)
        steer_for_mix = self.SteerMix * steer_raw  # mixing gain

        # Optional base slowdown based on steer magnitude
        slow_factor = max(0.35, 1.0 - self.TurnSlowdown * (abs(steer_raw) / bot.max_motor_speed))
        linear_velocity *= slow_factor

        # default: straight
        rightv = leftv = linear_velocity

        # 2) If no wall on chosen side -> search arc (reduced base + small bias)
        side_distance = right_distance if wall == "right" else left_distance
        search_sign = +1 if wall == "right" else -1
        if side_distance >= NO_WALL_THRESH:
            base = 0.5 * linear_velocity
            bias = 0.20 * bot.max_motor_speed  # smaller bias than before
            rightv = base - search_sign * bias
            leftv  = base + search_sign * bias
            return saturation(rightv), saturation(leftv)

        # 3) Signed steering mix (with diff cap)
        if wall == "left":
            leftv  = linear_velocity - steer_for_mix
            rightv = linear_velocity + steer_for_mix
        else:
            leftv  = linear_velocity + steer_for_mix
            rightv = linear_velocity - steer_for_mix

        # Limit instantaneous wheel differential to avoid snap turns
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

    # --- Rotate gating to avoid false triggers during strong steering ---
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

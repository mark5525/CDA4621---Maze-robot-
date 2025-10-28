"""
HamBot Wall Following PID Controller (Real Robot Version, mm units)
Targets: front=300 mm, side=300 mm, max RPM=60
Adds angle-aware side PID and soft-finish rotate to prevent "dive & spin"
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
        self.Kp = 2.0   # distance-to-wall (rpm/mm)
        self.Ki = 0.0   # keep 0 to avoid windup from corners
        self.Kd = 5.0   # distance derivative (rpm/mm/s)

        self.Kphi = 30.0        # angle gain (rpm/rad) – small but stabilizing
        self.SideDeltaDeg = 15  # separation between side beams for angle estimate

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

    def reset_pids(self):
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

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

        return saturation(u)

    # ---------- angle-aware side control (chosen wall-distance -> steering) ----------
    def side_PID(self, wall="left", desired_distance_mm=300):
        if wall not in ("left", "right"):
            return 0.0

        lidar = bot.get_range_image()
        if not isinstance(lidar, list) or len(lidar) < 300:
            return 0.0

        # Core side windows (as you wanted): left 85:95, right 265:275
        if wall == "left":
            side_near = [safe_distance(v) for v in lidar[85:95]]     # around 90°
            side_far  = [safe_distance(v) for v in lidar[100:110]]   # ~90°+Δ
        else:
            side_near = [safe_distance(v) for v in lidar[265:275]]   # around 270°
            side_far  = [safe_distance(v) for v in lidar[250:260]]   # ~270°-Δ (behind-forward)

        d0 = min(side_near) if side_near else SENSOR_MAX_MM  # mm at ~90° or ~270°
        d1 = min(side_far)  if side_far  else SENSOR_MAX_MM  # mm at offset Δ

        # No wall -> let wall_follow() handle search arc
        if d0 >= NO_WALL_THRESH:
            return 0.0

        # Distance error
        e_d = d0 - desired_distance_mm

        # Small stop-band near setpoint so we don't chatter into the wall
        if abs(e_d) <= self.StopBand:
            self.SideIntegral = 0.0
            self.SidePrevError = 0.0
            e_d = 0.0

        # Estimate local wall angle (radians) using two beams separated by Δ
        # φ ≈ atan2(d0*cosΔ - d1, d0*sinΔ)
        Δ = math.radians(self.SideDeltaDeg)
        phi = math.atan2(d0 * math.cos(Δ) - d1, max(1e-6, d0 * math.sin(Δ)))

        # For the RIGHT wall, flip the angle sign so positive phi always means
        # "wall ahead is farther than wall at our side" (same control sense)
        if wall == "right":
            phi = -phi

        # Distance PD + angle P
        d_err = (e_d - self.SidePrevError) / self.Timestep
        self.SidePrevError = e_d

        u_dist = (self.Kp * e_d) + (self.Kd * d_err)
        u_angl = self.Kphi * phi

        u = u_dist + u_angl

        # Limit steering authority to keep things smooth
        steer_cap = 0.6 * bot.max_motor_speed
        u = math.copysign(min(abs(u), steer_cap), u)

        return saturation(u)

    # ---------- IMU-based rotate (radians; + = left/CCW) with soft finish ----------
    def rotate(self, radianAngle, base_rpm=18):
        self.reset_pids()
        left_sign  =  1 if radianAngle > 0 else -1
        right_sign = -left_sign

        def ang_delta(curr, ref):
            return (curr - ref + math.pi) % (2 * math.pi) - math.pi

        start = bot.get_heading()
        target = abs(radianAngle)
        angle_eps = math.radians(3.0)  # ~3° stop band

        while True:
            curr = bot.get_heading()
            delta = abs(ang_delta(curr, start))
            rem = target - delta
            if rem <= angle_eps:
                break

            # Slow down as we approach the target angle to avoid overshoot
            rpm = base_rpm
            if rem < math.radians(20):  # last ~20°
                rpm = max(6, base_rpm * 0.5)
            if rem < math.radians(10):  # last ~10°
                rpm = max(4, base_rpm * 0.3)

            bot.set_left_motor_speed( left_sign  * rpm)
            bot.set_right_motor_speed(right_sign * rpm)
            time.sleep(self.Timestep)

        # stop and settle
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

        target_mm = 300  # <-- side & front targets are both 300 mm

        # 1) Base forward speed from front controller
        linear_velocity = self.forward_PID(desired_distance_mm=target_mm)
        # Cap base so steering has authority
        linear_velocity = math.copysign(min(abs(linear_velocity), 40), linear_velocity)

        # 2) Steering from side controller (SIGNED)
        steer = self.side_PID(wall=wall, desired_distance_mm=target_mm)

        # default: go straight with base
        rightv = leftv = linear_velocity

        # 3) If no wall on chosen side -> search arc (reduced base + modest bias)
        search_sign = +1 if wall == "right" else -1
        side_distance = right_distance if wall == "right" else left_distance
        if side_distance >= NO_WALL_THRESH:
            base = 0.5 * linear_velocity
            bias = 0.25 * bot.max_motor_speed  # smaller bias to avoid big yaw
            rightv = base - search_sign * bias
            leftv  = base + search_sign * bias
            return saturation(rightv), saturation(leftv)

        # 4) Signed steering mix
        if wall == "left":
            leftv  = linear_velocity - steer
            rightv = linear_velocity + steer
        else:
            leftv  = linear_velocity + steer
            rightv = linear_velocity - steer

        return saturation(rightv), saturation(leftv)

# ========================
# Instantiate controller
# ========================
pp = Defintions()

# ========================
# Main loop (same style/flow as your reference)
# ========================
wall = "left"

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

    # Rotate if very close ahead; soft finish prevents overshoot/oscillation
    if front_distance < 450 and wall == "right":
        pp.rotate(-math.pi / 2)
    elif front_distance < 450 and wall == "left":
        pp.rotate(math.pi / 2)

    time.sleep(dt)

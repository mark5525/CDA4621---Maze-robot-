"""
HamBot Wall Following PID Controller (Real Robot Version, mm units)
Adapted to keep Class `Defintions`, mm throughout, and max RPM = 60
"""
import time
import math
# numpy not required for this mm-only version
from robot_systems.robot import HamBot  # same import style as your reference

# ========================
# HamBot Setup
# ========================
bot = HamBot(lidar_enabled=True, camera_enabled=False)
bot.max_motor_speed = 60  # keep max RPM at 60
dt = 0.032                # loop timestep (we'll also set pp.Timestep = dt)

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
    # Use robot’s max if not provided; max is 60 by requirement
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

        # Side PID gains (rpm per mm)
        self.Kp = 2
        self.Ki = 0
        self.Kd = 5

        self.Timestep = 0.025  # will be overwritten to dt below

        # Separate PID states (forward & side)
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

        # Helpers
        self.StopBand = 10.0       # mm
        self.I_Limit  = 200.0      # integral clamp
        self.ApproachSlope = 0.5   # rpm per mm
        self.MinApproachRPM = 6.0  # rpm

    # ---------- helpers ----------
    def reset_pids(self):
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

    # ---------- forward (front-distance) PID -> base throttle ----------
    def forward_PID(self, desired_distance_mm=400):
        lidar = bot.get_range_image()
        win = [safe_distance(v) for v in lidar[175:185]] if isinstance(lidar, list) else []
        if not win:
            return 0.0

        measured = min(win)                 # mm
        error = measured - desired_distance_mm  # mm

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

    # ---------- side (chosen wall-distance) PID -> steering ----------
    def side_PID(self, wall="left", desired_distance_mm=400):
        if wall not in ("left", "right"):
            return 0.0

        lidar = bot.get_range_image()
        if not isinstance(lidar, list):
            return 0.0

        left_vals  = [safe_distance(v) for v in lidar[90:115]]
        right_vals = [safe_distance(v) for v in lidar[265:290]]
        actual_left  = min(left_vals)  if left_vals  else SENSOR_MAX_MM
        actual_right = min(right_vals) if right_vals else SENSOR_MAX_MM

        # if no wall on the chosen side, return 0; search handled in wall_follow
        if wall == "left" and actual_left >= NO_WALL_THRESH:
            return 0.0
        if wall == "right" and actual_right >= NO_WALL_THRESH:
            return 0.0

        side_meas = actual_right if wall == "right" else actual_left
        error = side_meas - desired_distance_mm  # mm

        if abs(error) <= self.StopBand:
            self.SideIntegral = 0.0
            self.SidePrevError = 0.0
            return 0.0

        # PID
        self.SideIntegral += error * self.Timestep
        self.SideIntegral = max(-self.I_Limit, min(self.SideIntegral, self.I_Limit))
        d_err = (error - self.SidePrevError) / self.Timestep
        self.SidePrevError = error

        u = (self.Kp * error) + (self.Ki * self.SideIntegral) + (self.Kd * d_err)

        # limit steering authority
        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        return saturation(u)

    # ---------- IMU-based rotate (radians; + = left/CCW) ----------
    def rotate(self, radianAngle, base_rpm=18):
        self.reset_pids()
        left_sign  =  1 if radianAngle > 0 else -1
        right_sign = -left_sign

        def ang_delta(curr, ref):
            return (curr - ref + math.pi) % (2 * math.pi) - math.pi

        start = bot.get_heading()
        while True:
            curr = bot.get_heading()
            if abs(ang_delta(curr, start)) >= abs(radianAngle):
                bot.set_left_motor_speed(0)
                bot.set_right_motor_speed(0)
                break
            bot.set_left_motor_speed( left_sign  * base_rpm)
            bot.set_right_motor_speed(right_sign * base_rpm)
            time.sleep(self.Timestep)

        self.reset_pids()
        time.sleep(self.Timestep)

    # ---------- high-level: wall following (returns right_rpm, left_rpm) ----------
    def wall_follow(self, wall="left"):
        lidar = bot.get_range_image()
        if not isinstance(lidar, list) or len(lidar) < 290:
            return 0.0, 0.0

        left_distance  = safe_distance(min(lidar[90:115]))    # mm
        right_distance = safe_distance(min(lidar[265:290]))   # mm
        print("LEFT D:", int(left_distance), "mm")
        print("RIGHT D:", int(right_distance), "mm")

        target_mm = 400

        linear_velocity  = self.forward_PID(desired_distance_mm=target_mm)
        angular_velocity = self.side_PID(wall=wall, desired_distance_mm=target_mm)

        rightv = leftv = linear_velocity

        search_sign = +1 if wall == "right" else -1
        side_distance = right_distance if wall == "right" else left_distance

        if side_distance >= NO_WALL_THRESH:
            # No wall detected → gentle arc
            base = 0.6 * linear_velocity
            bias = 0.4 * bot.max_motor_speed  # fixed bias fraction of max
            rightv = base - search_sign * bias
            leftv  = base + search_sign * bias
        else:
            # Wall-following adjustment (mirrors your reference logic)
            if wall == "right":
                if right_distance < target_mm:
                    rightv = linear_velocity + abs(angular_velocity)
                    leftv  = linear_velocity - abs(angular_velocity)
                elif right_distance < 2000:  # 2.0 m in mm
                    rightv = linear_velocity - abs(angular_velocity)
                    leftv  = linear_velocity + abs(angular_velocity)
            else:
                if left_distance < target_mm:
                    rightv = linear_velocity - abs(angular_velocity)
                    leftv  = linear_velocity + abs(angular_velocity)
                elif target_mm < left_distance < 2000:
                    rightv = linear_velocity + abs(angular_velocity)
                    leftv  = linear_velocity - abs(angular_velocity)

        return saturation(rightv), saturation(leftv)

# ========================
# Instantiate controller and sync timestep
# ========================
pp = Defintions()
pp.Timestep = dt  # match the sample's dt in the class as well

# ========================
# Main loop (kept same flow/style as your reference)
# ========================
wall = "left"

while True:

    lidar = bot.get_range_image()
    if isinstance(lidar, list) and len(lidar) >= 360:
        center_idx = 180  # front center index
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

    # rotate at ~0.45 m = 450 mm
    if front_distance < 450 and wall == "right":
        pp.rotate(-math.pi / 2)
    elif front_distance < 450 and wall == "left":
        pp.rotate(math.pi / 2)

    time.sleep(dt)


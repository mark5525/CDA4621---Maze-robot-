from HamBot.src.robot_systems.robot import HamBot
import time, math

# ========================
# Constants (mm / rpm)
# ========================
SENSOR_MAX_MM   = 9500   # fallback for "no wall" / out of range
NO_WALL_THRESH  = 2500   # >= this on chosen side => treat as no wall (search mode)

# ========================
# Utilities
# ========================
def safe_distance_mm(value_mm, max_range_mm=SENSOR_MAX_MM):
    """Clamp bad/missing LiDAR values to a safe 'far away' distance in mm."""
    try:
        v = float(value_mm)
        if math.isnan(v) or math.isinf(v) or v <= 0:
            return max_range_mm
        return min(v, max_range_mm)
    except:
        return max_range_mm

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

# ========================
# Controller
# ========================
class Defintions():
    def __init__(self):
        # Forward-distance PID gains (rpm per mm)
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5

        # Side-distance (steering) PID gains (rpm per mm)
        self.Kp = 2
        self.Ki = 0
        self.Kd = 5

        self.Timestep = 0.025  # s

        # Legacy fields (kept for compatibility)
        self.Integral = 0.0
        self.Derivative = 0.0
        self.PrevError = 0.0

        # Separate state for forward and side loops (prevents cross-talk)
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

        # Approach/anti-windup helpers
        self.StopBand = 10.0      # mm
        self.I_Limit  = 200.0     # integral clamp (arbitrary rpm·s scale)
        self.ApproachSlope = 0.5  # rpm per mm (caps speed as you approach target)
        self.MinApproachRPM = 6.0 # minimum rpm far from target

    # -------- Forward PID (front distance -> base throttle rpm) --------
    def forward_PID(self, bot, desired_distance_mm=400):
        scan = bot.get_range_image()
        win = [safe_distance_mm(a) for a in scan[175:185]] if isinstance(scan, list) else []
        if not win:
            return 0.0

        measured = min(win)  # mm
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

        # Soft-approach cap
        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u)

    # -------- Side PID (chosen wall distance -> steering rpm) --------
    def side_PID(self, bot, s_follow="left", desired_distance_mm=300):
        if s_follow not in ("left", "right"):
            return 0.0

        scan = bot.get_range_image()
        if not isinstance(scan, list):
            return 0.0

        left_win  = [safe_distance_mm(a) for a in scan[90:115]]
        right_win = [safe_distance_mm(a) for a in scan[265:290]]
        actual_left  = min(left_win)  if left_win  else SENSOR_MAX_MM
        actual_right = min(right_win) if right_win else SENSOR_MAX_MM

        # If no wall on chosen side, return 0 here; wall_follow will handle search arc.
        if s_follow == "left"  and actual_left  >= NO_WALL_THRESH: return 0.0
        if s_follow == "right" and actual_right >= NO_WALL_THRESH: return 0.0

        side_meas = actual_right if s_follow == "right" else actual_left
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

        # Limit steering authority in proportion to error (keeps turns smooth)
        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u)

    # -------- IMU-based rotate (radians, positive = left/CCW) --------
    def rotate(self, bot, radianAngle, base_rpm=18):
        self.reset_pids()

        left_sign  =  1 if radianAngle > 0 else -1
        right_sign = -left_sign

        def ang_delta(curr, ref):
            # Wrap to [-pi, pi]
            return (curr - ref + math.pi) % (2 * math.pi) - math.pi

        start = bot.get_heading()  # radians
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

    def reset_pids(self):
        self.ForwardIntegral = 0.0
        self.ForwardPrevError = 0.0
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

    # -------- High level: wall following (returns right_cmd, left_cmd) --------
    def wall_follow(self, bot, wall="left", side_target_mm=300, front_target_mm=400, cruise_cap_rpm=40):
        scan = bot.get_range_image()
        if not isinstance(scan, list):
            return 0.0, 0.0

        left_win  = [safe_distance_mm(a) for a in scan[90:115]]
        right_win = [safe_distance_mm(a) for a in scan[265:290]]
        actual_left  = min(left_win)  if left_win  else SENSOR_MAX_MM
        actual_right = min(right_win) if right_win else SENSOR_MAX_MM

        # 1) Base (forward) speed from front controller
        base = self.forward_PID(bot, desired_distance_mm=front_target_mm)
        base = math.copysign(min(abs(base), cruise_cap_rpm), base)  # keep turns authoritative

        # 2) Steering from side controller
        steer = self.side_PID(bot, wall, desired_distance_mm=side_target_mm)

        # 3) If no wall on chosen side -> search arc (reduced base + fixed bias)
        side_dist = actual_right if wall == "right" else actual_left
        search_sign = +1 if wall == "right" else -1
        if side_dist >= NO_WALL_THRESH:
            base_arc = 0.6 * base
            bias_rpm = 0.4 * cruise_cap_rpm
            rightv = saturation(bot, base_arc - search_sign * bias_rpm)
            leftv  = saturation(bot, base_arc + search_sign * bias_rpm)
            return rightv, leftv

        # 4) Normal mixing (use |steer| for intuitive bias)
        if wall == "left":
            leftv  = saturation(bot, base - abs(steer))
            rightv = saturation(bot, base + abs(steer))
        else:
            leftv  = saturation(bot, base + abs(steer))
            rightv = saturation(bot, base - abs(steer))

        return rightv, leftv

# ========================
# Main
# ========================
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    wall = "left"          # or "right"
    side_goal_mm  = 300    # desired side distance (mm)
    front_goal_mm = 400    # desired front distance (mm)
    rotate_thresh = 450    # rotate if front < this (mm)

    pp = Defintions()

    try:
        while True:
            lidar = Bot.get_range_image()

            # Front distance (mm)
            if isinstance(lidar, list) and len(lidar) >= 360:
                front_d = min(safe_distance_mm(a) for a in lidar[175:185])
                print(f"Front distance: {front_d:.0f} mm")
            else:
                print("No LiDAR data received")
                front_d = SENSOR_MAX_MM

            # Wall-following command (rpm)
            right_cmd, left_cmd = pp.wall_follow(
                Bot,
                wall=wall,
                side_target_mm=side_goal_mm,
                front_target_mm=front_goal_mm,
                cruise_cap_rpm=40
            )

            # Corner handling with IMU rotate
            if front_d < rotate_thresh:
                pp.rotate(Bot, (-math.pi/2) if wall == "right" else (math.pi/2))
                time.sleep(pp.Timestep)
                continue

            # Send motor commands at end of loop
            Bot.set_left_motor_speed(left_cmd)
            Bot.set_right_motor_speed(right_cmd)

            print(f"L:{left_cmd:.1f}  R:{right_cmd:.1f}  (wall={wall})")
            print("-" * 50)
            time.sleep(pp.Timestep)

    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)


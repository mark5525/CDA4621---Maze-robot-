from HamBot.src.robot_systems.robot import HamBot
import time, math

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class Defintions():
    def __init__(self):
        # Forward PID (kept)
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5

        # Side PID gains (kept)
        self.Kp = 2
        self.Ki = 0
        self.Kd = 5

        self.Timestep = 0.025

        # Forward PID state (kept)
        self.Integral = 0.0
        self.Derivative = 0.0
        self.PrevError = 0.0

        # --- NEW: Side PID state (separate from forward to avoid cross-talk)
        self.S_Integral = 0.0
        self.S_PrevError = 0.0

        # gentle-stop helpers (kept)
        self.StopBand = 10.0           # mm
        self.I_Limit  = 200.0
        self.ApproachSlope = 0.5
        self.MinApproachRPM = 6.0

        # --- Small driving defaults (used in main and rotate)
        self.cruise_rpm = 20.0
        self.steer_limit = 30.0
        self.front_turn_thresh_mm = 260.0
        self.open_wrap_extra_mm = 140.0

    def forward_PID(self, bot, desired_distance):
        scan = bot.get_range_image()
        window = [a for a in scan[175:180] if a and a > 0]
        if not window:
            return 0.0

        measured_distance = min(window)
        error = measured_distance - desired_distance

        if abs(error) <= self.StopBand:
            self.Integral = 0.0
            self.PrevError = 0.0
            return 0.0

        self.Integral += error * self.Timestep
        self.Integral = max(-self.I_Limit, min(self.Integral, self.I_Limit))

        derivative = (error - self.PrevError) / self.Timestep
        self.PrevError = error

        u = (self.K_p * error) + (self.K_i * self.Integral) + (self.K_d * derivative)

        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)
        return saturation(bot, u)

    def side_PID(self, bot, s_follow, desired_distance):
        """
        Returns a steering 'yaw' (RPM to add/subtract around a base/cruise speed).
        +yaw means CCW turn (more right-wheel, less left-wheel).
        """
        scan = bot.get_range_image()

        # --- FIX: look at the correct side window
        if s_follow == "left":
            side_vals = [a for a in scan[85:105] if a and a > 0]     # 90° ±10°
        else:
            side_vals = [a for a in scan[265:285] if a and a > 0]    # 270° ±10°

        if not side_vals:
            # No side reading: return a gentle bias toward the followed side
            return 0.0

        sideActual = min(side_vals)
        error = sideActual - desired_distance  # + if too far from the wall

        # --- PID with side-specific state
        P = self.Kp * error

        self.S_Integral += error * self.Timestep
        self.S_Integral = max(-self.I_Limit, min(self.S_Integral, self.I_Limit))
        I = self.Ki * self.S_Integral

        d_err = (error - self.S_PrevError) / self.Timestep
        self.S_PrevError = error
        D = self.Kd * d_err

        yaw = P + I + D

        # If following the right wall, flip sign so +error turns CW toward that wall
        if s_follow == "right":
            yaw = -yaw

        # Limit steering authority; do NOT clamp to motor max
        yaw = max(-self.steer_limit, min(self.steer_limit, yaw))
        return yaw

    def rotate(self, bot, direction="left", desired_distance=300, s_follow="left"):
        """
        Lidar-only ~90° in-place rotation.
        Spins until: (a) the new side looks like a valid wall near desired_distance
                      and (b) front is reasonably clear.
        Falls back to a time cap to avoid deadlock in weird geometry.
        """
        base_rpm = 25.0
        spin = 1 if direction == "left" else -1

        # Helper to read front/left/right quickly
        def _read_flr():
            sc = bot.get_range_image()
            f = min([a for a in sc[175:185] if a and a > 0] or [float("inf")])
            l = min([a for a in sc[85:105]  if a and a > 0] or [float("inf")])
            r = min([a for a in sc[265:285] if a and a > 0] or [float("inf")])
            return f, l, r

        t0 = time.time()
        timeout_s = 2.0   # ~ good for ~90° at 25 rpm; adjust if needed

        # Start spin
        bot.set_left_motor_speed(-spin * base_rpm)
        bot.set_right_motor_speed( spin * base_rpm)

        while True:
            front, left, right = _read_flr()
            side = left if s_follow == "left" else right

            side_ok  = (abs(side - desired_distance) <= max(1.5*self.StopBand, 20.0))
            front_ok = (front > desired_distance + 120.0)  # reasonably clear ahead

            if side_ok and front_ok:
                break

            if time.time() - t0 > timeout_s:
                break

            time.sleep(self.Timestep)

        bot.stop_motors()
        time.sleep(0.05)  # settle

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    wall_follow = "left"   # "left" or "right"
    d_distance  = 300
    pp = Defintions()

    while True:
        scan = Bot.get_range_image()
        forward_distance = min([a for a in scan[175:185] if a and a > 0] or [float("inf")])

        # steering from side PID
        yaw = pp.side_PID(Bot, wall_follow, d_distance)

        # --- minimal changes to your structure: keep the ifs, just mix yaw around cruise
        if wall_follow == "left":
            left_cmd  = saturation(Bot, pp.cruise_rpm - yaw)
            right_cmd = saturation(Bot, pp.cruise_rpm + yaw)
            Bot.set_left_motor_speed(left_cmd)
            Bot.set_right_motor_speed(right_cmd)

        if wall_follow == "right":
            # flip mixing for right-follow
            left_cmd  = saturation(Bot, pp.cruise_rpm + yaw)
            right_cmd = saturation(Bot, pp.cruise_rpm - yaw)
            Bot.set_left_motor_speed(left_cmd)
            Bot.set_right_motor_speed(right_cmd)

        print(f"yaw={yaw:.1f}  front={forward_distance:.0f}")

        # --- simple, lidar-only turning logic that mirrors standard behavior
        # 1) Frontal obstacle -> rotate 90° away from the followed wall
        if forward_distance < pp.front_turn_thresh_mm:
            turn_dir = "right" if wall_follow == "left" else "left"
            pp.rotate(Bot, direction=turn_dir, desired_distance=d_distance, s_follow=wall_follow)
            # reset side PID state after a discrete turn
            pp.S_Integral = 0.0
            pp.S_PrevError = 0.0

        # 2) Opening on followed side -> wrap 90° toward the followed wall
        else:
            # grab side distance for opening check
            if wall_follow == "left":
                side_vals = [a for a in scan[85:105] if a and a > 0]
            else:
                side_vals = [a for a in scan[265:285] if a and a > 0]
            side_dist = min(side_vals) if side_vals else float("inf")

            if (math.isinf(side_dist)) or (side_dist > d_distance + pp.open_wrap_extra_mm):
                turn_dir = "left" if wall_follow == "left" else "right"
                pp.rotate(Bot, direction=turn_dir, desired_distance=d_distance, s_follow=wall_follow)
                pp.S_Integral = 0.0
                pp.S_PrevError = 0.0

        time.sleep(pp.Timestep)

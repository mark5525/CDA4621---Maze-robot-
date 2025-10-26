from HamBot.src.robot_systems.robot import HamBot
import time, math

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

def _ang_norm_deg(a):
    # normalize to (-180, 180]
    a = (a + 180.0) % 360.0 - 180.0
    return a

class Defintions():
    def __init__(self):
        # Forward “approach” PID (you already had these)
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5

        # Side wall-follow PID
        self.Kp = 2.0
        self.Ki = 0.0
        self.Kd = 5.0

        # Yaw (heading) PD for clean 90° rotations
        self.K_yaw_p = 0.9
        self.K_yaw_d = 0.2

        self.Timestep = 0.025

        # forward PID state
        self.Integral = 0.0
        self.PrevError = 0.0

        # side PID state
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

        # NEW: gentle-stop helpers (forward only)
        self.StopBand = 10.0           # mm (tune 8–15)
        self.I_Limit  = 200.0          # integral clamp
        self.ApproachSlope = 0.5       # rpm per mm (speed cap shrinks near target)
        self.MinApproachRPM = 6.0      # don't crawl too slowly far out

    # ---------- SENSING HELPERS ----------
    @staticmethod
    def _min_valid(scan_slice):
        vals = [d for d in scan_slice if d and d > 0]
        return min(vals) if vals else float("inf")

    def read_distances(self, bot):
        scan = bot.get_range_image()
        # Lidar convention (your note): 0° back, 90° left, 180° front, 270° right
        front = self._min_valid(scan[170:190])
        left  = self._min_valid(scan[ 85:105])
        right = self._min_valid(scan[265:285])
        return front, left, right

    # ---------- FORWARD (kept from Task 1, used here only if you want approach control) ----------
    def forward_PID(self, bot, desired_distance):
        scan = bot.get_range_image()
        window = [a for a in scan[175:180] if a and a > 0]
        if not window:
            return 0.0

        measured_distance = min(window)
        error = measured_distance - desired_distance

        # clean stop when close, and prevent re-accel
        if abs(error) <= self.StopBand:
            self.Integral = 0.0
            self.PrevError = 0.0
            return 0.0

        # PID terms
        self.Integral += error * self.Timestep
        self.Integral = max(-self.I_Limit, min(self.Integral, self.I_Limit))
        derivative = (error - self.PrevError) / self.Timestep
        self.PrevError = error

        u = (self.K_p * error) + (self.K_i * self.Integral) + (self.K_d * derivative)

        # soft approach – limit RPM based on how close you are
        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u)

    # ---------- SIDE WALL-FOLLOW PID ----------
    def side_PID(self, bot, side_follow: str, desired_distance_mm: float):
        """
        Returns a steering 'yaw' term in RPM to be *added* to right wheel and *subtracted* from left wheel.
        Positive yaw => CCW turn. Sign is chosen so 'too far from the wall' steers back toward it.
        """
        front, left, right = self.read_distances(bot)
        measured = left if side_follow == "left" else right

        # error is positive when we're too far from the wall
        err = (measured - desired_distance_mm)

        # PID with separate side integral
        self.SideIntegral += err * self.Timestep
        self.SideIntegral = max(-300.0, min(self.SideIntegral, 300.0))
        d_err = (err - self.SidePrevError) / self.Timestep
        self.SidePrevError = err

        yaw = (self.Kp * err) + (self.Ki * self.SideIntegral) + (self.Kd * d_err)

        # map 'too far' => turn toward that wall
        #   left-follow: positive err (too far) => CCW => +yaw
        #   right-follow: positive err (too far) => CW  => -yaw
        yaw = yaw if side_follow == "left" else -yaw

        # limit steer authority
        yaw = max(-30.0, min(30.0, yaw))
        return yaw

    # ---------- 90° ROTATIONS USING HEADING ----------
    def rotate_relative_deg(self, bot, delta_deg, base_spin_rpm=30, tol_deg=2.0, max_time=3.0):
        """
        Closed-loop 90° (or any) in-place rotation using heading PD.
        """
        if not hasattr(bot, "get_heading"):
            # fallback: open-loop spin if IMU not available
            sign = 1.0 if delta_deg > 0 else -1.0
            bot.set_left_motor_speed(-sign * base_spin_rpm)
            bot.set_right_motor_speed( sign * base_spin_rpm)
            time.sleep(abs(delta_deg) / 90.0 * 1.0)  # ~1s per 90°
            bot.stop_motors()
            return

        start = bot.get_heading()
        target = (start + delta_deg) % 360.0
        prev_err = _ang_norm_deg(target - bot.get_heading())
        t0 = time.time()

        while True:
            now_h = bot.get_heading()
            err = _ang_norm_deg(target - now_h)
            d_err = (err - prev_err) / self.Timestep
            prev_err = err

            u = self.K_yaw_p * err + self.K_yaw_d * d_err
            u = max(-base_spin_rpm, min(base_spin_rpm, u))

            # CCW for +u
            bot.set_left_motor_speed(-u)
            bot.set_right_motor_speed(+u)

            if abs(err) <= tol_deg:  # finished
                break
            if time.time() - t0 > max_time:  # give up safety
                break
            time.sleep(self.Timestep)

        bot.stop_motors()
        # small settle
        time.sleep(0.05)

    # ---------- HIGH-LEVEL WALL FOLLOW ----------
    def follow_wall_loop(self, bot, side_follow="left",
                         side_distance_mm=300.0,
                         cruise_rpm=22.0,
                         front_turn_thresh_mm=260.0,
                         open_wrap_extra_mm=140.0):
        """
        Main loop: follow a wall; on front block => turn 90° away from obstacle;
        on opening at the followed side => wrap 90° into the opening and continue.
        """
        assert side_follow in ("left", "right")
        wrap_sign = +1 if side_follow == "left" else -1   # +90 for left-follow, -90 for right-follow
        away_sign = -wrap_sign                             # opposite direction when front is blocked

        while True:
            front, left, right = self.read_distances(bot)
            side_meas = left if side_follow == "left" else right

            # --- 1) Frontal obstacle => rotate 90° away and continue
            if front < front_turn_thresh_mm:
                bot.stop_motors()
                self.rotate_relative_deg(bot, away_sign * 90.0)
                # reset side PID state after discrete turns
                self.SideIntegral = 0.0
                self.SidePrevError = 0.0
                continue

            # --- 2) Wall ends (opening) => wrap 90° toward the chosen wall
            if side_meas > (side_distance_mm + open_wrap_extra_mm) or math.isinf(side_meas):
                bot.stop_motors()
                self.rotate_relative_deg(bot, wrap_sign * 90.0)
                self.SideIntegral = 0.0
                self.SidePrevError = 0.0
                continue

            # --- 3) Regular wall follow: drive forward with side PID steering
            yaw = self.side_PID(bot, side_follow, side_distance_mm)

            left_cmd  = saturation(bot, cruise_rpm - yaw)
            right_cmd = saturation(bot, cruise_rpm + yaw)

            bot.set_left_motor_speed(left_cmd)
            bot.set_right_motor_speed(right_cmd)
            time.sleep(self.Timestep)

# -------------------- RUNNERS --------------------
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    # Choose the wall to hug and the target standoff distance
    # Run once with "left" (maze1.xml), then again with "right".
    wall_follow_mode = "left"      # "left" or "right"
    side_standoff_mm = 300.0       # desired distance to wall
    cruise_rpm       = 22.0        # forward cruise (adjust per maze)

    ctrl = Defintions()
    try:
        ctrl.follow_wall_loop(
            Bot,
            side_follow=wall_follow_mode,
            side_distance_mm=side_standoff_mm,
            cruise_rpm=cruise_rpm,
            front_turn_thresh_mm=260.0,   # turn away when closer than this
            open_wrap_extra_mm=140.0      # "opening" if side > standoff + extra
        )
    finally:
        Bot.stop_motors()




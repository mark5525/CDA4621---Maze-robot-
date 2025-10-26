from HamBot.src.robot_systems.robot import HamBot
import time, math

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

def _ang_norm_deg(a):
    return (a + 180.0) % 360.0 - 180.0

class Defintions():
    def __init__(self):
        # Forward PID (kept)
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5

        # Side PID
        self.Kp = 2
        self.Ki = 0
        self.Kd = 5

        self.Timestep = 0.025
        self.Integral = 0.0
        self.PrevError = 0.0

        # Separate state for side PID (NEW, necessary)
        self.SideIntegral = 0.0
        self.SidePrevError = 0.0

        # gentle-stop helpers (kept)
        self.StopBand = 10.0
        self.I_Limit  = 200.0
        self.ApproachSlope = 0.5
        self.MinApproachRPM = 6.0

        # Wall-follow params (NEW)
        self.cruise_rpm = 22.0          # base forward speed
        self.yaw_limit  = 30.0          # max steering authority (rpm)
        self.front_turn_thresh_mm = 260 # if front < this, 90° away
        self.open_wrap_extra_mm   = 140 # if side > desired + extra, 90° toward

        # Turn PD (for rotate)
        self.K_yaw_p = 1.0
        self.K_yaw_d = 0.25

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
        Returns steering 'yaw' in RPM. +yaw means CCW (left) turn authority.
        """
        scan = bot.get_range_image()
        if s_follow == "left":
            side_vals = [a for a in scan[85:105] if a and a > 0]
        else:
            side_vals = [a for a in scan[265:285] if a and a > 0]

        if not side_vals:
            return 0.0  # no side reading; main loop will handle wrap

        sideActual = min(side_vals)
        error = sideActual - desired_distance  # + if too far from wall

        # side PID (separate state)
        self.SideIntegral += error * self.Timestep
        self.SideIntegral = max(-300.0, min(self.SideIntegral, 300.0))
        d_err = (error - self.SidePrevError) / self.Timestep
        self.SidePrevError = error

        yaw = (self.Kp * error) + (self.Ki * self.SideIntegral) + (self.Kd * d_err)

        # If following right wall, flip sign so +error (too far) turns toward the right
        if s_follow == "right":
            yaw = -yaw

        # limit steering authority (do NOT clamp to max motor speed)
        yaw = max(-self.yaw_limit, min(self.yaw_limit, yaw))
        return yaw

    def rotate(self, direction="left", angle_deg=90, bot=None, timeout=3.0):
        """
        In-place rotation. direction: 'left' (CCW) or 'right' (CW).
        Uses IMU heading if available; else timed open-loop spin.
        """
        b = bot if bot is not None else globals().get("Bot")
        if b is None:
            return

        sign = 1.0 if direction == "left" else -1.0
        base_rpm = 30.0

        if hasattr(b, "get_heading") and callable(b.get_heading):
            start = b.get_heading()
            target = (start + sign * angle_deg) % 360.0
            prev_err = _ang_norm_deg(target - b.get_heading())
            t0 = time.time()
            while True:
                now_h = b.get_heading()
                err = _ang_norm_deg(target - now_h)
                d_err = (err - prev_err) / self.Timestep
                prev_err = err

                u = self.K_yaw_p * err + self.K_yaw_d * d_err
                u = max(-base_rpm, min(base_rpm, u))

                b.set_left_motor_speed(-u)
                b.set_right_motor_speed(+u)

                if abs(err) <= 2.0:
                    break
                if time.time() - t0 > timeout:
                    break
                time.sleep(self.Timestep)
            b.stop_motors()
            time.sleep(0.05)
        else:
            # fallback timed spin
            b.set_left_motor_speed(-sign * base_rpm)
            b.set_right_motor_speed( sign * base_rpm)
            time.sleep(abs(angle_deg) / 90.0)  # ~1s per 90°
            b.stop_motors()
            time.sleep(0.05)

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    wall_follow = "left"  # "left" or "right"
    d_distance  = 300     # desired side standoff (mm)
    pp = Defintions()

    # map turn directions once (keeps your if-structure, minimal edits)
    if wall_follow == "left":
        turn_away = "right"  # on front block, turn away from followed wall
        wrap_dir  = "left"   # when wall ends, wrap toward followed wall
    if wall_follow == "right":
        turn_away = "left"
        wrap_dir  = "right"

    while True:
        scan = Bot.get_range_image()
        forward_distance = min([a for a in scan[175:185] if a and a > 0] or [float("inf")])

        # side distance for opening detection
        if wall_follow == "left":
            side_vals = [a for a in scan[85:105] if a and a > 0]
        else:
            side_vals = [a for a in scan[265:285] if a and a > 0]
        side_distance = min(side_vals) if side_vals else float("inf")

        # 1) frontal obstacle -> 90° away, continue
        if forward_distance < pp.front_turn_thresh_mm:
            Bot.stop_motors()
            pp.rotate(direction=turn_away, bot=Bot)
            pp.SideIntegral = 0.0
            pp.SidePrevError = 0.0
            continue

        # 2) wall ends (opening) -> 90° toward the followed wall
        if (math.isinf(side_distance)) or (side_distance > d_distance + pp.open_wrap_extra_mm):
            Bot.stop_motors()
            pp.rotate(direction=wrap_dir, bot=Bot)
            pp.SideIntegral = 0.0
            pp.SidePrevError = 0.0
            continue

        # 3) regular wall-follow: cruise +/- yaw
        yaw = pp.side_PID(Bot, wall_follow, d_distance)
        left_cmd  = saturation(Bot, pp.cruise_rpm - yaw)
        right_cmd = saturation(Bot, pp.cruise_rpm + yaw)
        Bot.set_left_motor_speed(left_cmd)
        Bot.set_right_motor_speed(right_cmd)

        print(f"yaw={yaw:.1f}  front={forward_distance:.0f}  side={side_distance:.0f}")
        time.sleep(pp.Timestep)




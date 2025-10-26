from HamBot.src.robot_systems.robot import HamBot
import time, math

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class Defintions():
    def __init__(self):
        # Forward PID (yours)
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5

        # Side PID gains (yours)
        self.Kp = 2
        self.Ki = 0
        self.Kd = 5

        self.Timestep = 0.025

        # Forward PID state (yours)
        self.Integral = 0.0
        self.Derivative = 0.0
        self.PrevError = 0.0

        # --- Side PID state (separate; needed)
        self.S_Integral = 0.0
        self.S_PrevError = 0.0

        # Helpers (yours)
        self.StopBand = 10.0
        self.I_Limit  = 200.0
        self.ApproachSlope = 0.5
        self.MinApproachRPM = 6.0

        # Simple driving constants
        self.base_rpm = 20.0          # cruise/base speed
        self.steer_limit = 30.0       # max correction from side PID
        self.front_turn_thresh_mm = 260.0  # turn when front is closer than this

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
        Basic PID on side distance. Returns a small correction 'u'.
        We'll apply it directly to wheel speeds in the main loop.
        """
        scan = bot.get_range_image()

        # Use the correct side window
        if s_follow == "left":
            side_vals = [a for a in scan[85:105] if a and a > 0]     # ~90° ±10°
        else:
            side_vals = [a for a in scan[265:285] if a and a > 0]    # ~270° ±10°

        if not side_vals:
            return 0.0

        sideActual = min(side_vals)
        error = sideActual - desired_distance  # + => too far from wall; - => too close

        # PID (separate state from forward)
        P = self.Kp * error

        self.S_Integral += error * self.Timestep
        self.S_Integral = max(-self.I_Limit, min(self.S_Integral, self.I_Limit))
        I = self.Ki * self.S_Integral

        d_err = (error - self.S_PrevError) / self.Timestep
        self.S_PrevError = error
        D = self.Kd * d_err

        u = P + I + D

        # Keep correction modest
        if u >  self.steer_limit: u =  self.steer_limit
        if u < -self.steer_limit: u = -self.steer_limit
        return u  # keep it raw; we'll mix it into wheels outside

    def rotate(self, direction=None, bot=None, desired_distance=None, s_follow=None):
        """
        Lidar-only ~90° rotate.
        Spins until the new side is near desired_distance and the front is clear,
        or a short timeout elapses. If args are None, uses globals to keep your call style.
        """
        b = bot if bot is not None else globals().get("Bot")
        if b is None:
            return

        # Defaults from globals to preserve 'pp.rotate()' in your loop
        follow = s_follow if s_follow is not None else globals().get("wall_follow", "left")
        d_des  = desired_distance if desired_distance is not None else globals().get("d_distance", 300)

        # If not provided, turn AWAY from followed wall on a frontal block
        if direction is None:
            direction = "right" if follow == "left" else "left"

        spin = 1 if direction == "left" else -1
        base_rpm = 25.0
        timeout_s = 2.0
        t0 = time.time()

        # Start in-place spin
        b.set_left_motor_speed(-spin * base_rpm)
        b.set_right_motor_speed( spin * base_rpm)

        # helper to read front/side
        def _read_front_side(follow_side):
            sc = b.get_range_image()
            front = min([a for a in sc[175:185] if a and a > 0] or [float("inf")])
            if follow_side == "left":
                side = min([a for a in sc[85:105] if a and a > 0] or [float("inf")])
            else:
                side = min([a for a in sc[265:285] if a and a > 0] or [float("inf")])
            return front, side

        while True:
            f, s = _read_front_side(follow)
            side_ok  = abs(s - d_des) <= max(1.5*self.StopBand, 20.0)
            front_ok = f > (d_des + 120.0)

            if side_ok and front_ok:
                break
            if time.time() - t0 > timeout_s:
                break
            time.sleep(self.Timestep)

        b.stop_motors()
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

        # side PID correction (basic PID, no "yaw" concept)
        u = pp.side_PID(Bot, wall_follow, d_distance)
        print("u=", round(u,1), "front=", round(forward_distance if forward_distance!=float('inf') else -1, 0))

        # --- keep your simple if-structure; just let the wheel speeds "correct themselves"
        if wall_follow == "left":
            # too far (u>0) => slow left / speed up right to drift left toward wall
            Bot.set_left_motor_speed( saturation(Bot, pp.base_rpm - u) )
            Bot.set_right_motor_speed( saturation(Bot, pp.base_rpm + u) )

        if wall_follow == "right":
            # too far (u>0) => speed up left / slow right to drift right toward wall
            Bot.set_left_motor_speed( saturation(Bot, pp.base_rpm + u) )
            Bot.set_right_motor_speed( saturation(Bot, pp.base_rpm - u) )

        # Simple frontal-turn trigger (lidar-only)
        if forward_distance < pp.front_turn_thresh_mm:
            pp.rotate()  # uses wall_follow & d_distance by default
            # reset side PID state after a discrete turn
            pp.S_Integral = 0.0
            pp.S_PrevError = 0.0

        time.sleep(pp.Timestep)

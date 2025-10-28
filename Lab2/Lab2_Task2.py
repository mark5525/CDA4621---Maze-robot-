from HamBot.src.robot_systems.robot import HamBot
import time, math

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class Defintions():
    def __init__(self):
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5
        self.Kp = 2
        self.Ki = 0
        self.Kd = 5
        self.Timestep = 0.025
        self.Integral = 0.0
        self.Derivative = 0.0
        self.PrevError = 0.0

        # NEW: gentle-stop helpers
        self.StopBand = 10.0           # mm (tune 8–15)
        self.I_Limit  = 200.0          # integral clamp
        self.ApproachSlope = 0.5       # rpm per mm (speed cap shrinks near target)
        self.MinApproachRPM = 6.0      # don't crawl too slowly far out

    def forward_PID(self, bot, desired_distance):
        # NOTE: use the passed-in bot, not the global Bot
        scan = bot.get_range_image()
        window = [a for a in scan[175:180] if a and a > 0]
        if not window:
            return 0.0

        measured_distance = min(window)
        error = measured_distance - desired_distance

        # NEW: clean stop when close, and prevent re-accel
        if abs(error) <= self.StopBand:
            self.Integral = 0.0
            self.PrevError = 0.0
            return 0.0

        # PID terms
        self.Integral += error * self.Timestep
        # NEW: anti-windup clamp
        self.Integral = max(-self.I_Limit, min(self.Integral, self.I_Limit))

        derivative = (error - self.PrevError) / self.Timestep
        self.PrevError = error

        u = (self.K_p * error) + (self.K_i * self.Integral) + (self.K_d * derivative)

        # NEW: soft approach – limit RPM based on how close you are
        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u)

    def side_PID(self, bot, s_follow, desired_distance):
        if s_follow not in ("left", "right"):
            return 0.0  # or raise

        scan = bot.get_range_image()
        sl = slice(85, 95) if s_follow == "left" else slice(265, 275)
        window = [a for a in scan[sl] if a and a > 0]

        if not window:
            # optional: also reset integrator to avoid stale windup
            self.Integral = 0.0
            self.PrevError = 0.0
            return 0.0  # or return None if you want caller to detect "no data"

        measured_distance = min(window)
        error = measured_distance - desired_distance

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

        # Optional polarity flip for right-following:
        # if s_follow == "right":
        #     u = -u

        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u)

    def rotate(self, bot, direction="left", rpm=20, duration=0.70):
        """
        Simple in-place spin ~90°. No yaw/IMU. Tune 'duration' to your robot.
        Resets integrator to avoid stale history after rotation.
        """
        if direction == "left":
            left_cmd, right_cmd = -abs(rpm), abs(rpm)
        else:
            left_cmd, right_cmd = abs(rpm), -abs(rpm)

        bot.set_left_motor_speed(left_cmd)
        bot.set_right_motor_speed(right_cmd)
        time.sleep(duration)
        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)

        # Reset controller memory after the maneuver
        self.Integral = 0.0
        self.PrevError = 0.0
        time.sleep(self.Timestep)

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    wall_follow = "left"  # or "right"
    d_distance = 300  # side distance goal (mm)
    front_goal = 300  # how close to front wall before rotate (mm)
    pp = Defintions()


    while True:
        scan = Bot.get_range_image()
        fw = [a for a in scan[175:180] if a and a > 0] if isinstance(scan, list) else []
        forward_distance = min(fw) if fw else float("inf")

        # --- NEW: base = forward throttle, steer = side correction ---
        base  = pp.forward_PID(Bot, front_goal)                # forward speed
        steer = pp.side_PID(Bot, wall_follow, d_distance)      # steering term

        if wall_follow == "left":
            left_cmd  = saturation(Bot, base - steer)
            right_cmd = saturation(Bot, base + steer)
        else:  # "right"
            left_cmd  = saturation(Bot, base + steer)
            right_cmd = saturation(Bot, base - steer)

        # Rotate if front is close
        if forward_distance <= front_goal + pp.StopBand:
            pp.rotate(Bot, direction=("left" if wall_follow == "left" else "right"))
            time.sleep(pp.Timestep)
            continue

        # Send to motors at end of loop
        Bot.set_left_motor_speed(left_cmd)
        Bot.set_right_motor_speed(right_cmd)

        time.sleep(pp.Timestep)











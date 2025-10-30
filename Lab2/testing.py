"""
HamBot Task 2 — Wall Following (left/right)
- Side-distance PID steers (differential RPM), front band slows/rotates.
- Wrap sharp corners smoothly via side "drop-away" bias.
- Startup pull-in to avoid initial drift away from the tracked wall.
- Max RPM fixed at 60. Uses mm units for Lidar distances.

Usage:
  python Lab2_Task2.py             # defaults to left wall
  python Lab2_Task2.py right       # follow right wall
"""

import time
import sys
import math
from HamBot.src.robot_systems.robot import HamBot

# ========================
# Utility
# ========================

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class SlewLimiter:
    """Limits how fast a value can change per timestep (rpm per tick)."""
    def __init__(self, max_delta_per_tick):
        self.max_delta = float(max_delta_per_tick)
        self.prev = 0.0
    def step(self, target):
        delta = target - self.prev
        if   delta >  self.max_delta: delta =  self.max_delta
        elif delta < -self.max_delta: delta = -self.max_delta
        self.prev += delta
        return self.prev

def robust_min(values, keep=5):
    """
    Return a robust min by averaging the 'keep' smallest positive values.
    Helps reject occasional outliers/spikes.
    """
    vals = [v for v in values if v and v > 0]
    if not vals: return float("inf")
    vals.sort()
    k = min(keep, len(vals))
    return sum(vals[:k]) / k

# ========================
# Sensing helpers
# ========================

def get_front_distance(bot):
    # Front window ~ 175..180 deg (narrow)
    scan = bot.get_range_image()
    window = scan[175:181]
    return robust_min(window, keep=3)

def get_side_distance(bot, side):
    # Left ~ 90 deg, Right ~ 270 deg; use +/- 12° window
    scan = bot.get_range_image()
    if side == "left":
        window = scan[90-12:90+13]
    else:
        window = scan[270-12:270+13]
    return robust_min(window, keep=5)

def get_diag_distance(bot, side):
    # Diagonal forward along the tracked wall: left=135°, right=225°
    scan = bot.get_range_image()
    if side == "left":
        window = scan[135-8:135+9]
    else:
        # wrap-around for indices near 360:
        a, b = 225-8, 225+9
        window = scan[a:b]
    return robust_min(window, keep=4)

# ========================
# Controllers
# ========================

class SidePID:
    def __init__(self, kp=0.12, ki=0.02, kd=1.20, dt=0.032, i_limit=250.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.i = 0.0
        self.prev_e = 0.0
        self.i_limit = abs(i_limit)

    def step(self, error):
        # PI-D (derivative on measurement via error difference)
        self.i += error * self.dt
        if self.i >  self.i_limit: self.i =  self.i_limit
        if self.i < -self.i_limit: self.i = -self.i_limit
        d = (error - self.prev_e) / self.dt
        self.prev_e = error
        return self.kp*error + self.ki*self.i + self.kd*d

# ========================
# Wall-follow policy
# ========================

class WallFollower:
    def __init__(self, bot, side="left"):
        self.bot = bot
        self.side = side  # "left" or "right"

        # Timing
        self.dt = 0.032
        self.start_time = time.time()

        # Speed config
        self.cruise_rpm = 22.0
        self.rotate_rpm = 28.0
        self.max_rpm_change_per_tick = 4.0  # slew limiter to avoid surges

        # Distances (mm)
        self.target_side_mm = 300.0
        self.stop_front_mm = 280.0
        self.slow_front_mm = 450.0
        self.rotate_exit_mm = 360.0  # to unlatch rotate

        # Corner wrap config
        self.wrap_start_mm = 550.0
        self.wrap_gain = 0.008
        self.wrap_gain_max = 0.25   # clamp extra bias

        # Startup pull-in (gentle nudge toward the tracked wall)
        self.pull_duration = 0.9    # seconds
        self.pull_mag = 0.14        # added to steer signal (signed)

        # Steering scaling
        self.steer_to_rpm = 0.55    # converts PID "steer" to differential RPM

        # State
        self.side_pid = SidePID(dt=self.dt)
        self.mode = "follow"        # "follow" | "rotate"
        self.l_slew = SlewLimiter(self.max_rpm_change_per_tick)
        self.r_slew = SlewLimiter(self.max_rpm_change_per_tick)

    def _front_band_speed(self, f_mm):
        if f_mm >= self.slow_front_mm:
            return self.cruise_rpm
        if f_mm <= self.stop_front_mm:
            return 0.0
        # linear ramp between stop and slow thresholds
        alpha = (f_mm - self.stop_front_mm) / (self.slow_front_mm - self.stop_front_mm)
        return self.cruise_rpm * max(0.0, min(1.0, alpha))

    def _rotate_vector(self):
        # Rotate toward the tracked wall (left wall → CCW, right wall → CW)
        turn = self.rotate_rpm
        if self.side == "left":
            return (-turn, +turn)
        else:
            return (+turn, -turn)

    def step(self):
        f = get_front_distance(self.bot)
        s = get_side_distance(self.bot, self.side)
        d = get_diag_distance(self.bot, self.side)

        now = time.time()
        t = now - self.start_time

        # Mode transitions
        if self.mode == "follow":
            if f < self.stop_front_mm:
                self.mode = "rotate"
        elif self.mode == "rotate":
            # exit rotate once front is reasonably clear OR diagonal opens
            if (f > self.rotate_exit_mm) or (d > self.rotate_exit_mm):
                self.mode = "follow"

        # Mode actions
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_vector()
            # keep a trickle of forward bias if barely blocked to help "peel" past corners
            if f > self.stop_front_mm * 0.85:
                fwd_bias = 4.0
                if self.side == "left":
                    l_cmd -= fwd_bias
                    r_cmd -= fwd_bias
                else:
                    l_cmd -= fwd_bias
                    r_cmd -= fwd_bias
        else:
            # FOLLOW mode
            base = self._front_band_speed(f)

            # Side error (positive when too far from the wall)
            e_side = s - self.target_side_mm
            steer = self.side_pid.step(e_side)

            # Startup pull-in (decays to 0)
            if t < self.pull_duration:
                pull = self.pull_mag * (1.0 - (t / self.pull_duration))
                steer += (-pull if self.side == "left" else +pull)

            # Corner wrap bias: when the wall drops away, lean into the corner
            if s > self.wrap_start_mm and f > self.stop_front_mm:
                bias = self.wrap_gain * (s - self.wrap_start_mm)
                if bias > self.wrap_gain_max: bias = self.wrap_gain_max
                steer += (-bias if self.side == "left" else +bias)

            # Convert steer to differential wheel RPMs
            turn = self.steer_to_rpm * steer
            if self.side == "left":
                l_cmd = base - turn
                r_cmd = base + turn
            else:
                l_cmd = base + turn
                r_cmd = base - turn

        # Slew limit + saturation
        l_out = saturation(self.bot, self.l_slew.step(l_cmd))
        r_out = saturation(self.bot, self.r_slew.step(r_cmd))
        return l_out, r_out

# ========================
# Main
# ========================

if __name__ == "__main__":
    side_follow = "left"
    if len(sys.argv) >= 2 and sys.argv[1].lower() in ("left", "right"):
        side_follow = sys.argv[1].lower()

    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    dt = 0.032

    follower = WallFollower(Bot, side=side_follow)

    try:
        while True:
            l_rpm, r_rpm = follower.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)
            time.sleep(dt)
    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)




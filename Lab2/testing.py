
"""
HamBot — Task 2 (LEFT wall following only) — Tuned for less side oscillation
- Only numeric tweaks vs previous version (no logic changes)
"""

import time
from HamBot.src.robot_systems.robot import HamBot

# ========================
# Basic helpers
# ========================

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class SlewLimiter:
    """Limit how fast a command can change per control tick (rpm/tick)."""
    def __init__(self, max_delta_per_tick):
        self.max_delta = float(max_delta_per_tick)
        self.prev = 0.0
    def step(self, target):
        d = target - self.prev
        if   d >  self.max_delta: d =  self.max_delta
        elif d < -self.max_delta: d = -self.max_delta
        self.prev += d
        return self.prev

def robust_min(values, keep=5):
    """Average of the 'keep' smallest positive readings to resist spikes."""
    vals = [v for v in values if v and v > 0]
    if not vals: return float("inf")
    vals.sort()
    k = min(keep, len(vals))
    return sum(vals[:k]) / k

# ========================
# Sensing (angles: 0 back, 90 left, 180 front, 270 right)
# ========================

def front_distance(bot):
    scan = bot.get_range_image()
    # Narrow front window centered near 180°
    return robust_min(scan[175:186], keep=5)  # was keep=3

def left_side_distance(bot):
    scan = bot.get_range_image()
    # Around 90° ±12°
    return robust_min(scan[78:103], keep=7)   # was keep=5

def left_diag_distance(bot):
    scan = bot.get_range_image()
    # Around 135° ±8°
    return robust_min(scan[127:144], keep=5)  # was keep=4

# ========================
# Side (left) PID
# ========================

class SidePID:
    def __init__(self, kp=0.11, ki=0.01, kd=1.60, dt=0.032, i_limit=250.0):
        # was kp=0.12, ki=0.02, kd=1.10
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.i = 0.0
        self.prev_e = 0.0
        self.i_limit = abs(i_limit)
    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0
    def step(self, error):
        # clamp error to avoid runaway when wall not seen
        if error > 800.0:  error = 800.0
        if error < -800.0: error = -800.0
        self.i += error * self.dt
        if self.i >  self.i_limit: self.i =  self.i_limit
        if self.i < -self.i_limit: self.i = -self.i_limit
        d = (error - self.prev_e) / self.dt
        self.prev_e = error
        return self.kp*error + self.ki*self.i + self.kd*d

# ========================
# Controller
# ========================

class LeftWallFollower:
    def __init__(self, bot):
        self.bot = bot
        # Timing
        self.dt = 0.032
        self.start_time = time.time()

        # Speed/RPMs
        self.cruise_rpm = 20.0           # was 22.0
        self.rotate_rpm = 28.0
        self.max_rpm_slew = 2.5          # was 4.0 rpm/tick

        # Distance thresholds (mm)
        self.target_side_mm   = 300.0
        self.stop_front_mm    = 280.0
        self.slow_front_mm    = 450.0
        self.rotate_exit_mm   = 360.0
        self.reengage_side_mm = 800.0

        # Corner wrap
        self.wrap_start_mm  = 600.0       # was 550.0
        self.wrap_gain      = 0.006       # was 0.008
        self.wrap_gain_max  = 0.16        # was 0.22
        # Startup pull-in (smaller/shorter)
        self.pull_secs = 0.45             # was 0.6
        self.pull_mag  = 0.05             # was 0.08

        # Steering scaling (PID -> differential rpm)
        self.steer_to_rpm = 0.28          # was 0.35

        # "No wall" handling
        self.NO_WALL_THRESH = 2200.0      # was 2500.0

        # State
        self.mode = "follow"            # "follow" or "rotate"
        self.rotate_t0 = None
        self.rotate_min_time = 0.35
        self.rotate_max_time = 1.40

        self.pid = SidePID(dt=self.dt)
        self.l_slew = SlewLimiter(self.max_rpm_slew)
        self.r_slew = SlewLimiter(self.max_rpm_slew)

    def _front_band_speed(self, f_mm):
        if f_mm <= self.stop_front_mm: return 0.0
        if f_mm >= self.slow_front_mm: return self.cruise_rpm
        a = (f_mm - self.stop_front_mm) / (self.slow_front_mm - self.stop_front_mm)
        return self.cruise_rpm * max(0.0, min(1.0, a))

    def _rotate_ccw(self):
        return (-self.rotate_rpm, +self.rotate_rpm)

    def step(self):
        f = front_distance(self.bot)
        s = left_side_distance(self.bot)
        d = left_diag_distance(self.bot)
        t = time.time()

        # ================= Mode transitions =================
        if self.mode == "follow":
            if f < self.stop_front_mm:
                self.mode = "rotate"
                self.rotate_t0 = t
                self.pid.reset()

        elif self.mode == "rotate":
            rotating_for = t - (self.rotate_t0 or t)
            can_exit = rotating_for >= self.rotate_min_time
            must_exit = rotating_for >= self.rotate_max_time

            front_clear = f > self.rotate_exit_mm
            side_seen   = s < self.reengage_side_mm
            diag_seen   = d < self.reengage_side_mm

            if (must_exit or (can_exit and (front_clear or side_seen or diag_seen))):
                self.mode = "follow"
                self.rotate_t0 = None

        # ================= Commands =================
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_ccw()
            if f > self.stop_front_mm * 0.85:
                fb = 3.0
                l_cmd -= fb; r_cmd -= fb
        else:
            base = self._front_band_speed(f)

            if s >= self.NO_WALL_THRESH or s == float("inf"):
                e_side = 0.0
                steer = 0.0
            else:
                e_side = s - self.target_side_mm
                steer = self.pid.step(e_side)

            since_start = t - self.start_time
            if since_start < self.pull_secs:
                steer += self.pull_mag * (1.0 - since_start / self.pull_secs)

            if s > self.wrap_start_mm and f > self.stop_front_mm:
                bias = self.wrap_gain * (s - self.wrap_start_mm)
                if bias > self.wrap_gain_max: bias = self.wrap_gain_max
                steer += bias  # LEFT follow → positive steer = CCW

            turn = self.steer_to_rpm * steer
            l_cmd = base - turn
            r_cmd = base + turn

        l_out = saturation(self.bot, self.l_slew.step(l_cmd))
        r_out = saturation(self.bot, self.r_slew.step(r_cmd))
        return l_out, r_out

# ========================
# Main loop
# ========================

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    dt = 0.032

    ctrl = LeftWallFollower(Bot)

    try:
        while True:
            l_rpm, r_rpm = ctrl.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)
            time.sleep(dt)
    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)


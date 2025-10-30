
"""
HamBot — Task 2 (LEFT wall following only)
- Maintains a target lateral offset to the LEFT wall with a side PID (steer → differential RPM)
- When a frontal obstacle is detected: rotate in place ~90° CCW, then resume following
- Wraps corners when the left wall "drops away"
- Guards against the "spin in place" failure by clamping inputs and time-gating rotate mode

Usage:
  python Lab2_Task2_Left.py
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
    return robust_min(scan[175:186], keep=3)

def left_side_distance(bot):
    scan = bot.get_range_image()
    # Around 90° ±12°
    return robust_min(scan[78:103], keep=5)

def left_diag_distance(bot):
    scan = bot.get_range_image()
    # Around 135° ±8°
    return robust_min(scan[127:144], keep=4)

# ========================
# Side (left) PID
# ========================

class SidePID:
    def __init__(self, kp=0.12, ki=0.02, kd=1.10, dt=0.032, i_limit=250.0):
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
        self.cruise_rpm = 22.0
        self.rotate_rpm = 28.0
        self.max_rpm_slew = 4.0     # rpm per tick (keeps things smooth)

        # Distance thresholds (mm)
        self.target_side_mm   = 300.0
        self.stop_front_mm    = 280.0  # enter rotate if closer than this
        self.slow_front_mm    = 450.0  # start slowing here
        self.rotate_exit_mm   = 360.0  # exit rotate when front clears to this
        self.reengage_side_mm = 800.0  # side seen again after rotate

        # Corner wrap
        self.wrap_start_mm  = 550.0    # side "drop-away" starts wrap
        self.wrap_gain      = 0.008    # bias gain when side opens
        self.wrap_gain_max  = 0.22     # clamp
        # Startup pull-in (small, so it won't cause spin)
        self.pull_secs = 0.6
        self.pull_mag  = 0.08

        # Steering scaling (PID -> differential rpm)
        self.steer_to_rpm = 0.35  # toned down to prevent spiral turns

        # "No wall" handling
        self.NO_WALL_THRESH = 2500.0  # treat as "no side wall" above this

        # State
        self.mode = "follow"            # "follow" or "rotate"
        self.rotate_t0 = None
        self.rotate_min_time = 0.35     # stay rotating at least this long
        self.rotate_max_time = 1.40     # bail out if stuck

        self.pid = SidePID(dt=self.dt)
        self.l_slew = SlewLimiter(self.max_rpm_slew)
        self.r_slew = SlewLimiter(self.max_rpm_slew)

    def _front_band_speed(self, f_mm):
        if f_mm <= self.stop_front_mm: return 0.0
        if f_mm >= self.slow_front_mm: return self.cruise_rpm
        # linear ramp between stop and slow thresholds
        a = (f_mm - self.stop_front_mm) / (self.slow_front_mm - self.stop_front_mm)
        return self.cruise_rpm * max(0.0, min(1.0, a))

    def _rotate_ccw(self):
        # CCW to face down the new corridor; left wheel backward, right forward
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
                # Reset PID so we don't carry integral into the turn
                self.pid.reset()

        elif self.mode == "rotate":
            # Time-based guards to avoid instant exit / spin forever
            rotating_for = t - (self.rotate_t0 or t)
            can_exit = rotating_for >= self.rotate_min_time
            must_exit = rotating_for >= self.rotate_max_time

            # Conditions to exit rotate:
            #  - front has cleared OR
            #  - side wall is seen again (not huge) OR
            #  - diagonal is reasonably close (we've turned into corridor)
            front_clear = f > self.rotate_exit_mm
            side_seen   = s < self.reengage_side_mm
            diag_seen   = d < self.reengage_side_mm

            if (must_exit or (can_exit and (front_clear or side_seen or diag_seen))):
                self.mode = "follow"
                self.rotate_t0 = None

        # ================= Commands =================
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_ccw()

            # (Optional) give a tiny forward bias if not in a dead stop
            if f > self.stop_front_mm * 0.85:
                fb = 3.0
                l_cmd -= fb; r_cmd -= fb

        else:
            # FOLLOW mode
            base = self._front_band_speed(f)

            # Side error; clamp if "no wall" to avoid runaway turns
            if s >= self.NO_WALL_THRESH or s == float("inf"):
                e_side = 0.0  # don't feed garbage to PID
                steer = 0.0
            else:
                e_side = s - self.target_side_mm   # positive = too far from wall
                steer = self.pid.step(e_side)

            # Startup pull-in: small, short-lived nudge toward wall (CCW)
            since_start = t - self.start_time
            if since_start < self.pull_secs:
                steer += self.pull_mag * (1.0 - since_start / self.pull_secs)

            # Corner wrap bias when wall opens up
            if s > self.wrap_start_mm and f > self.stop_front_mm:
                bias = self.wrap_gain * (s - self.wrap_start_mm)
                if bias > self.wrap_gain_max: bias = self.wrap_gain_max
                steer += bias  # LEFT follow → positive steer = CCW

            # Map steer → differential RPM (LEFT forward slower when steer>0)
            turn = self.steer_to_rpm * steer
            l_cmd = base - turn
            r_cmd = base + turn

        # Slew + saturation
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



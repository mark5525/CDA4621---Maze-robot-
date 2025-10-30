
"""
HamBot — Task 2 (LEFT wall following) — Simple PD + Simple Wrap
- Side control: PD with deadband (no integral)
- Wrap: single proportional bias when side distance exceeds a threshold
- Rotate on frontal block; Search = gentle forward CCW arc when side is lost
- Small slew limiting to keep motor commands smooth
"""

import time
from HamBot.src.robot_systems.robot import HamBot

# ========================
# Utilities
# ========================

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class SlewLimiter:
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
    vals = [v for v in values if v and v > 0]
    if not vals: return float("inf")
    vals.sort()
    k = min(keep, len(vals))
    return sum(vals[:k]) / k

# ========================
# Sensing (0 back, 90 left, 180 front, 270 right)
# ========================

def front_distance(bot):
    scan = bot.get_range_image()
    return robust_min(scan[176:185], keep=5)

def left_side_distance(bot):
    scan = bot.get_range_image()
    return robust_min(scan[82:98], keep=5)   # ~90° ±8°

def left_diag_distance(bot):
    scan = bot.get_range_image()
    return robust_min(scan[128:142], keep=5) # ~135° ±7°

# ========================
# Simple PD (no integral)
# ========================

class SimplePD:
    def __init__(self, kp=0.14, kd=1.80, dt=0.032, d_clip=1500.0):
        self.kp, self.kd = kp, kd
        self.dt = dt
        self.prev_e = 0.0
        self.d_clip = abs(d_clip)
    def reset(self):
        self.prev_e = 0.0
    def step(self, e):
        d = (e - self.prev_e) / self.dt
        # clip derivative to avoid spikes from lidar glitches
        if d >  self.d_clip: d =  self.d_clip
        if d < -self.d_clip: d = -self.d_clip
        self.prev_e = e
        return self.kp*e + self.kd*d

# ========================
# Controller
# ========================

class LeftWallFollower:
    def __init__(self, bot):
        self.bot = bot
        self.dt = 0.032
        self.start_time = time.time()

        # Speeds
        self.cruise_rpm = 20.0
        self.rotate_rpm = 26.0
        self.search_rpm = 18.0
        self.max_rpm_slew = 1.6
        self.steer_to_rpm = 0.28
        self.turn_rpm_cap = 9.0

        # Distances (mm)
        self.target_side_mm = 300.0
        self.deadband_mm    = 10.0
        self.stop_front_mm  = 280.0
        self.slow_front_mm  = 460.0
        self.rotate_exit_mm = 380.0
        self.NO_WALL_THRESH = 1600.0

        # Simple wrap
        self.wrap_start_mm  = 480.0            # start biasing when side opens beyond this
        self.wrap_k         = 0.007            # proportional gain on (s - wrap_start)
        self.wrap_bias_max  = 0.12             # (pre RPM scaling) cap
        self.wrap_speed_fac = 0.70            # speed multiplier while wrapping

        # Startup pull-in (small)
        self.pull_secs = 0.60
        self.pull_mag  = 0.08

        # Search (when side lost briefly)
        self.lost_side_time = 0.25
        self.search_turn_rpm = 4.5
        self._lost_since = None

        # State
        self.mode = "follow"  # "follow" | "rotate" | "search"
        self.rotate_t0 = None
        self.rotate_min_time = 0.36
        self.rotate_max_time = 1.20

        self.pd = SimplePD(dt=self.dt)
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
        now = time.time()

        # Side seen?
        side_seen = (s != float("inf")) and (s < self.NO_WALL_THRESH)

        # ============== Mode transitions ==============
        if self.mode == "follow":
            if f < self.stop_front_mm:
                self.mode = "rotate"; self.rotate_t0 = now; self.pd.reset()
            else:
                if side_seen:
                    self._lost_since = None
                else:
                    if self._lost_since is None:
                        self._lost_since = now
                    elif (now - self._lost_since) > self.lost_side_time:
                        self.mode = "search"

        elif self.mode == "rotate":
            rotating_for = now - (self.rotate_t0 or now)
            can_exit = rotating_for >= self.rotate_min_time
            must_exit = rotating_for >= self.rotate_max_time
            front_clear = f > self.rotate_exit_mm
            if must_exit or (can_exit and front_clear):
                self.mode = "follow"; self.rotate_t0 = None

        elif self.mode == "search":
            if f < self.stop_front_mm:
                self.mode = "rotate"; self.rotate_t0 = now; self.pd.reset()
            elif side_seen:
                self.mode = "follow"; self._lost_since = None

        # ============== Commands ==============
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_ccw()
            if f > self.stop_front_mm * 0.85:
                fb = 2.0; l_cmd -= fb; r_cmd -= fb

        elif self.mode == "search":
            base = self.search_rpm
            l_cmd = base - self.search_turn_rpm
            r_cmd = base + self.search_turn_rpm

        else:  # follow
            base = self._front_band_speed(f)

            # PD on side error (with deadband)
            if not side_seen:
                steer_pd = 0.0
            else:
                e = s - self.target_side_mm
                if abs(e) <= self.deadband_mm: e = 0.0
                steer_pd = self.pd.step(e)

            # Simple wrap bias (only when side opens and front not blocked)
            wrap_bias = 0.0
            if side_seen and (s > self.wrap_start_mm) and (f > self.stop_front_mm):
                wrap_bias = self.wrap_k * (s - self.wrap_start_mm)
                if wrap_bias > self.wrap_bias_max: wrap_bias = self.wrap_bias_max
                base *= self.wrap_speed_fac  # slow down a bit during wrap

            steer = steer_pd + wrap_bias   # positive steer = CCW for left follow
            turn = self.steer_to_rpm * steer
            if turn >  self.turn_rpm_cap: turn =  self.turn_rpm_cap
            if turn < -self.turn_rpm_cap: turn = -self.turn_rpm_cap

            # Startup pull-in
            since_start = now - self.start_time
            if since_start < self.pull_secs:
                turn += self.steer_to_rpm * self.pull_mag * (1.0 - since_start / self.pull_secs)

            l_cmd = base - turn
            r_cmd = base + turn

        l_out = saturation(self.bot, self.l_slew.step(l_cmd))
        r_out = saturation(self.bot, self.r_slew.step(r_cmd))
        return l_out, r_out

# ========================
# Main
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

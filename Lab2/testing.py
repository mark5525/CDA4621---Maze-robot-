
"""
HamBot — Task 2 (LEFT wall following) — SIMPLE, re-tuned for:
1) Clean 90° CW rotates (no 120° overshoot)
2) Straighter side tracking (reduced oscillation)
3) Tighter corner hugging (smaller wrap radius)

Changes vs prior "simple_CW_tuned":
- Rotate: rotate_rpm=22, rotate_min_time=0.28, rotate_max_time=0.90, rotate_exit_mm=340
          Exit rotate when (time>=min) and (front_clear OR side_seen OR diag_close)
- Side PD: kp=0.08, kd=2.10; deadband=16; steer_to_rpm=0.22; max_rpm_slew=1.2; turn_rpm_cap=6.5
- Wrap (tighter): wrap_start_mm=380, wrap_k=0.0105, wrap_speed_fac=0.55, wrap_bias_max=0.12
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
    # Narrow + smoothed to reduce oscillation
    return robust_min(scan[84:96], keep=7)   # ~90° ±6°

def left_diag_distance(bot):
    scan = bot.get_range_image()
    return robust_min(scan[128:142], keep=5) # ~135° ±7°

# ========================
# Simple PD (no integral)
# ========================

class SimplePD:
    def __init__(self, kp=0.08, kd=2.10, dt=0.032, d_clip=1500.0):
        self.kp, self.kd = kp, kd
        self.dt = dt
        self.prev_e = 0.0
        self.d_clip = abs(d_clip)
    def reset(self):
        self.prev_e = 0.0
    def step(self, e):
        d = (e - self.prev_e) / self.dt
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
        self.cruise_rpm = 19.0
        self.rotate_rpm = 22.0      # gentler CW rotate to reduce overshoot
        self.search_rpm = 18.0
        self.max_rpm_slew = 1.2     # calmer changes for straighter tracking
        self.steer_to_rpm = 0.22    # less steering authority to avoid wag
        self.turn_rpm_cap = 6.5     # hard cap on differential

        # Distances (mm)
        self.target_side_mm = 300.0
        self.deadband_mm    = 16.0
        self.stop_front_mm  = 280.0
        self.slow_front_mm  = 460.0
        self.rotate_exit_mm = 340.0
        self.NO_WALL_THRESH = 1600.0
        self.diag_capture_mm = 480.0   # ~sqrt(2)*300 for new corridor capture

        # Wrap (tighter corners)
        self.wrap_start_mm  = 380.0
        self.wrap_k         = 0.0105
        self.wrap_bias_max  = 0.12
        self.wrap_speed_fac = 0.55

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
        self.rotate_min_time = 0.28
        self.rotate_max_time = 0.90

        self.pd = SimplePD(dt=self.dt)
        self.l_slew = SlewLimiter(self.max_rpm_slew)
        self.r_slew = SlewLimiter(self.max_rpm_slew)

    def _front_band_speed(self, f_mm):
        if f_mm <= self.stop_front_mm: return 0.0
        if f_mm >= self.slow_front_mm: return self.cruise_rpm
        a = (f_mm - self.stop_front_mm) / (self.slow_front_mm - self.stop_front_mm)
        return self.cruise_rpm * max(0.0, min(1.0, a))

    def _rotate_cw(self):
        # CLOCKWISE spin for LEFT-wall follow: left forward, right backward
        return (+self.rotate_rpm, -self.rotate_rpm)

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
            diag_close  = d < self.diag_capture_mm
            if must_exit or (can_exit and (front_clear or side_seen or diag_close)):
                self.mode = "follow"; self.rotate_t0 = None

        elif self.mode == "search":
            if f < self.stop_front_mm:
                self.mode = "rotate"; self.rotate_t0 = now; self.pd.reset()
            elif side_seen:
                self.mode = "follow"; self._lost_since = None

        # ============== Commands ==============
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_cw()

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
                base *= self.wrap_speed_fac  # slow down during wrap

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



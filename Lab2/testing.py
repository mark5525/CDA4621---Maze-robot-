
"""
HamBot — Task 2 (LEFT wall following) — Stable + anti-oscillation + no-spin
- Straighter tracking via: derivative-filtered PID, deadband, EMA-smoothed side distance
- Gentler corner wrap based on side "opening rate" (ds/dt) with tight cap
- New "search" mode when wall is briefly lost: forward CCW arc instead of in-place spin
- Clear 90° rotate only when the FRONT is too close
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
    return robust_min(scan[176:185], keep=5)

def left_side_distance(bot):
    scan = bot.get_range_image()
    # Narrower window to avoid mixing fore/aft returns
    return robust_min(scan[82:98], keep=5)

def left_diag_distance(bot):
    scan = bot.get_range_image()
    return robust_min(scan[128:142], keep=5)

# ========================
# Side (left) PID with derivative filtering + integral leak
# ========================

class SidePID:
    def __init__(self, kp=0.15, ki=0.008, kd=2.20, dt=0.032,
                 i_limit=240.0, i_leak=0.04, d_alpha=0.25, out_limit=1200.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.i = 0.0
        self.prev_e = 0.0
        self.i_limit = abs(i_limit)
        self.i_leak = float(i_leak)      # 0..1 per step
        self.d_alpha = float(d_alpha)    # EMA for derivative
        self.d_filt = 0.0
        self.out_limit = abs(out_limit)
    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0
        self.d_filt = 0.0
    def step(self, error):
        # clamp error to avoid runaway when wall not seen
        if error > 800.0:  error = 800.0
        if error < -800.0: error = -800.0
        # integral with leakage (prevents long-term windup)
        self.i = (1.0 - self.i_leak)*self.i + error * self.dt
        if self.i >  self.i_limit: self.i =  self.i_limit
        if self.i < -self.i_limit: self.i = -self.i_limit
        # derivative with EMA filtering
        d_raw = (error - self.prev_e) / self.dt
        self.d_filt = self.d_alpha*d_raw + (1.0 - self.d_alpha)*self.d_filt
        self.prev_e = error
        u = self.kp*error + self.ki*self.i + self.kd*self.d_filt
        # output limit
        if u >  self.out_limit: u =  self.out_limit
        if u < -self.out_limit: u = -self.out_limit
        return u

# ========================
# Controller
# ========================

class LeftWallFollower:
    def __init__(self, bot):
        self.bot = bot
        # Timing
        self.dt = 0.032
        self.start_time = time.time()

        # Speeds
        self.cruise_rpm = 20.0
        self.rotate_rpm = 26.0             # gentler rotate to avoid overshoot
        self.search_rpm = 18.0             # forward speed while searching
        self.max_rpm_slew = 2.0            # tight to reduce zig-zag

        # Distance thresholds (mm)
        self.target_side_mm   = 300.0
        self.deadband_mm      = 12.0        # within this, treat error as zero
        self.stop_front_mm    = 280.0       # enter rotate if closer than this
        self.slow_front_mm    = 460.0
        self.rotate_exit_mm   = 380.0
        self.reengage_side_mm = 700.0

        # Corner wrap — rate-based and small
        self.wrap_ds_trigger  = 70.0        # mm increase in side over dt to start bias
        self.wrap_gain_rate   = 0.0055      # bias per (mm increase)
        self.wrap_bias_max    = 0.10        # cap on added steer (pre RPM scaling)

        # Startup pull-in
        self.pull_secs = 0.60
        self.pull_mag  = 0.08

        # Steering scaling
        self.steer_to_rpm = 0.30            # PID units -> differential RPM
        self.turn_rpm_cap = 8.0             # cap |turn| so we never whip the wheels

        # "No wall" handling
        self.NO_WALL_THRESH = 2000.0        # treat as "no side wall" above this

        # Wall-loss search (prevents circles)
        self.lost_side_time = 0.35          # seconds of "no side" to enter search
        self.search_turn_rpm = 4.5          # small CCW arc, not in-place spin
        self._lost_since = None

        # State
        self.mode = "follow"                # "follow" | "rotate" | "search"
        self.rotate_t0 = None
        self.rotate_min_time = 0.36
        self.rotate_max_time = 1.40

        self.pid = SidePID(dt=self.dt)
        self.l_slew = SlewLimiter(self.max_rpm_slew)
        self.r_slew = SlewLimiter(self.max_rpm_slew)

        # Filters
        self.side_ema = None
        self.ema_alpha = 0.28               # EMA smoothing for side distance

        self.prev_side = None               # for ds/dt

    def _ema_side(self, s_raw):
        if s_raw == float("inf"):
            return s_raw
        if self.side_ema is None:
            self.side_ema = s_raw
        else:
            self.side_ema = self.ema_alpha*s_raw + (1.0 - self.ema_alpha)*self.side_ema
        return self.side_ema

    def _front_band_speed(self, f_mm):
        if f_mm <= self.stop_front_mm: return 0.0
        if f_mm >= self.slow_front_mm: return self.cruise_rpm
        a = (f_mm - self.stop_front_mm) / (self.slow_front_mm - self.stop_front_mm)
        return self.cruise_rpm * max(0.0, min(1.0, a))

    def _rotate_ccw(self):
        return (-self.rotate_rpm, +self.rotate_rpm)

    def step(self):
        f = front_distance(self.bot)
        s_raw = left_side_distance(self.bot)
        d = left_diag_distance(self.bot)
        s = self._ema_side(s_raw)
        t = time.time()

        # Compute side opening rate (approx. ds over one step)
        ds = 0.0
        if self.prev_side is not None and s != float("inf"):
            ds = s - self.prev_side
        self.prev_side = s if s != float("inf") else self.prev_side

        # Track wall loss timer
        side_seen_now = (s != float("inf")) and (s < self.NO_WALL_THRESH)
        if side_seen_now:
            self._lost_since = None
        else:
            if self._lost_since is None:
                self._lost_since = t

        # =============== Mode transitions ===============
        if self.mode == "follow":
            if f < self.stop_front_mm:
                self.mode = "rotate"
                self.rotate_t0 = t
                self.pid.reset()
            elif (self._lost_since is not None) and ((t - self._lost_since) > self.lost_side_time) and (f > self.stop_front_mm):
                # Side lost but front is clear -> search with forward CCW arc
                self.mode = "search"

        elif self.mode == "rotate":
            rotating_for = t - (self.rotate_t0 or t)
            can_exit = rotating_for >= self.rotate_min_time
            must_exit = rotating_for >= self.rotate_max_time

            front_clear = f > self.rotate_exit_mm
            side_seen   = side_seen_now
            diag_seen   = d < self.reengage_side_mm

            if (must_exit or (can_exit and (front_clear or side_seen or diag_seen))):
                self.mode = "follow"
                self.rotate_t0 = None

        elif self.mode == "search":
            # Exit search as soon as we see the side again or a corner condition demands rotation
            if f < self.stop_front_mm:
                self.mode = "rotate"
                self.rotate_t0 = t
                self.pid.reset()
            elif side_seen_now:
                self.mode = "follow"

        # =============== Commands ===============
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_ccw()
            if f > self.stop_front_mm * 0.85:
                fb = 2.5
                l_cmd -= fb; r_cmd -= fb

        elif self.mode == "search":
            # gentle forward CCW arc to reacquire the left wall
            base = self.search_rpm
            l_cmd = base - self.search_turn_rpm
            r_cmd = base + self.search_turn_rpm

        else:  # follow
            base = self._front_band_speed(f)

            # Side error with deadband
            if not side_seen_now:
                error = 0.0
                steer = 0.0
            else:
                error = s - self.target_side_mm
                if abs(error) <= self.deadband_mm:
                    error = 0.0
                steer = self.pid.step(error)

            # Startup pull-in
            since_start = t - self.start_time
            if since_start < self.pull_secs:
                steer += self.pull_mag * (1.0 - since_start / self.pull_secs)

            # Corner wrap bias based on side opening RATE (ds)
            if (ds > self.wrap_ds_trigger) and (f > self.stop_front_mm):
                bias = min(self.wrap_bias_max, self.wrap_gain_rate * ds)
                steer += bias  # LEFT follow → positive steer = CCW

            # Convert steer to differential RPMs with turn cap
            turn = self.steer_to_rpm * steer
            if turn >  self.turn_rpm_cap: turn =  self.turn_rpm_cap
            if turn < -self.turn_rpm_cap: turn = -self.turn_rpm_cap

            l_cmd = base - turn
            r_cmd = base + turn

        # Slew + saturation
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

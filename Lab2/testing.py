
"""
HamBot — Task 2 (LEFT wall following) — Smooth & Tight Corner Wrap
- Adds a dedicated WRAP mode to prevent oscillation during cornering.
- Uses feedforward curvature + reduced speed + gain-scheduled PID inside WRAP.
- Keeps SEARCH for wall loss and ROTATE for true frontal obstacles.
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
    return robust_min(scan[82:98], keep=5)

def left_diag_distance(bot):
    scan = bot.get_range_image()
    return robust_min(scan[128:142], keep=5)

# ========================
# Side (left) PID with derivative filtering + integral leak
# ========================

class SidePID:
    def __init__(self, kp=0.15, ki=0.008, kd=2.00, dt=0.032,
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
    def step(self, error, scale=1.0):
        # clamp error to avoid runaway when wall not seen
        if error > 800.0:  error = 800.0
        if error < -800.0: error = -800.0
        # scale the error (used in WRAP to soften PID)
        error *= scale
        # integral with leakage
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
        self.rotate_rpm = 26.0
        self.search_rpm = 18.0
        self.max_rpm_slew = 2.0

        # Distance thresholds (mm)
        self.target_side_mm   = 300.0
        self.deadband_mm      = 12.0
        self.stop_front_mm    = 280.0
        self.slow_front_mm    = 460.0
        self.rotate_exit_mm   = 380.0
        self.reengage_side_mm = 700.0

        # Corner WRAP detection (enter when side opens fast or far)
        self.wrap_ds_trigger   = 60.0      # mm per step
        self.wrap_open_mm      = 180.0     # side - target
        self.wrap_hysteresis   = 40.0      # prevents chatter

        # WRAP behavior
        self.wrap_pid_scale    = 0.55      # reduce PID authority to avoid oscillation
        self.wrap_speed_factor = (0.55, 0.82)  # min..max factor on cruise during wrap
        self.wrap_ff_k_ds      = 0.0050    # feedforward from ds
        self.wrap_ff_k_s       = 0.0016    # feedforward from (s - target)
        self.wrap_ff_k_diag    = 0.0020    # feedforward from diag deficit
        self.wrap_ff_cap       = 0.12      # max additional steer (pre RPM scaling)
        self.wrap_ff_alpha     = 0.25      # EMA filter for FF to keep it smooth
        self.wrap_min_time     = 0.20
        self.wrap_max_time     = 1.10

        # Startup pull-in
        self.pull_secs = 0.60
        self.pull_mag  = 0.08

        # Steering scaling
        self.steer_to_rpm = 0.30
        self.turn_rpm_cap = 8.0

        # "No wall" handling
        self.NO_WALL_THRESH = 2000.0

        # Wall-loss search
        self.lost_side_time = 0.35
        self.search_turn_rpm = 4.5
        self._lost_since = None

        # State
        self.mode = "follow"                # "follow" | "rotate" | "search" | "wrap"
        self.rotate_t0 = None
        self.rotate_min_time = 0.36
        self.rotate_max_time = 1.40

        self.wrap_t0 = None
        self.ff_ema = 0.0

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

    def _wrap_should_enter(self, s, ds, d, f):
        if s == float("inf"): return False
        far_open = (s - self.target_side_mm) > (self.wrap_open_mm + self.wrap_hysteresis)
        fast_open = ds > self.wrap_ds_trigger
        diag_far = d > self.reengage_side_mm  # corridor likely continues
        front_ok = f > self.stop_front_mm
        return front_ok and (fast_open or far_open or diag_far)

    def _wrap_should_exit(self, s, ds, d, f, now):
        if self.wrap_t0 is None: return True
        t = now - self.wrap_t0
        long_enough = t >= self.wrap_min_time
        too_long = t >= self.wrap_max_time
        close_enough = (s != float("inf")) and (s - self.target_side_mm) < (self.wrap_open_mm - self.wrap_hysteresis)
        opening_slow = ds < (0.5 * self.wrap_ds_trigger)
        diag_ok = d < self.reengage_side_mm
        front_ok = f > self.stop_front_mm
        return too_long or (long_enough and ( (close_enough and opening_slow) or diag_ok ) and front_ok)

    def _wrap_feedforward(self, s, ds, d):
        if s == float("inf"): return 0.0
        e = (s - self.target_side_mm)
        diag_def = max(0.0, d - self.reengage_side_mm)  # positive if diag is far
        ff_raw = self.wrap_ff_k_ds*max(0.0, ds) + self.wrap_ff_k_s*max(0.0, e) + self.wrap_ff_k_diag*diag_def
        # EMA to keep it smooth
        self.ff_ema = self.wrap_ff_alpha*ff_raw + (1.0 - self.wrap_ff_alpha)*self.ff_ema
        if self.ff_ema >  self.wrap_ff_cap: self.ff_ema =  self.wrap_ff_cap
        if self.ff_ema < 0.0: self.ff_ema = 0.0
        return self.ff_ema  # positive → CCW

    def step(self):
        f = front_distance(self.bot)
        s_raw = left_side_distance(self.bot)
        d = left_diag_distance(self.bot)
        s = self._ema_side(s_raw)
        now = time.time()

        # Compute ds (side opening rate)
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
                self._lost_since = now

        # ================= Mode transitions =================
        if self.mode == "follow":
            if f < self.stop_front_mm:
                self.mode = "rotate"; self.rotate_t0 = now; self.pid.reset()
            elif (self._lost_since is not None) and ((now - self._lost_since) > self.lost_side_time) and (f > self.stop_front_mm):
                self.mode = "search"
            elif self._wrap_should_enter(s, ds, d, f):
                self.mode = "wrap"; self.wrap_t0 = now; self.ff_ema = 0.0  # reset FF

        elif self.mode == "rotate":
            rotating_for = now - (self.rotate_t0 or now)
            can_exit = rotating_for >= self.rotate_min_time
            must_exit = rotating_for >= self.rotate_max_time
            front_clear = f > self.rotate_exit_mm
            side_seen   = side_seen_now
            diag_seen   = d < self.reengage_side_mm
            if (must_exit or (can_exit and (front_clear or side_seen or diag_seen))):
                self.mode = "follow"; self.rotate_t0 = None

        elif self.mode == "search":
            if f < self.stop_front_mm:
                self.mode = "rotate"; self.rotate_t0 = now; self.pid.reset()
            elif side_seen_now:
                # resume normal follow; wrap logic will catch if still opening
                self.mode = "follow"

        elif self.mode == "wrap":
            if f < self.stop_front_mm:
                self.mode = "rotate"; self.rotate_t0 = now; self.pid.reset()
            elif self._wrap_should_exit(s, ds, d, f, now):
                self.mode = "follow"

        # ================= Commands =================
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_ccw()
            if f > self.stop_front_mm * 0.85:
                fb = 2.0; l_cmd -= fb; r_cmd -= fb

        elif self.mode == "search":
            base = self.search_rpm
            l_cmd = base - self.search_turn_rpm
            r_cmd = base + self.search_turn_rpm

        elif self.mode == "wrap":
            # WRAP mode: reduced speed, FF curvature + softened PID to prevent oscillation
            # Speed factor based on how far/fast opening is
            e = 0.0 if s == float("inf") else max(0.0, s - self.target_side_mm)
            open_norm = min(1.0, max(e, ds) / (self.wrap_open_mm + 2*self.wrap_ds_trigger))
            speed_fac = self.wrap_speed_factor[0]*(open_norm) + self.wrap_speed_factor[1]*(1.0 - open_norm)
            base = self.cruise_rpm * speed_fac

            # Feedforward curvature
            ff = self._wrap_feedforward(s, ds, d)

            # PID with gain scheduling
            error = 0.0 if not side_seen_now else (s - self.target_side_mm)
            steer_pid = self.pid.step(error, scale=self.wrap_pid_scale)

            steer = steer_pid + ff  # positive → CCW
            turn = self.steer_to_rpm * steer
            if turn >  self.turn_rpm_cap: turn =  self.turn_rpm_cap
            if turn < -self.turn_rpm_cap: turn = -self.turn_rpm_cap

            l_cmd = base - turn
            r_cmd = base + turn

        else:  # follow
            base = self._front_band_speed(f)

            if not side_seen_now:
                error = 0.0; steer = 0.0
            else:
                error = s - self.target_side_mm
                if abs(error) <= self.deadband_mm: error = 0.0
                steer = self.pid.step(error)

            # Startup pull-in
            since_start = now - self.start_time
            if since_start < self.pull_secs:
                steer += self.pull_mag * (1.0 - since_start / self.pull_secs)

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

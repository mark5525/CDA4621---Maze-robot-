
"""
HamBot — Task 2 (LEFT/RIGHT Wall Following) — Strong Corner Wrap + Straighter Follow

Fixes your two issues:
1) **Ignores corners / drives straight** → Added **reliable corner-wrap detection**
   using side opening (s - target) and opening rate (ds). When triggered, it slows
   down and adds a controlled CCW/CW bias until the new wall is acquired.
2) **Wall follow not good enough** → Cleaner side PD (no integral), smoothed side
   distance (EMA), tighter steering authority, and slew limiting for straight lines.
3) Frontal obstacle → time-gated, tapered ~90° rotate (CW for left follow, CCW for right).

Keep this file simple: one FOLLOW loop with small state flags (ROTATE, SEARCH, WRAP).
"""

import time, math
from HamBot.src.robot_systems.robot import HamBot

# ========================
# Timing
# ========================
DT = 0.032

# ========================
# Geometry / tuning
# ========================
TARGET_MM        = 300.0
NO_WALL_MM       = 2000.0

CRUISE_RPM       = 19.0
SEARCH_RPM       = 16.0
ROT_RPM          = 22.0
ROT_MIN_RPM      = 6.0
TURN_CAP_RPM     = 7.0         # steering cap
STEER_TO_RPM     = 0.24        # map steer -> differential rpm
SLEW_RPM_PER_TIK = 1.4

# Front thresholds
FRONT_STOP_MM    = 280.0
FRONT_SLOW_MM    = 460.0
ROT_EXIT_MM      = 340.0

# Side PD (no integral)
Kp = 0.12
Kd = 2.00
D_CLIP = 1500.0

# Side smoothing
EMA_ALPHA        = 0.28

# Corner wrap detection
WRAP_OPEN_MM     = 120.0       # side - target
WRAP_DS_TRIG     = 55.0        # opening rate per step
WRAP_MIN_T       = 0.22
WRAP_MAX_T       = 1.10
WRAP_BIAS_GAIN   = 0.0062      # bias from (s - wrap_threshold)
WRAP_RATE_GAIN   = 0.0045      # bias from ds
WRAP_BIAS_MAX    = 0.12
WRAP_SPEED_FAC   = 0.60        # slow during wrap
DIAG_CAPTURE_MM  = 480.0       # ~sqrt(2)*300

# Search (if side gone too long)
LOST_SIDE_T      = 0.28
SEARCH_TURN_RPM  = 4.5

# ========================
# Helpers
# ========================
def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class SlewLimiter:
    def __init__(self, max_delta):
        self.max_delta = float(max_delta)
        self.prev = 0.0
    def step(self, target):
        d = target - self.prev
        if d >  self.max_delta: d =  self.max_delta
        if d < -self.max_delta: d = -self.max_delta
        self.prev += d
        return self.prev

def robust_min(vals, keep=5):
    xs = [v for v in vals if v and v > 0]
    if not xs: return float('inf')
    xs.sort()
    return sum(xs[:min(keep, len(xs))]) / min(keep, len(xs))

# ========================
# Sensors
# Angles: 0 back, 90 left, 180 front, 270 right
# ========================
def sense_front(bot):
    scan = bot.get_range_image()
    return robust_min(scan[176:186], keep=5)

def sense_left(bot):
    scan = bot.get_range_image()
    return robust_min(scan[84:96], keep=7)

def sense_right(bot):
    scan = bot.get_range_image()
    return robust_min(scan[264:276], keep=7)

def sense_diag_left(bot):
    scan = bot.get_range_image()
    return robust_min(scan[128:142], keep=5)

# ========================
# Controller
# ========================
class Follower:
    def __init__(self, bot, wall_side="left"):
        self.bot = bot
        self.wall_side = wall_side  # "left" | "right"

        self.side_ema = None
        self.prev_side = None
        self.prev_err  = 0.0

        self.mode = "follow"        # "follow" | "rotate" | "search" | "wrap"
        self.rotate_t0 = None
        self.wrap_t0 = None
        self.lost_since = None

        self.l_slew = SlewLimiter(SLEW_RPM_PER_TIK)
        self.r_slew = SlewLimiter(SLEW_RPM_PER_TIK)

    # --- utilities ---
    def _front_band_speed(self, f_mm):
        if f_mm <= FRONT_STOP_MM: return 0.0
        if f_mm >= FRONT_SLOW_MM: return CRUISE_RPM
        a = (f_mm - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM)
        return CRUISE_RPM * max(0.0, min(1.0, a))

    def _rotate_cw(self):
        # CW: left+, right-
        return (+ROT_RPM, -ROT_RPM)

    def _rotate_ccw(self):
        # CCW: left-, right+
        return (-ROT_RPM, +ROT_RPM)

    # --- main step ---
    def step(self):
        f = sense_front(self.bot)
        dL = sense_diag_left(self.bot)

        # pick side
        if self.wall_side == "left":
            s_raw = sense_left(self.bot)
            side_seen = s_raw < NO_WALL_MM
            turn_sign = +1   # +steer => CCW
        else:
            s_raw = sense_right(self.bot)
            side_seen = s_raw < NO_WALL_MM
            turn_sign = -1   # +steer => CW

        # EMA
        if s_raw != float('inf'):
            if self.side_ema is None: self.side_ema = s_raw
            else: self.side_ema = EMA_ALPHA*s_raw + (1.0-EMA_ALPHA)*self.side_ema
        s = self.side_ema if self.side_ema is not None else s_raw

        # ds
        ds = 0.0
        if self.prev_side is not None and s != float('inf'):
            ds = s - self.prev_side
        self.prev_side = s if s != float('inf') else self.prev_side

        now = time.time()
        if side_seen: self.lost_since = None
        else:
            if self.lost_since is None: self.lost_since = now

        # ===== Mode transitions =====
        if self.mode == "follow":
            if f < FRONT_STOP_MM:
                self.mode = "rotate"; self.rotate_t0 = now
            else:
                # Corner wrap enter if opening far or fast
                open_far  = (s != float('inf')) and ((s - TARGET_MM) > WRAP_OPEN_MM)
                open_fast = ds > WRAP_DS_TRIG
                if (open_far or open_fast) and f > FRONT_STOP_MM:
                    self.mode = "wrap"; self.wrap_t0 = now
                elif (not side_seen) and (self.lost_since and (now - self.lost_since) > LOST_SIDE_T):
                    self.mode = "search"

        elif self.mode == "rotate":
            t = now - (self.rotate_t0 or now)
            can_exit = t >= 0.26
            must_exit = t >= 0.90
            front_clear = f > ROT_EXIT_MM
            diag_close = dL < DIAG_CAPTURE_MM
            if must_exit or (can_exit and (front_clear or side_seen or diag_close)):
                self.mode = "follow"; self.rotate_t0 = None

        elif self.mode == "wrap":
            t = now - (self.wrap_t0 or now)
            done_time = t >= WRAP_MIN_T
            too_long  = t >= WRAP_MAX_T
            diag_close = dL < DIAG_CAPTURE_MM
            close_enough = (s != float('inf')) and ((s - TARGET_MM) < 0.6*WRAP_OPEN_MM)
            if f < FRONT_STOP_MM:
                self.mode = "rotate"; self.rotate_t0 = now; self.wrap_t0 = None
            elif too_long or (done_time and (diag_close or close_enough)):
                self.mode = "follow"; self.wrap_t0 = None
            elif (not side_seen) and (self.lost_since and (now - self.lost_since) > LOST_SIDE_T + 0.4):
                self.mode = "search"

        elif self.mode == "search":
            if f < FRONT_STOP_MM:
                self.mode = "rotate"; self.rotate_t0 = now
            elif side_seen:
                self.mode = "follow"; self.lost_since = None

        # ===== Commands =====
        if self.mode == "rotate":
            # CW for LEFT follow; CCW for RIGHT follow
            if self.wall_side == "left":
                l_cmd, r_cmd = self._rotate_cw()
            else:
                l_cmd, r_cmd = self._rotate_ccw()

            # taper as we rotate (simple: scale by remaining angle using front clearance proxy)
            # small fixed taper: avoid overshoot
            l_cmd *= 0.95; r_cmd *= 0.95

        elif self.mode == "search":
            base = SEARCH_RPM
            # arc toward the chosen wall
            turn = SEARCH_TURN_RPM * turn_sign
            l_cmd = base - turn
            r_cmd = base + turn

        else:
            # FOLLOW / WRAP
            base = self._front_band_speed(f)

            # Side PD (signed steer)
            if side_seen and s != float('inf'):
                e = s - TARGET_MM
                dterm = (e - self.prev_err) / DT
                if dterm >  D_CLIP: dterm =  D_CLIP
                if dterm < -D_CLIP: dterm = -D_CLIP
                steer_pd = Kp*e + Kd*dterm
                self.prev_err = e
            else:
                steer_pd = 0.0

            # Wrap bias if in wrap mode
            wrap_bias = 0.0
            if self.mode == "wrap":
                e_open = max(0.0, (s - (TARGET_MM + WRAP_OPEN_MM))) if s != float('inf') else WRAP_OPEN_MM
                wrap_bias = WRAP_BIAS_GAIN*e_open + WRAP_RATE_GAIN*max(0.0, ds)
                if wrap_bias > WRAP_BIAS_MAX: wrap_bias = WRAP_BIAS_MAX
                base *= WRAP_SPEED_FAC

            steer = steer_pd + wrap_bias
            # Signed turn relative to chosen side
            turn = STEER_TO_RPM * steer * turn_sign
            if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
            if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM

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

    # Choose: "left" or "right"
    wallSide = "left"

    ctrl = Follower(Bot, wall_side=wallSide)

    try:
        while True:
            l_rpm, r_rpm = ctrl.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)

            # Front obstacle → rotate with tapered 90 and resume
            f = sense_front(Bot)
            if f < FRONT_STOP_MM:
                # perform rotation here? Already handled in mode, so just sleep
                pass

            time.sleep(DT)

    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)


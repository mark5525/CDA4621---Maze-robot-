
"""
HamBot — Task 2 (Wall Following) — Smooth Corner Wrap (constant-curvature hold)

Fix for "wavy straight line after first turn into corner":
- On WRAP entry, compute a CCW/CW curvature (turn RPM) from side opening and its rate.
- Hold that curvature with an EMA ("turn hold") and enforce a MIN turn toward the wall.
- During WRAP, side PD is heavily downscaled (or zeroed) so it doesn't fight the wrap.
- Exit WRAP only after diagonal capture or side is back near target and opening has slowed.

Use: set wallSide = "left" or "right".
"""

import time, math
from HamBot.src.robot_systems.robot import HamBot

DT = 0.032

# Distances
TARGET_MM        = 300.0
NO_WALL_MM       = 2000.0

# Speeds / scaling
CRUISE_RPM       = 19.0
SEARCH_RPM       = 16.0
ROT_RPM          = 22.0
ROT_MIN_RPM      = 6.0
STEER_TO_RPM     = 0.24
TURN_CAP_RPM     = 8.0
SLEW_RPM_PER_TIK = 1.4

# Front thresholds
FRONT_STOP_MM    = 280.0
FRONT_SLOW_MM    = 460.0
ROT_EXIT_MM      = 340.0

# Side PD (no integral) for FOLLOW
Kp = 0.12
Kd = 2.00
D_CLIP = 1500.0

# Side smoothing
EMA_ALPHA        = 0.28

# Corner wrap detection
WRAP_OPEN_MM     = 100.0        # side - target
WRAP_DS_TRIG     = 45.0         # opening rate per step
WRAP_MIN_T       = 0.22
WRAP_MAX_T       = 1.10
WRAP_SPEED_FAC   = 0.45         # slower during wrap
DIAG_CAPTURE_MM  = 480.0        # ~sqrt(2)*300

# Smooth-wrap (curvature hold) gains
WRAP_K_OPEN      = 0.0080       # -> turn from (s - (target+open))
WRAP_K_RATE      = 0.0055       # -> turn from ds
WRAP_MIN_TURN    = 3.8          # MIN turn RPM toward chosen wall during wrap
WRAP_TURN_ALPHA  = 0.25         # EMA for turn hold
WRAP_PD_SCALE    = 0.25         # scale down PD in wrap (0 disables)

# Search
LOST_SIDE_T      = 0.28
SEARCH_TURN_RPM  = 4.5

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
    k = min(keep, len(xs))
    return sum(xs[:k]) / k

# Sensors
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

        # wrap turn hold
        self.wrap_turn_hold = 0.0

        self.l_slew = SlewLimiter(SLEW_RPM_PER_TIK)
        self.r_slew = SlewLimiter(SLEW_RPM_PER_TIK)

    def _front_band_speed(self, f_mm):
        if f_mm <= FRONT_STOP_MM: return 0.0
        if f_mm >= FRONT_SLOW_MM: return CRUISE_RPM
        a = (f_mm - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM)
        return CRUISE_RPM * max(0.0, min(1.0, a))

    def _rotate_cw(self):  # left-follow
        return (+ROT_RPM, -ROT_RPM)

    def _rotate_ccw(self): # right-follow
        return (-ROT_RPM, +ROT_RPM)

    def _wrap_ff_turn(self, s, ds, turn_sign):
        # Feed-forward turn request from opening and rate
        if s == float('inf'):
            e_open = WRAP_OPEN_MM
        else:
            e_open = max(0.0, s - (TARGET_MM + WRAP_OPEN_MM))
        ff = WRAP_K_OPEN*e_open + WRAP_K_RATE*max(0.0, ds)
        # Convert to turn RPM and apply sign toward chosen wall
        turn = STEER_TO_RPM * ff * turn_sign
        # Enforce MIN magnitude toward the wall
        min_signed = WRAP_MIN_TURN * (1 if turn_sign > 0 else -1)
        # If turn is too small or wrong direction, clamp to min_signed
        if turn_sign > 0:
            if turn < min_signed: turn = min_signed
        else:
            if turn > min_signed: turn = min_signed
        # Cap magnitude
        if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
        if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
        return turn

    def step(self):
        f = sense_front(self.bot)
        dL = sense_diag_left(self.bot)

        # pick side
        if self.wall_side == "left":
            s_raw = sense_left(self.bot)
            turn_sign = +1   # +turn => CCW
        else:
            s_raw = sense_right(self.bot)
            turn_sign = -1   # +turn => CW

        side_seen = s_raw < NO_WALL_MM

        # EMA side
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
                open_far  = (s != float('inf')) and ((s - TARGET_MM) > WRAP_OPEN_MM)
                open_fast = ds > WRAP_DS_TRIG
                if (open_far or open_fast) and f > FRONT_STOP_MM:
                    self.mode = "wrap"; self.wrap_t0 = now
                    # initialize wrap turn hold
                    init_turn = self._wrap_ff_turn(s, ds, turn_sign)
                    self.wrap_turn_hold = init_turn
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

            # Update wrap turn hold (EMA, and do not let it change direction)
            new_ff = self._wrap_ff_turn(s, ds, turn_sign)
            self.wrap_turn_hold = (1.0 - WRAP_TURN_ALPHA)*self.wrap_turn_hold + WRAP_TURN_ALPHA*new_ff
            # Prevent sign flip (always turn toward the wall during wrap)
            if turn_sign > 0 and self.wrap_turn_hold < WRAP_MIN_TURN: self.wrap_turn_hold = WRAP_MIN_TURN
            if turn_sign < 0 and self.wrap_turn_hold > -WRAP_MIN_TURN: self.wrap_turn_hold = -WRAP_MIN_TURN

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
            if self.wall_side == "left":
                l_cmd, r_cmd = self._rotate_cw()
            else:
                l_cmd, r_cmd = self._rotate_ccw()
            l_cmd *= 0.95; r_cmd *= 0.95

        elif self.mode == "search":
            base = SEARCH_RPM
            turn = SEARCH_TURN_RPM * turn_sign
            l_cmd = base - turn
            r_cmd = base + turn

        else:
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

            # Wrap turn (constant curvature hold) + scaled PD
            if self.mode == "wrap":
                base *= WRAP_SPEED_FAC
                turn_wrap = self.wrap_turn_hold
                steer = WRAP_PD_SCALE*steer_pd   # PD subdued in wrap
                turn_pd = STEER_TO_RPM * steer * (1 if turn_sign > 0 else -1)  # signed toward wall
                turn = turn_wrap + turn_pd
            else:
                turn = STEER_TO_RPM * steer_pd * (1 if turn_sign > 0 else -1)

            # Cap
            if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
            if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM

            l_cmd = base - turn
            r_cmd = base + turn

        l_out = saturation(self.bot, self.l_slew.step(l_cmd))
        r_out = saturation(self.bot, self.r_slew.step(r_cmd))
        return l_out, r_out

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    wallSide = "left"  # or "right"
    ctrl = Follower(Bot, wall_side=wallSide)

    try:
        while True:
            l_rpm, r_rpm = ctrl.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)
            time.sleep(DT)
    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)

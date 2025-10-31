
"""
HamBot — Task 2 (From Scratch) — Wall Following with 250 mm side & front targets

Meets rubric:
1) Maintains consistent contact with the chosen wall (left or right).
2) On a frontal obstacle (< 250 mm), rotates 90° and resumes following the same wall.
3) If the wall ends (sharp corner), wraps smoothly and continues following the same wall.
4) Can run left or right wall following (set wall_side below).

Key ideas:
- Side PID (PD only) computes a *signed* steer from side distance error (target = 250 mm).
- Base forward RPM is throttled by the front distance (target = 250 mm).
- Corners (when side opens fast/far or is lost) use a short, constant-curvature WRAP with slow-down.
- Rotations are tapered near target so 90° does not overshoot.
"""

import time, math
from HamBot.src.robot_systems.robot import HamBot

# ========================
# Timing & Targets
# ========================
DT = 0.032
SIDE_TARGET_MM  = 250.0
FRONT_TARGET_MM = 250.0

# ========================
# Speeds / limits
# ========================
CRUISE_RPM         = 18.0     # base forward speed on clear path
SEARCH_RPM         = 16.0
ROTATE_RPM         = 22.0
ROTATE_MIN_RPM     = 6.0
TURN_CAP_RPM       = 8.0      # cap on differential RPM (tighter arcs if larger)
STEER_TO_RPM       = 0.24     # map PD "steer" to differential RPM
SLEW_RPM_PER_TICK  = 1.4

# ========================
# Front thresholds (based on FRONT_TARGET_MM)
# ========================
FRONT_STOP_MM      = FRONT_TARGET_MM
FRONT_SLOW_MM      = FRONT_TARGET_MM + 180.0  # start slowing before stopping
ROTATE_EXIT_MM     = FRONT_TARGET_MM + 120.0  # how clear before exiting rotate

# ========================
# Side PD (no integral) — calmer values for straight walls
# ========================
KP_SIDE = 0.10
KD_SIDE = 1.90
DERIV_CLIP = 1500.0

# ========================
# Side smoothing
# ========================
EMA_ALPHA = 0.30    # 0..1, higher = faster to react
NO_WALL_MM = 2000.0 # if side distance exceeds this, treat as "no wall"

# ========================
# Corner wrap (simple & smooth)
# ========================
WRAP_OPEN_MM   = 90.0     # enter wrap if side exceeds target by > this
WRAP_DS_TRIG   = 35.0     # or if side increases faster than this per step
WRAP_MIN_TIME  = 0.12     # min time in wrap (s)
WRAP_MAX_TIME  = 0.80     # max time (s)
WRAP_SPEED_FAC = 0.45     # slow during wrap
WRAP_MIN_TURN  = 4.0      # minimum turn toward the wall (RPM) during wrap
WRAP_TURN_ALPHA= 0.45     # EMA to hold a constant curvature turn
WRAP_K_OPEN    = 0.0090   # curvature from opening size
WRAP_K_RATE    = 0.0060   # curvature from opening rate
DIAG_CAPTURE_MM= 420.0    # when diagonal gets this close, exit wrap (new wall seen)

# ========================
# Helpers
# ========================
def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class Slew:
    def __init__(self, max_delta):
        self.prev = 0.0
        self.maxd = float(max_delta)
    def step(self, target):
        d = target - self.prev
        if d >  self.maxd: d =  self.maxd
        if d < -self.maxd: d = -self.maxd
        self.prev += d
        return self.prev

def robust_min(vals, keep=5):
    xs = [v for v in vals if v and v > 0]
    if not xs: return float('inf')
    xs.sort()
    k = min(keep, len(xs))
    return sum(xs[:k]) / k

# ========================
# Sensor helpers (0 back, 90 left, 180 front, 270 right)
# ========================
def front_mm(bot):
    scan = bot.get_range_image()
    return robust_min(scan[176:186], keep=5)

def side_mm(bot, side):
    scan = bot.get_range_image()
    if side == "left":
        return robust_min(scan[84:96], keep=7)
    else:
        return robust_min(scan[264:276], keep=7)

def diag_mm(bot, side):
    scan = bot.get_range_image()
    if side == "left":
        return robust_min(scan[128:142], keep=5)   # ~135°
    else:
        return robust_min(scan[218:232], keep=5)   # ~225°

# ========================
# Controller
# ========================
class WallFollower:
    def __init__(self, bot, wall_side="left"):
        self.bot = bot
        self.side = wall_side   # "left" or "right"

        # EMA of side distance + last values for derivative
        self.side_ema = None
        self.prev_side = None
        self.prev_err  = 0.0

        # Modes: "follow", "wrap", "rotate", "search"
        self.mode = "follow"
        self.t0_wrap = None
        self.t0_rotate = None
        self.lost_since = None

        # Constant-curvature turn held during wrap
        self.turn_hold = 0.0

        # Slew limiters for clean motor commands
        self.l_slew = Slew(SLEW_RPM_PER_TICK)
        self.r_slew = Slew(SLEW_RPM_PER_TICK)

    # --- low-level helpers ---
    def _front_speed(self, f_mm):
        if f_mm <= FRONT_STOP_MM: return 0.0
        if f_mm >= FRONT_SLOW_MM: return CRUISE_RPM
        a = (f_mm - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM)
        return CRUISE_RPM * max(0.0, min(1.0, a))

    def _rotate_pair(self):
        # CW for left-follow; CCW for right-follow
        if self.side == "left":
            return (+ROTATE_RPM, -ROTATE_RPM)
        else:
            return (-ROTATE_RPM, +ROTATE_RPM)

    def _wrap_feedforward(self, s, ds, sign):
        # sign: +1 turn CCW (toward left); -1 turn CW (toward right)
        if s == float('inf'):
            e_open = WRAP_OPEN_MM
        else:
            e_open = max(0.0, s - (SIDE_TARGET_MM + WRAP_OPEN_MM))
        ff = WRAP_K_OPEN * e_open + WRAP_K_RATE * max(0.0, ds)
        turn = STEER_TO_RPM * ff * sign
        # Minimum turn toward the wall
        min_signed = WRAP_MIN_TURN * (1 if sign > 0 else -1)
        if sign > 0:
            if turn < min_signed: turn = min_signed
        else:
            if turn > min_signed: turn = min_signed
        # Cap
        if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
        if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
        return turn

    # --- main step ---
    def step(self):
        f = front_mm(self.bot)
        s_raw = side_mm(self.bot, self.side)
        d = diag_mm(self.bot, self.side)
        sign = +1 if self.side == "left" else -1
        side_seen = s_raw < NO_WALL_MM

        # EMA of side
        if s_raw != float('inf'):
            if self.side_ema is None: self.side_ema = s_raw
            else: self.side_ema = EMA_ALPHA*s_raw + (1.0-EMA_ALPHA)*self.side_ema
        s = self.side_ema if self.side_ema is not None else s_raw

        # side opening rate ds
        ds = 0.0
        if self.prev_side is not None and s != float('inf'):
            ds = s - self.prev_side
        self.prev_side = s if s != float('inf') else self.prev_side

        # lost-wall timer
        now = time.time()
        if side_seen: self.lost_since = None
        else:
            if self.lost_since is None: self.lost_since = now

        # --- mode transitions ---
        if self.mode == "follow":
            if f < FRONT_STOP_MM:
                self.mode = "rotate"; self.t0_rotate = now
            else:
                open_far  = (s != float('inf')) and ((s - SIDE_TARGET_MM) > WRAP_OPEN_MM)
                open_fast = ds > WRAP_DS_TRIG
                if (open_far or open_fast) and f > FRONT_STOP_MM:
                    self.mode = "wrap"; self.t0_wrap = now
                    self.turn_hold = self._wrap_feedforward(s, ds, sign)

        elif self.mode == "rotate":
            t = now - (self.t0_rotate or now)
            can_exit  = t >= 0.26
            must_exit = t >= 0.90
            clear     = f > ROTATE_EXIT_MM
            diag_ok   = d < DIAG_CAPTURE_MM
            if must_exit or (can_exit and (clear or side_seen or diag_ok)):
                self.mode = "follow"; self.t0_rotate = None

        elif self.mode == "wrap":
            t = now - (self.t0_wrap or now)
            done_time   = t >= WRAP_MIN_TIME
            too_long    = t >= WRAP_MAX_TIME
            diag_close  = d < DIAG_CAPTURE_MM
            side_caught = (s != float('inf')) and abs(s - SIDE_TARGET_MM) < 0.6*WRAP_OPEN_MM
            if f < FRONT_STOP_MM:
                self.mode = "rotate"; self.t0_rotate = now; self.t0_wrap = None
            elif too_long or (done_time and (diag_close or side_caught)):
                self.mode = "follow"; self.t0_wrap = None
            else:
                # Update constant-curvature turn hold
                new_ff = self._wrap_feedforward(s, ds, sign)
                self.turn_hold = (1.0 - WRAP_TURN_ALPHA)*self.turn_hold + WRAP_TURN_ALPHA*new_ff

        # --- command compute ---
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_pair()
            # small taper to prevent overshoot; loop control in main handles tapering with dt
            l_cmd *= 0.95; r_cmd *= 0.95

        else:
            # FOLLOW or WRAP
            base = self._front_speed(f)

            # Side PD (signed steer)
            if side_seen and s != float('inf'):
                e = s - SIDE_TARGET_MM
                dterm = (e - self.prev_err) / DT
                if dterm >  DERIV_CLIP: dterm =  DERIV_CLIP
                if dterm < -DERIV_CLIP: dterm = -DERIV_CLIP
                steer_pd = KP_SIDE*e + KD_SIDE*dterm
                self.prev_err = e
            else:
                steer_pd = 0.0

            if self.mode == "wrap":
                base *= WRAP_SPEED_FAC
                turn = self.turn_hold + (STEER_TO_RPM * 0.22 * steer_pd * (1 if sign > 0 else -1))
            else:
                turn = STEER_TO_RPM * steer_pd * (1 if sign > 0 else -1)

            # clamp and map to wheels
            if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
            if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
            l_cmd = base - turn
            r_cmd = base + turn

        # Slew + saturation
        l_out = saturation(self.bot, self.l_slew.step(l_cmd))
        r_out = saturation(self.bot, self.r_slew.step(r_cmd))
        return l_out, r_out

# ========================
# Rotation utility (tapered ~90°)
# ========================
def rotate_90(bot, wall_side):
    """
    For LEFT-wall follow, rotate **clockwise** 90° (right turn).
    For RIGHT-wall follow, rotate **counter-clockwise** 90° (left turn).
    """
    target_deg = 90.0
    sign = +1 if wall_side == "left" else -1  # +: CW (L+ R-), -: CCW (L- R+)

    start = bot.get_heading()
    while True:
        cur = bot.get_heading()
        delta = (cur - start + 540) % 360 - 180
        prog = abs(delta); rem = max(0.0, target_deg - prog)
        if rem <= 2.0:
            break

        scale = max(ROTATE_MIN_RPM/ROTATE_RPM, min(1.0, rem/target_deg))
        rpm = ROTATE_RPM * scale
        bot.set_left_motor_speed( sign * rpm)
        bot.set_right_motor_speed(-sign * rpm)
        time.sleep(DT)

    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)

# ========================
# Main
# ========================
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    # Choose wall side here ("left" or "right") to run each maze twice if needed
    wall_side = "left"

    ctrl = WallFollower(Bot, wall_side=wall_side)

    try:
        while True:
            # primary step
            l_rpm, r_rpm = ctrl.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)

            # Handle rotation here when front is blocked (explicit rotate call)
            f = front_mm(Bot)
            if f < FRONT_STOP_MM:
                # Stop motors first for crisp rotate
                Bot.set_left_motor_speed(0.0)
                Bot.set_right_motor_speed(0.0)
                rotate_90(Bot, wall_side)

            time.sleep(DT)

    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)
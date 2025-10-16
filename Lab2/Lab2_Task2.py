import time
from collections import deque
# ... your existing imports + code above remain ...

# ---------- NEW: sector stats + safety monitor ----------
# Define forward/diagonal sectors (indices for ~360-pt scan where ~180 ≈ front)
SECT = {
    "FRONT_WIDE":  (165, 195),   # ~30°
    "FRONT_LEFT":  (150, 175),   # diagonal left
    "FRONT_RIGHT": (185, 210),   # diagonal right
}

def _sector_stats(scan, a0, a1):
    """Robust stats for a scan sector [a0:a1). Returns (median, p20, count)."""
    vals = [d for d in scan[a0:a1] if d and d > 0]
    if not vals:
        return None, None, 0
    vals.sort()
    n = len(vals)
    med = vals[n // 2]
    p20 = vals[max(0, int(0.2 * n) - 1)]
    return med, p20, n

class WallGuard:
    """
    Tracks front distance history to compute TTC and apply hysteresis.
    hard_min: 'slam brakes' distance [mm]
    warn_min: start aggressive slow-down [mm]
    ttc_thresh: seconds to impact to escalate to hard stop
    frames: persistence for noise rejection
    """
    def __init__(self, hard_min=250, warn_min=500, ttc_thresh=0.8, frames=2):
        self.hard_min = hard_min
        self.warn_min = warn_min
        self.ttc_thresh = ttc_thresh
        self.frames = frames
        self.fwd_hist = deque(maxlen=12)   # store conservative front distance (min of p20s)
        self.time_hist = deque(maxlen=12)

    def assess(self, scan, now):
        # Stats from front & diagonals (use p20 for obstacle-leaning estimate)
        _, p20F, nF  = _sector_stats(scan, *SECT["FRONT_WIDE"])
        _, p20FL, nL = _sector_stats(scan, *SECT["FRONT_LEFT"])
        _, p20FR, nR = _sector_stats(scan, *SECT["FRONT_RIGHT"])

        valid_p20 = [p for p in (p20F, p20FL, p20FR) if p is not None]
        if not valid_p20:
            return False, False, None, None, p20FL, p20FR  # no data

        p20min = min(valid_p20)  # conservative forward clearance
        self.fwd_hist.append(float(p20min))
        self.time_hist.append(float(now))

        # Derivative-based TTC from distance history
        ttc = None
        if len(self.fwd_hist) >= 2:
            d2, d1 = self.fwd_hist[-1], self.fwd_hist[-2]
            t2, t1 = self.time_hist[-1], self.time_hist[-2]
            dt = max(1e-3, t2 - t1)
            v_approach = (d1 - d2) / dt  # + if getting closer
            # require meaningful approach speed to avoid division by noise
            if v_approach > 40.0:  # mm/s threshold
                ttc = d2 / v_approach

        # Base gating
        hard = (p20min <= self.hard_min)
        warn = (p20min <= self.warn_min)

        # Escalate based on TTC
        if ttc is not None and ttc < self.ttc_thresh:
            hard = True

        # Persistence (hysteresis) to avoid one-frame spikes
        def persisted(thresh, k):
            src = list(self.fwd_hist)[-k:]
            return len(src) >= k and all(d <= thresh for d in src)

        if not hard and persisted(self.hard_min + 30, self.frames):
            hard = True
        if not hard and not warn and persisted(self.warn_min, self.frames):
            warn = True

        return hard, warn, p20min, ttc, p20FL, p20FR

# ---------- NEW: post-turn cooldown helper ----------
class Cooldown:
    def __init__(self, seconds=0.35):
        self.seconds = seconds
        self._last = 0.0
    def ping(self):
        self._last = time.monotonic()
    def active(self):
        return (time.monotonic() - self._last) < self.seconds


if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 40
#
    side_follow = "right"           # "left" or "right"
    desired_front_distance = 300    # mm (still used by forward_PID)
    desired_side_distance  = 300    # mm

    # ---------- NEW: safety & cooldown ----------
    guard = WallGuard(hard_min=260, warn_min=520, ttc_thresh=0.85, frames=2)
    turn_cd = Cooldown(seconds=0.40)

    # Small hysteresis so we don't false-trigger on side gaps
    no_side_counter = 0
    NO_SIDE_THRESH = 2  # need 2 consecutive frames missing the wall

    while True:
        scan = Bot.get_range_image()
        now  = time.monotonic()

        # FRONT distance (median for stability, still used elsewhere)
        front_med = _median_positive(scan[173:187])
        forward_distance = front_med if front_med is not None else float("inf")

        # SIDE visibility
        if side_follow == "left":
            side_med = _median_positive(scan[90:108])
        else:
            side_med = _median_positive(scan[252:270])

        # Corner detection: side wall disappears consecutively
        if side_med is None:
            no_side_counter += 1
        else:
            no_side_counter = 0

        # --- Corner turn when side truly disappears ---
        if no_side_counter >= NO_SIDE_THRESH and not turn_cd.active():
            Bot.stop_motors()
            time.sleep(0.08)
            rotation(Bot, -90 if side_follow == "left" else +90, pivot_rpm=8)
            turn_cd.ping()
            no_side_counter = 0
            continue

        # ---------- NEW: robust front hazard assessment ----------
        hard, warn, p20min, ttc, p20FL, p20FR = guard.assess(scan, now)

        # Decide safest turn direction on hard stop:
        # by default, keep the side-follow rule; override if diagonals are very asymmetric.
        def safest_turn_deg():
            base = (-90 if side_follow == "left" else +90)  # keep your handedness
            if (p20FL is not None) and (p20FR is not None) and abs(p20FL - p20FR) > 100:
                # Turn AWAY from the nearer diagonal
                return (+90) if (p20FL < p20FR) else (-90)
            return base

        # --- HARD STOP / ESCAPE TURN ---
        if hard and not turn_cd.active():
            Bot.stop_motors()
            # brief brake to kill residual momentum
            Bot.set_left_motor_speed(0); Bot.set_right_motor_speed(0)
            time.sleep(0.05)
            rotation(Bot, safest_turn_deg(), pivot_rpm=10, timeout_s=2.0)
            turn_cd.ping()
            continue

        # --- Forward command (with smart slow-down if WARN) ---
        base_fwd = forward_PID(Bot, f_distance=desired_front_distance, kp=0.25)

        if warn and p20min is not None:
            # Scale speed smoothly based on clearance within [hard_min, warn_min]
            num = max(0.0, p20min - guard.hard_min)
            den = max(1.0, guard.warn_min - guard.hard_min)
            # alpha in [0.15, 0.75]: never fully stop here; hard-stop handles that case
            alpha = 0.15 + 0.60 * (num / den)
            base_fwd *= alpha

        # --- Side correction (gentle) ---
        delta = side_PID(Bot, side_follow=side_follow,
                         side_distance=desired_side_distance,
                         kp=0.05, deadband=15)

        # Rate-limit steering so it can't yank the robot
        steer_cap = min(12.0, max(8.0, 0.5 * abs(base_fwd)))
        if   delta >  steer_cap: delta =  steer_cap
        elif delta < -steer_cap: delta = -steer_cap

        # Compose wheel speeds
        left_v  = base_fwd
        right_v = base_fwd
        if side_follow == "left":
            left_v  = saturation(Bot, left_v  - delta)
            right_v = saturation(Bot, right_v + delta)
        else:
            left_v  = saturation(Bot, left_v  + delta)
            right_v = saturation(Bot, right_v - delta)

        Bot.set_right_motor_speed(right_v)
        Bot.set_left_motor_speed(left_v)

        time.sleep(0.05)  # ~20 Hz

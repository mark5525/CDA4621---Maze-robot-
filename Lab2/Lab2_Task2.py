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

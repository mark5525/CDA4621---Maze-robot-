class Defintions:
    def __init__(self):
        # --- Side PD (same vibe, calm-ish) ---
        self.Kp_side = 0.85
        self.Kd_side = 8.5
        self.StopBand = 15.0

        # Asymmetric shaping
        self.Kp_far_scale   = 0.55
        self.Kp_close_scale = 1.10
        self.Kd_far_scale   = 0.70
        self.Kd_close_scale = 1.00

        # --- Speed profile ---
        self.BaseMin  = 20.0
        self.BaseMax  = 55.0
        self.BaseGain = 0.14

        # NEW: base ramp & smoothing
        self.BaseLPAlpha = 0.25    # low-pass weight for base
        self.BaseSlew    = 3.0     # max base rpm change per cycle
        self.prev_base   = self.BaseMin

        # Steering limits/smoothing
        self.SteerFrac    = 0.55
        self.SteerMax     = 16.0
        self.ForwardFloor = 8.0
        self.SteerLPAlpha = 0.35
        self.steer_lp     = 0.0

        # Robust "no wall" persistence
        self.no_wall_count           = 0
        self.no_wall_frames_required = 3
        self.no_wall_clear_frames    = 2

        # Corner wrap parameters
        self.WrapDiagClear   = 450
        self.WrapSideRiseMM  = 60
        self.WrapBiasMax     = 14.0
        self.WrapBiasGain    = 0.03
        self.WrapHoldSec     = 0.35
        self.wrap_until_ts   = 0.0
        self.prev_side_meas  = 300.0

        # NEW: tame base & bias during wrap
        self.WrapBaseScale   = 0.55   # scale base while wrap active
        self.WrapBaseMax     = 32.0   # cap base during wrap
        self.WrapBiasLPAlpha = 0.35   # ramp-in for wrap bias
        self.wrap_bias_lp    = 0.0

        self.prev_err_side = 0.0
        self.Timestep = dt

    def side_PID(self, wall="left", target_mm=300):
        scan = bot.get_range_image()
        if not isinstance(scan, list) or len(scan) < 300:
            return 0.0, 0.0

        # Side & diag windows
        if wall == "left":
            d_primary  = median_valid_mm(scan[85:95])
            d_fallback = median_valid_mm(scan[95:105])
            d_diag     = median_valid_mm(scan[100:120])
        else:
            d_primary  = median_valid_mm(scan[265:275])
            d_fallback = median_valid_mm(scan[255:265])
            d_diag     = median_valid_mm(scan[240:260])

        d_side = min(d_primary, d_fallback)

        # --- No-wall persistence -> gentle search arc
        if d_side >= NO_WALL_THRESH:
            self.no_wall_count = min(self.no_wall_frames_required, self.no_wall_count + 1)
        else:
            self.no_wall_count = max(0, self.no_wall_count - self.no_wall_clear_frames)

        if self.no_wall_count >= self.no_wall_frames_required:
            base = 24.0
            bias = 10.0
            if wall == "left":
                left = base + bias; right = base - bias
            else:
                left = base - bias; right = base + bias
            # decay steer
            self.steer_lp = 0.8 * self.steer_lp
            self.prev_side_meas = d_side
            # reset base ramp gently toward cruise
            self.prev_base = 0.8 * self.prev_base + 0.2 * base
            return sat(left), sat(right)

        # --- Corner wrap detection
        now = time.time()
        side_rising = (d_side - self.prev_side_meas) > self.WrapSideRiseMM
        diag_clear  = d_diag > (target_mm + self.WrapDiagClear)
        if (side_rising and diag_clear) or (d_side > target_mm + 80 and diag_clear):
            self.wrap_until_ts = now + self.WrapHoldSec

        wrap_active = now < self.wrap_until_ts

        # --- PD steering on side error (asymmetric)
        err = d_side - target_mm
        if abs(err) <= self.StopBand: err = 0.0

        if err > 0:
            kp = self.Kp_side * self.Kp_far_scale
            kd = self.Kd_side * self.Kd_far_scale
        else:
            kp = self.Kp_side * self.Kp_close_scale
            kd = self.Kd_side * self.Kd_close_scale

        derr = (err - self.prev_err_side) / self.Timestep
        self.prev_err_side = err

        steer_raw = kp * err + kd * derr

        # Wrap bias with smooth ramp-in & direction
        if wrap_active:
            raw_bias = min(self.WrapBiasMax, self.WrapBiasGain * max(0.0, d_diag - target_mm))
        else:
            raw_bias = 0.0
        self.wrap_bias_lp = (1 - self.WrapBiasLPAlpha) * self.wrap_bias_lp + self.WrapBiasLPAlpha * raw_bias
        steer_raw += (self.wrap_bias_lp if wall == "left" else -self.wrap_bias_lp)

        # Low-pass steer
        self.steer_lp = (1 - self.SteerLPAlpha) * self.steer_lp + self.SteerLPAlpha * steer_raw
        steer = self.steer_lp

        # --- Desired base from |err|
        desired_base = min(self.BaseMax, max(self.BaseMin, self.BaseGain * abs(err)))

        # If wrapping, tame base (scale + cap)
        if wrap_active:
            desired_base = min(self.WrapBaseMax, self.WrapBaseScale * desired_base)

        # Low-pass + slew-limit base to prevent surges
        base_lp = (1 - self.BaseLPAlpha) * self.prev_base + self.BaseLPAlpha * desired_base
        delta = max(-self.BaseSlew, min(self.BaseSlew, base_lp - self.prev_base))
        base = self.prev_base + delta
        self.prev_base = base

        # Steering clamps relative to (possibly reduced) base
        steer_cap = min(self.SteerFrac * max(base, 1.0), self.SteerMax)
        steer = math.copysign(min(abs(steer), steer_cap), steer)

        # Mix signed steer into wheels
        if wall == "left":
            left_rpm  = base - steer
            right_rpm = base + steer
        else:
            left_rpm  = base + steer
            right_rpm = base - steer

        # Forward floor to avoid pivots
        left_rpm  = max(left_rpm,  self.ForwardFloor)
        right_rpm = max(right_rpm, self.ForwardFloor)

        self.prev_side_meas = d_side
        return sat(left_rpm), sat(right_rpm)

    # forward_PID and rotate stay as you have them (immediate rotate at 300 mm)



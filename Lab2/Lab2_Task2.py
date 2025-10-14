import time
from HamBot.src.robot_systems.robot import HamBot
import math
from collections import deque
#
class PIDController:
    """PID controller with sensor filtering to reduce oscillation"""
    def __init__(self, kp, kd=0.0, filter_size=5, deadband=0):
        self.kp = kp
        self.kd = kd
        self.deadband = deadband  # Ignore small errors to prevent micro-corrections
        self.prev_error = 0.0
        self.filter = deque(maxlen=filter_size)
        self.derivative_filter = deque(maxlen=3)  # Smooth derivative
    
    def compute(self, actual, target, dt=0.02):
        # Add to filter for smoothing
        self.filter.append(actual)
        
        # Use average for smoother response (less jittery than median)
        filtered_actual = sum(self.filter) / len(self.filter)
        
        # Calculate error
        error = filtered_actual - target
        
        # Apply deadband to prevent tiny oscillations
        if abs(error) < self.deadband:
            error = 0.0
        
        # Proportional term
        p_term = self.kp * error
        
        # Derivative term (dampens oscillations)
        d_term = 0.0
        if dt > 0 and len(self.filter) >= self.filter.maxlen:
            raw_derivative = (error - self.prev_error) / dt
            # Smooth the derivative to prevent spikes
            self.derivative_filter.append(raw_derivative)
            smoothed_derivative = sum(self.derivative_filter) / len(self.derivative_filter)
            d_term = self.kd * smoothed_derivative
        
        self.prev_error = error
        
        return p_term + d_term
    
    def reset(self):
        self.prev_error = 0.0
        self.filter.clear()
        self.derivative_filter.clear()

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm


def forward_PID(Bot, pid_controller, f_distance=300):
    scan = Bot.get_range_image()
    d_range = [a for a in scan[175:180] if a > 0]
    if not d_range:
        return 0.0
    actual = min(d_range)
    rpm_v = pid_controller.compute(actual, f_distance)
    forward_v = saturation(Bot, rpm_v)
    return forward_v

def side_PID(Bot, pid_controller, side_follow, side_distance=300):
    if side_follow == "left":
        # Left side: 90 degrees (perpendicular)
        values = [d for d in Bot.get_range_image()[85:95] if d and d > 0]
    else:
        # Right side: 270 degrees (perpendicular)
        values = [d for d in Bot.get_range_image()[265:275] if d and d > 0]
    if not values:
        return 0.0
    # Use minimum distance to wall
    actual = min(values)
    rpm_v = pid_controller.compute(actual, side_distance)
    return saturation(Bot, rpm_v)


def rotation(Bot, angle, pivot_rpm=18, timeout_s=2.5, desired_front_distance=300, extra_clear=30, consecutive_clear=2):
    def _front_mm():
        scan = Bot.get_range_image()
        vals = [d for d in scan[178:183] if d and d > 0]
        return min(vals) if vals else float("inf")

    clear_thresh = desired_front_distance + extra_clear
    clear_hits = 0
    
    # Estimate time needed for the rotation
    # Reduced time to turn tighter
    min_rotation_time = abs(angle) / 90.0 * 0.65  # Reduced from 0.8 to turn less

    rpm = saturation(Bot, abs(pivot_rpm))

    t0 = time.monotonic()
    while True:
        elapsed = time.monotonic() - t0
        fwd = _front_mm()
        
        # Only check for clearance AFTER minimum rotation time
        if elapsed >= min_rotation_time:
            if fwd >= clear_thresh:
                clear_hits += 1
                if clear_hits >= consecutive_clear:
                    Bot.stop_motors()
                    time.sleep(0.05)  # Shorter pause to resume quickly
                    return
            else:
                clear_hits = 0
        
        if angle < 0:
            Bot.set_left_motor_speed(+rpm)
            Bot.set_right_motor_speed(-rpm)
        else:
            Bot.set_left_motor_speed(-rpm)
            Bot.set_right_motor_speed(+rpm)
        if timeout_s and elapsed > timeout_s:
            Bot.stop_motors()
            time.sleep(0.05)  # Shorter pause to resume quickly
            return
        time.sleep(0.01)



if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 50  # Increased for better speed
    side_follow = "right"
    desired_front_distance = 300
    desired_side_distance = 300
    
    # Debug mode - set to True to see controller values
    DEBUG = True
    
    # Create PID controllers with derivative terms to reduce oscillation
    # kp: proportional gain, kd: derivative gain (dampens oscillations)
    # filter_size: larger = smoother but slower response
    # deadband: ignores small errors to prevent micro-oscillations
    forward_controller = PIDController(kp=0.5, kd=0.3, filter_size=4, deadband=5)
    side_controller = PIDController(kp=0.18, kd=0.25, filter_size=4, deadband=5)

    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        if forward_distance < desired_front_distance:
            rotation(Bot, -90 if side_follow == "left" else 90)
            # Reset controllers after rotation
            forward_controller.reset()
            side_controller.reset()
            continue
        
        forward_velocity = forward_PID(Bot, forward_controller, f_distance=300)
        right_v = forward_velocity
        left_v = forward_velocity
        delta_velocity = side_PID(Bot, side_controller, side_follow=side_follow, side_distance=desired_side_distance)

        # Limit correction but ensure minimum correction capability
        lim = max(abs(forward_velocity) * 0.9, 10)  # At least 10 RPM correction allowed
        if delta_velocity > lim: delta_velocity = lim
        if delta_velocity < -lim: delta_velocity = -lim

        if side_follow == "left":
            left_v = saturation(Bot, left_v - delta_velocity)
            right_v = saturation(Bot, right_v + delta_velocity)
        else:
            left_v = saturation(Bot, left_v + delta_velocity)
            right_v = saturation(Bot, right_v - delta_velocity)

        if DEBUG:
            print(f"Fwd:{forward_velocity:.1f} Delta:{delta_velocity:.1f} L:{left_v:.1f} R:{right_v:.1f}")

        Bot.set_right_motor_speed(right_v)
        Bot.set_left_motor_speed(left_v)

        time.sleep(0.025)  # ~40 Hz - balance between responsiveness and stability








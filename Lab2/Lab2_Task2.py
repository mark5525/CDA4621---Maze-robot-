import time
from HamBot.src.robot_systems.robot import HamBot
import math

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm


def forward_PID(Bot, f_distance = 300, kp = 0.8):
    scan = Bot.get_range_image()
    d_range = [a for a in scan[175:180] if a > 0]
    if not d_range:
        return 0.0
    actual = min(d_range)
    e = actual - f_distance
    rpm_v = kp * e
    forward_v = saturation(Bot, rpm_v)
    return forward_v

def side_PID(Bot, side_follow, side_distance = 300, kp = 0.10):
    if side_follow == "left":
        values = [d for d in Bot.get_range_image()[90:105] if d and d > 0]
    else:
        values = [d for d in Bot.get_range_image()[270:285] if d and d > 0]
    if not values:
        return 0.0
    actual = min(values)
    e = actual - side_distance
    rpm_v = kp * e
    return saturation(Bot, rpm_v)


def rotation(Bot, angle, pivot_rpm = 6, timeout_s = 4.0, desired_front_distance = 300, extra_clear = 40, consecutive_clear = 1):
    def _front_mm():
        scan = Bot.get_range_image()
        vals = [d for d in scan[178:183] if d and d > 0]
        return min(vals) if vals else float("inf")

    clear_thresh = desired_front_distance + extra_clear
    clear_hits = 0

    rpm = saturation(Bot, abs(pivot_rpm))

    t0 = time.monotonic()
    while True:
        fwd = _front_mm()
        if fwd >= clear_thresh:
            clear_hits += 1
            if clear_hits >= consecutive_clear:
                Bot.stop_motors()
                return
        else:
            clear_hits = 0
        if angle < 0:
            Bot.set_left_motor_speed(+rpm)
            Bot.set_right_motor_speed(-rpm)
        else:
            Bot.set_left_motor_speed(-rpm)
            Bot.set_right_motor_speed(+rpm)
        if timeout_s and (time.monotonic() - t0) > timeout_s:
            Bot.stop_motors()
            return
        time.sleep(0.01)

def determine_wall_orientation(Bot, side_follow):
    """Determine which direction the wall is relative to the robot"""
    scan = Bot.get_range_image()
    
    # Check all four directions
    north_values = [d for d in scan[0:15] + scan[345:360] if d and d > 0]  # 0-15° and 345-360°
    south_values = [d for d in scan[165:195] if d and d > 0]  # 165-195°
    east_values = [d for d in scan[75:105] if d and d > 0]   # 75-105°
    west_values = [d for d in scan[255:285] if d and d > 0]  # 255-285°
    
    # Find the closest wall direction
    distances = {}
    if north_values: distances['north'] = min(north_values)
    if south_values: distances['south'] = min(south_values)
    if east_values: distances['east'] = min(east_values)
    if west_values: distances['west'] = min(west_values)
    
    if not distances:
        return None
    
    # Return the direction with the closest wall
    return min(distances, key=distances.get)

def get_correct_turn_direction(side_follow, wall_orientation):
    """Determine correct turn direction based on wall orientation"""
    if wall_orientation is None:
        # Default behavior if we can't determine orientation
        return 90 if side_follow == "left" else -90
    
    # When following left wall and hitting front wall
    if side_follow == "left":
        if wall_orientation == "north":  # Wall to north, turn right
            return 90
        elif wall_orientation == "south":  # Wall to south, turn left
            return -90
        elif wall_orientation == "east":  # Wall to east, turn right
            return 90
        elif wall_orientation == "west":  # Wall to west, turn left
            return -90
    
    # When following right wall and hitting front wall
    else:
        if wall_orientation == "north":  # Wall to north, turn left
            return -90
        elif wall_orientation == "south":  # Wall to south, turn right
            return 90
        elif wall_orientation == "east":  # Wall to east, turn left
            return -90
        elif wall_orientation == "west":  # Wall to west, turn right
            return 90
    
    return 90  # Default fallback

def search_for_wall(Bot, side_follow, search_angle=45, pivot_rpm=8, timeout_s=3.0):
    """Search for wall by rotating gradually until wall is found"""
    def _check_wall():
        scan = Bot.get_range_image()
        if side_follow == "left":
            side_values = [d for d in scan[80:115] if d and d > 0]  # Expanded range
        else:
            side_values = [d for d in scan[260:295] if d and d > 0]  # Expanded range
        return side_values and min(side_values) < 700  # More generous threshold
    
    rpm = saturation(Bot, abs(pivot_rpm))
    t0 = time.monotonic()
    
    # Start with a small rotation to search for wall
    while True:
        if _check_wall():
            Bot.stop_motors()
            return True  # Wall found
            
        # Rotate in the direction that makes sense for wall following
        if side_follow == "left":
            # When following left wall and lose it, turn left to find it
            Bot.set_left_motor_speed(-rpm)
            Bot.set_right_motor_speed(+rpm)
        else:
            # When following right wall and lose it, turn right to find it
            Bot.set_left_motor_speed(+rpm)
            Bot.set_right_motor_speed(-rpm)
            
        if timeout_s and (time.monotonic() - t0) > timeout_s:
            Bot.stop_motors()
            return False  # Wall not found
        time.sleep(0.01)



if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 40
    side_follow = "left"
    desired_front_distance = 300
    desired_side_distance = 300

    while True:
        scan = Bot.get_range_image()
        forward_distance = min([a for a in scan[175:180] if a > 0] or [float("inf")])

        # Check if we can still see the side wall
        if side_follow == "left":
            side_values = [d for d in scan[85:110] if d and d > 0]  # Expanded range
        else:
            side_values = [d for d in scan[265:290] if d and d > 0]  # Expanded range

        # Turn when side wall disappears (reached corner)
        # Also check if wall is too far away (likely lost it)
        wall_lost = not side_values or (side_values and min(side_values) > 600)  # Less aggressive threshold
        if wall_lost:
            Bot.stop_motors()
            time.sleep(0.1)  # Brief pause
            # Search for wall with conservative parameters to avoid overshooting
            wall_found = search_for_wall(Bot, side_follow, pivot_rpm=8, timeout_s=3.0)
            if not wall_found:
                # If wall not found, do a smaller turn to clear corner
                rotation(Bot, -45 if side_follow == "left" else 45, pivot_rpm = 10)
            continue

        # Emergency turn if too close to front wall
        if forward_distance < desired_front_distance:
            # Determine wall orientation and turn accordingly
            wall_orientation = determine_wall_orientation(Bot, side_follow)
            turn_angle = get_correct_turn_direction(side_follow, wall_orientation)
            rotation(Bot, turn_angle, pivot_rpm = 10)
            continue

        forward_velocity = forward_PID(Bot, f_distance=300, kp=0.3)  # Reduced gain = smoother, less oscillation
        right_v = forward_velocity
        left_v = forward_velocity
        delta_velocity = side_PID(Bot, side_follow=side_follow, side_distance=desired_side_distance, kp=0.05)  # Reduced gain = gentler corrections

        lim = max(abs(forward_velocity) * 0.6, 8)  # Reduced correction limit = less aggressive corrections
        if delta_velocity > lim: delta_velocity = lim
        if delta_velocity < -lim: delta_velocity = -lim

        if side_follow == "left":
            left_v = saturation(Bot, left_v - delta_velocity)
            right_v = saturation(Bot, right_v + delta_velocity)
        else:
            left_v = saturation(Bot, left_v + delta_velocity)
            right_v = saturation(Bot, right_v - delta_velocity)

        Bot.set_right_motor_speed(right_v)
        Bot.set_left_motor_speed(left_v)

        time.sleep(0.08)  # ~12 Hz - slower loop = more stable, less oscillation








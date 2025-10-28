"""
HamBot Wall Following PID Controller (Real Robot Version)
Author: Seyoung Kan
Date: 2025-10-10
"""
import time
import math
import numpy as np
from robot_systems.robot import HamBot  # HamBot 라이브러리

# ========================
# HamBot Setup
# ========================
bot = HamBot(lidar_enabled=True, camera_enabled=False)
dt = 0.032

# PID state variables
prev_error = 0
integral = 0
prev_angle_error = 0
angleIntegral = 0


# ========================
# Utility functions
# ========================
def resetPID():
    global prev_error, integral, prev_angle_error, angleIntegral
    print("restPID")
    prev_error = 0
    integral = 0
    prev_angle_error = 0
    angleIntegral = 0


def safe_distance(value, max_range=9.5):
    print("safe_distance")
    if math.isinf(value) or math.isnan(value):
        return max_range
    return min(value, max_range)


def saturation(speed, max_speed=20):
    print("saturation")
    return max(min(speed, max_speed), -max_speed)


# ========================
# PID Controllers
# ========================
def forwardPID(target_distance=0.4):
    global prev_error, integral
    print("ForwardPID")
    lidar = bot.get_range_image()
    actual_distance = np.mean([safe_distance(v) for v in lidar[175:185]]) / 600
    error = actual_distance - target_distance

    Kp = 3.0
    Ki = 0
    Kd = 0.15

    P = Kp * error
    I = Ki * integral
    D = Kd * (error - prev_error) / dt
    prev_error = error
    integral += error * dt

    v = P + I + D
    return saturation(v)


def sidePID(wall="left"):
    print("SidePID")
    global prev_angle_error, angleIntegral
    lidar = bot.get_range_image()
    side_distance = 0.4
    Kp = 0.45
    Ki = 0.00019
    Kd = 1

    actual_left = safe_distance(np.min(lidar[90:115])) / 600
    actual_right = safe_distance(np.min(lidar[265:290])) / 600

    if wall == "left" and actual_left > 2.5:
        return 0.0
    if wall == "right" and actual_right > 2.5:
        return 0.0

    error = (actual_right - side_distance) if wall == "right" else (actual_left - side_distance)
    P = Kp * error
    angleIntegral += error * dt
    angleIntegral = np.clip(angleIntegral, -1.0, 1.0)
    I = Ki * angleIntegral
    D = Kd * (error - prev_angle_error) / dt
    prev_angle_error = error

    angular_velocity = P + I + D
    return saturation(angular_velocity)


# ========================
# Rotation for corners
# ========================
def rotate(radianAngle):
    print("Rotate")
    resetPID()
    base_speed = 1.0
    left_direction = 1 if radianAngle > 0 else -1
    right_direction = -left_direction

    initial_yaw = bot.get_heading()  # radians

    while True:
        current_yaw = bot.get_heading()
        delta = (current_yaw - initial_yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(delta) >= abs(radianAngle):
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            break
        bot.set_left_motor_speed(left_direction * base_speed)
        bot.set_right_motor_speed(right_direction * base_speed)
        time.sleep(dt)
    resetPID()


# ========================
# Wall following
# ========================
def wall_follow(wall="left"):
    print("Wall_Following")
    lidar = bot.get_range_image()
    left_distance = safe_distance(np.min(lidar[90:115])) / 600
    print("LEFT D: ", left_distance)
    right_distance = safe_distance(np.min(lidar[265:290])) / 600
    print("Right D: ", right_distance)
    target = 0.4

    linear_velocity = forwardPID(target_distance=target)
    angular_velocity = sidePID(wall)

    rightv = leftv = linear_velocity

    search_sign = +1 if wall == "right" else -1
    side_distance = right_distance if wall == "right" else left_distance

    if side_distance >= 2.5:
        # No wall detected → gentle arc
        base = 0.6 * linear_velocity
        bias = 0.4
        rightv = base - search_sign * bias
        leftv = base + search_sign * bias
    else:
        # Wall-following PID adjustment
        if wall == "right":
            if right_distance < target:
                rightv = linear_velocity + abs(angular_velocity)
                leftv = linear_velocity - abs(angular_velocity)
            elif right_distance < 2.0:
                rightv = linear_velocity - abs(angular_velocity)
                leftv = linear_velocity + abs(angular_velocity)
        else:
            if left_distance < target:
                rightv = linear_velocity - abs(angular_velocity)
                leftv = linear_velocity + abs(angular_velocity)
            elif left_distance > target and left_distance < 2.0:
                rightv = linear_velocity + abs(angular_velocity)
                leftv = linear_velocity - abs(angular_velocity)

    return saturation(rightv), saturation(leftv)


# ========================
# Main loop
# ========================
wall = "left"

while True:

    lidar = bot.get_range_image()
    if lidar is None or len(lidar) < 360:
        center_idx = len(lidar) // 2
        print(f"Front distance: {lidar[center_idx]:.3f} m")
    else:
        print("No LiDAR data received")

    rightv, leftv = wall_follow(wall)
    bot.set_left_motor_speed(leftv)
    bot.set_right_motor_speed(rightv)

    front_distance = min(lidar[175:185]) / 600  # front
    print(front_distance)
    print("-" * 50)

    if front_distance < 0.45 and wall == "right":
        rotate(-math.pi / 2)
    elif front_distance < 0.45 and wall == "left":
        rotate(math.pi / 2)

    time.sleep(dt)


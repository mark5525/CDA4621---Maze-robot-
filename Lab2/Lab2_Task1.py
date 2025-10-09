from HamBot.src.robot_systems.robot import HamBot
import math

def saturation(robot,v):
    if v >robot.max_motor_velocity:
        return robot.max_motor_velocity
    elif v < -robot.max_motor_velocity:
        return -robot.max_motor_velocity
    else:
        return v


def forward_PID(Bot, forward_distance = 0.5, kp = 3):
    actual = min(Bot.get_lidar_range_image()[175,185])
    e = actual - forward_distance
    forward_v = Bot.saturation(kp *e)
    return forward_v

def side_PID(Bot, side_distance = 0.1, kp = 3, side = "left"):
    if side == "left":
        actual = min(Bot.get_lidar_range_image()[90,115])
        e = abs(actual - side_distance)
        return kp * e

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)

    scan = Bot.get_range_image()
    forward_distance = min(scan[180])
    forward_velocity = forward_PID(kp=3)
    delta_velocity = side_PID(kp=0.1)
    side_distance = min(scan[90])
    # too close
    if side_distance < 2:
        left_v = forward_velocity
        right_v = saturation(forward_velocity - delta_velocity)
    elif side_distance > 2:
        right_v = forward_velocity
        right_v = saturation(forward_velocity - delta_velocity)
    else:
        right_v = forward_velocity
        left_v = forward_velocity

    Bot.set_left_motor_velocity(left_v)
    Bot.set_right_motor_velocity(right_v)




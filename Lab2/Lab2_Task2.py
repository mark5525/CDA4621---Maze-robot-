from HamBot.src.robot_systems.robot import HamBot
from HamBot.src.robot_systems.lidar import Lidar
import time
import math

def saturation(self,v):
    if v >self.max_motor_velocity:
        return self.max_motor_velocity
    elif v < -self.max_motor_velocity:
        return -self.max_motor_velocity
    else:
        return v

def forward_PID(self, f_distance = 0.5, kp = 3):
    actual = min(self.get_lidar_range_image()[175:185])
    e = actual - f_distance
    forward_v = self.sat(kp * e)
    return forward_v

def side_PID(self, s_distance = 0.1, kp = 3, side = "left"):
    if side == "Left":
        actual = min(self.get_lidar_range_image()[90:115])
        e = abs(actual - s_distance)
        return kp * e

def rotate(self, end_bearing, k_p = .5):
    delta = end_bearing - self.get_compass_reading()
    velocity = self.velocity_saturation(k_p * abs(delta), suppress = True)
    if -180 <= delta <= 0 or 180 < delta <= 360:
        self.set_left_motor_velocity(1 * velocity)
        self.set_right_motor_velocity(-1 * velocity)
    elif 0 < delta <= 180 or -360 <= delta < -180:
        self.set_left_motor_velocity(-1 * velocity)
        self.set_right_motor_velocity(1 * velocity)

def calculate_wheel_distance_traveled(self, starting_encoder_position):
    current_encoder_readings = self.get_encoder_readings()
    differences = list(map(operator.sub, current_encoder_readings, starting_encoder_position))
    average_differences = sum(differences) / len(differences)
    average_distance = average_differences * self.wheel_radius
    return average_distance

if __name__ == "__main__":
    robot = HamBot(lidar_enabled = True, camera_enabled = False)
    while robot.experiment_supervisor.step(robot.timestep) != 1:
        forward_distance = min(robot.get_lidar_range_image()[175: 180])
        forward_velocity = robot.forward_PID(kp=3)
        delta_velocity = robot.side_PID(kp=0.1)
        side_distance = abs(robot.get_lidar_range_image()[90:115])
        # too close
        if side_distance < 2:
            left_v = forward_velocity
            right_v = robot.saturation(forward_velocity - delta_velocity)
        elif side_distance > 2:
            right_v = forward_velocity
            right_v = robot.saturation(forward_velocity - delta_velocity)
        else:
            right_v = forward_velocity
            left_v = forward_velocity

        robot.set_left_motor_velocity(left_v)
        robot.set_right_motor_velocity(right_v)


    def rotation(robot.


        degree):
    # have a screenshot

    if forward_distance < 0.5:
        robot.rotation_PID(90)
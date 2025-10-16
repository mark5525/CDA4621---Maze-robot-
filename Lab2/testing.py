from HamBot.src.robot_systems.robot import HamBot
import math
import time
#
def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm

class Defintions():
    def __init__(self):
        self.K_p = 0.2
        self.K_i = 0.6
        self.K_d = 0.8
        self.Timestep = 0.032
        self.Integral = 0
        self.PrevError = 0


    def forward_PID(self, Bot, desired_distance):
        scan = Bot.get_range_image()
        window = [a for a in scan[175:180] if a and a > 0]
        if not window:
            return 0.0

        measured_distance = min(window)
        error = measured_distance - desired_distance
        proportional = self.K_p * error
        self.Integral += error * self.Timestep
        integral = self.Integral * self.K_i
        derive = (error - self.PrevError) / self.Timestep
        derivative = derive * self.K_d
        control = proportional + integral + derivative
        brake_start = 200
        scale = min(1.0, abs(error)/ brake_start)
        control *=scale
        tol = 20
        if abs(error) <= tol:
            self.PrevError = error
            return 0
        self.PrevError = error
        return saturation(Bot, control)




if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    d_distance = 600
    tolerance = 8
    pp = Defintions()
    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        forward_velocity = pp.forward_PID( Bot, d_distance)
        print(forward_velocity)
        if forward_distance != float("inf") and abs(forward_distance - d_distance) <= tolerance:
            Bot.stop_motors()
            break
        Bot.set_left_motor_speed(forward_velocity)
        Bot.set_right_motor_speed(forward_velocity)
        time.sleep(pp.Timestep)
        print(forward_distance)
#



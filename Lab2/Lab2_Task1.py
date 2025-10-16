from HamBot.src.robot_systems.robot import HamBot
import math
import time
def saturation(bot, rpm):
    max_rpm = getattr(Bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm

class Defintions():
    def __init__(self):
        self.K_p = 0.2
        self.K_i = 0.6
        self.K_d = 0
        self.Integral = 0.0
        self.Timestep = 0.032

    def forward_PID(self, Bot, desired_distance):
        scan = Bot.get_range_image()
        window = [a for a in scan[175:180] if a and a > 0]
        if not window:
            return 0.0
        self.Error_Previous = 0
        # measurements
        self.MeasuredDistance = min(window)
        self.DesiredDistance  = desired_distance

        # PID terms
        self.Error = self.MeasuredDistance - self.DesiredDistance
        derr = (self.Error - self.Error_Previous) / self.Timestep
        self.Integral += self.Error * self.Timestep
        self.Proportional = self.K_p * self.Error
        self.Derivative  = self.K_d * derr

        # control (note: NOT P*Error)
        self.Control = self.Proportional + self.K_i*self.Integral + self.Derivative
        out = saturation(Bot, self.Control)

        # prepare for next tick
        self.Error_Previous = self.Error

        return out

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    d_distance = 600
    pp = Defintions()
    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        forward_velocity = pp.forward_PID( Bot, d_distance)
        print(forward_velocity)
        time.sleep(0.1)
        if forward_distance > 620:
            Bot.set_left_motor_speed(forward_velocity)
            Bot.set_right_motor_speed(forward_velocity)
            print(forward_distance)
        elif forward_distance < 580:
            Bot.set_left_motor_speed(forward_velocity)
            Bot.set_right_motor_speed(forward_velocity)
            print(forward_distance)
        else:
            Bot.stop_motors()
            break

        time.sleep(pp.Timestep)

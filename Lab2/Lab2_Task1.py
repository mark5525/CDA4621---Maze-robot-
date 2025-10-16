from HamBot.src.robot_systems.robot import HamBot
import math

def saturation(Bot, rpm):
    max_rpm = getattr(Bot, "max_motor_speed", 60)
    if rpm > max_rpm:
        return max_rpm
    if rpm < -max_rpm:
        return -max_rpm
    return rpm

class Defintions():
    def __init__(self):
        self.DesiredDistance = 0
        self.MeasuredDistance = 0
        self.Error = self.DesiredDistance - self.MeasuredDistance
        self.Error_Previous = 0
        self.K_p = 0.2
        self.K_i = 0.6
        self.K_d = 0.5
        self.Time = 0
        self.Integral = 0.0
        self.Timestep = 0.032
        self.Saturated_Control = saturation(Bot, self.Control)

    def forward_PID(self, Bot, desired_distance):
        scan = Bot.get_range_image()
        window = [a for a in scan[175:180] if a > 0]
        if not window:
            return 0.0

        # Update measurements & error
        self.MeasuredDistance = min(window)
        self.DesiredDistance = desired_distance
        self.Error = self.DesiredDistance - self.MeasuredDistance

        # PID terms
        self.Integral += self.Error * self.Timestep  # accumulate
        derivative = (self.Error - self.Error_Previous) / self.Timestep

        u = (self.K_p * self.Error) + (self.K_i * self.Integral) + (self.K_d * derivative)

        self.Error_Previous = self.Error
        return saturation(Bot, u)

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    d_distance = 600
    pp = Defintions()
    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        forward_velocity = pp.forward_PID( Bot, d_distance)
        print("forward velocity", forward_velocity)
        Bot.set_left_motor_speed(forward_velocity)
        Bot.set_right_motor_speed(forward_velocity)

#
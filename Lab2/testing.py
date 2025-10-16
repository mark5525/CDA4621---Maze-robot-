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
        self.Error_Previous = None
        self.K_p = 0.2
        self.K_i = 0.6
        self.K_d = 0.5
        self.Time = 0
        self.Integral = 0.0
        self.Timestep = 0.032
        self.Saturated_Control = saturation(Bot, self.Control)

    def forward_PID(self, Bot, desired_distance):
        Forward_PID_Values = Defintions()
        scan = Bot.get_range_image()
        window = [a for a in scan[175:180] if a > 0]
        if not window:
            return 0.0
        Forward_PID_Values.MeasuredDistance = min(window)
        Forward_PID_Values.DesiredDistance = desired_distance
        Prev_error = Forward_PID_Values.Error
        self.Proportional = Forward_PID_Values.K_p * Forward_PID_Values.Error
        self.Integral += Forward_PID_Values.Error *self.Timestep
        self.Derivative = (Forward_PID_Values.Error - Forward_PID_Values.Error_Previous) / Forward_PID_Values.Timestep
        self.Control = (Forward_PID_Values.Proportional * Forward_PID_Values.Error) + (Forward_PID_Values.K_i *self.Integral) + (Forward_PID_Values.K_d * self.Derivative)
        Sat_control = saturation(Bot, self.Control)
        Forward_PID_Values.Error_Previous = Prev_error

        return Sat_control

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    d_distance = 600
    pp = Defintions()
    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        forward_velocity = pp.forward_PID( Bot, d_distance)
        print("forward velocity", forward_velocity)
        Bot.set_left_motor_speed(forward_velocity)
        Bot.set_right_motor_speed(forward_velocity)

#
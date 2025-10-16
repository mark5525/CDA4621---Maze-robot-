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
        self.Proportional = self.K_p * self.Error
        self.Integral +=  self.Error * self.Timestep
        self.Derivative = (self.Error - self.Error_Previous) / self.Timestep
        self.Control = (self.Proportional * self.Error) + (self.K_i *self.Integral) + (self.K_d * self.Derivative)
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
        Forward_PID_Values.Proportional = Forward_PID_Values.K_p * Forward_PID_Values.Error
        Forward_PID_Values.Integral += Forward_PID_Values.Error *self.Timestep
        Forward_PID_Values.Derivative = (Forward_PID_Values.Error - Forward_PID_Values.Error_Previous) / Forward_PID_Values.Timestep
        Forward_PID_Values.Control = (Forward_PID_Values.Proportional * Forward_PID_Values.Error) + (Forward_PID_Values.K_i *self.Integral) + (Forward_PID_Values.K_d * self.Derivative)
        Sat_control = Forward_PID_Values.Saturated_Control
        Forward_PID_Values.Error_Previous = Prev_error
        brake_start = 200
        scale = min(1,abs(Forward_PID_Values.Error)/brake_start)

        return Sat_control

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    d_distance = 600
    pp = Defintions()
    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        forward_velocity = pp.forward_PID( Bot, d_distance)
        if forward_velocity == 0:
            break
        print("forward velocity", forward_velocity)
        Bot.set_left_motor_speed(forward_velocity)
        Bot.set_right_motor_speed(forward_velocity)


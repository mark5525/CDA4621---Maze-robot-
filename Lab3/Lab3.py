from Lab2.Lab2_Task2 import WallFollower, rotate_90
import math, time
from HamBot.src.robot_systems.robot import HamBot
import HamBot.src.robot_systems.camera

'''
main functions to use 
 def _capture_loop(self):
        """Continuously capture frames and normalize to TRUE RGB, honoring flip setting."""
        use this to see what is around us 


use this to confirm we are at the target 
 def get_frame(self, copy=True):
        """
        Returns the latest RGB frame.

    if get_frame == "yellow" & lidar distance from the target is between (0.24-0.26 for error margin) 
    bot.stop motors 

     def set_target_colors(self, colors, tolerance=0.05):
        """
        Configure the set of RGB colors to detect as "landmarks".
    this function helps us set the target we are looking for 


    def stop()
    to stop the camera

    def fps()

    to adjust the frames per second, there's two options to have a dyanmic fps and a standardized fps 



    open the camera and set the target colors first. while the robot isn't within 0.25 of the object, 
    have it following a wall till it finds it use get_frame to confirm
    stop the motors then stop the camera 
'''
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=True)
    Bot.max_motor_speed = 60


    Bot.set_target_colors((96, 111, 31)) #yellow
    wall_side = "left"  # or "right"
    ctrl = Bot.WallFollower(Bot, wall_side=wall_side)

    try:
        while True:
            Bot._capture_loop()
            l_rpm, r_rpm = ctrl.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)
            if Bot.find_landmarks and (Bot.front_mm <= 240 or Bot.front_mm >= 260) :
                break
            # explicit rotate on front block
            f = Bot.front_mm(Bot)
            if f < Bot.FRONT_STOP_MM:
                Bot.set_left_motor_speed(0.0)
                Bot.set_right_motor_speed(0.0)
                Bot.rotate_90(Bot, wall_side)
                if Bot.find_landmarks:
                    break
            time.sleep(Bot.DT)


    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)

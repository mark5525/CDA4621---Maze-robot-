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

import math, time
import Lab2_Task2
from HamBot.src.robot_systems import camera
from HamBot.src.robot_systems.robot import HamBot

def first_scan(Bot):
    scan = Bot.get_range_image()
    # Get distances: left (90°), front (180°), right (270°)
    left = scan[90]
    front = scan[180]
    right = scan[270]
    
    match (left, front, right):
        case (l, f, r) if l < 200 and f < 200 and r < 200:
            print("Close on all sides")
        case (l, f, r) if f < 200:
            print("Close in front")
        case (l, f, r) if l < 200:
            print("Close on left")
        case (l, f, r) if r < 200:
            print("Close on right")
        case _:
            print("All clear")

def rotate_360(bot, direction="left", check_landmarks=None):
    """
    Rotate robot 360 degrees while optionally checking for landmarks.
    
    Args:
        bot: HamBot instance
        direction: "left" (counter-clockwise) or "right" (clockwise)
        check_landmarks: Function that returns True if landmark found (optional)
    
    Returns:
        True if landmark was found during rotation, False otherwise
    """
    sign = -1 if direction == "left" else +1  # -1: CCW (left), +1: CW (right)
    start_heading = bot.get_heading()
    ROTATE_RPM = 20.0
    ROTATE_MIN_RPM = 6.0
    DT = 0.032
    total_rotated = 0.0
    last_heading = start_heading
    
    while total_rotated < 360.0:
        # Check for landmarks FIRST - stop immediately if found
        if check_landmarks is not None and check_landmarks():
            bot.set_left_motor_speed(0.0)
            bot.set_right_motor_speed(0.0)
            return True
        
        cur_heading = bot.get_heading()
        
        # Calculate angular change (handles wrap-around at 360/0)
        delta = (cur_heading - last_heading + 180) % 360 - 180
        total_rotated += abs(delta)
        last_heading = cur_heading
        
        # Taper speed as we approach the target
        remaining = 360.0 - total_rotated
        if remaining <= 2.0:
            break
        scale = max(ROTATE_MIN_RPM/ROTATE_RPM, min(1.0, remaining/360.0))
        rpm = ROTATE_RPM * scale
        bot.set_left_motor_speed(sign * rpm)
        bot.set_right_motor_speed(-sign * rpm)
        time.sleep(DT)
    
    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)
    return False

def Landmark_checking(bot):
    """
    Rotate 360 degrees while checking for landmarks.
    Returns True if landmark found, False otherwise.
    """
    return rotate_360(bot, direction="left", 
                      check_landmarks=lambda: len(bot.camera.find_landmarks()) > 0)
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled = True, camera_enabled = True)
    Bot.max_motor_speed = 60

    if(Landmark_checking(Bot) == True):
        print("Landmark detected! Stopping rotation.")
        Bot.set_left_motor_speed(50)
        Bot.set_right_motor_speed(50)
    Bot.camera.set_target_colors((96, 111, 31)) #green
    wall_side = "left"  # or "right"
    ctrl = Lab2_Task2.WallFollower(Bot, wall_side=wall_side)

    try:
        while True:
            l_rpm, r_rpm = ctrl.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)
            if Bot.camera.find_landmarks and (Lab2_Task2.FRONT_STOP_MM <= 240 or Lab2_Task2.FRONT_STOP_MM >= 260) :
                break
            # explicit rotate on front block
            f = Lab2_Task2.front_mm(Bot)
            if f < Lab2_Task2.FRONT_STOP_MM:
                Bot.set_left_motor_speed(0.0)
                Bot.set_right_motor_speed(0.0)
                Lab2_Task2.rotate_90(Bot, wall_side)
                if Bot.camera.find_landmarks:
                    break
            time.sleep(Lab2_Task2.DT)


    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)
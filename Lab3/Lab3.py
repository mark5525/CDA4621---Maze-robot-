import math, time
import Lab2_Task2
from HamBot.src.robot_systems import camera
from HamBot.src.robot_systems.robot import HamBot
#
def first_scan(Bot):
    scan = Bot.get_range_image()
    # Get distances: left (90°), front (180°), right (270°)

def rotate_360(bot, direction="left", check_landmarks=None):

    sign = -1 if direction == "left" else +1
    start_heading = bot.get_heading()
    ROTATE_RPM = 15.0
    ROTATE_MIN_RPM = 5.0
    DT = 0.05
    CHECK_INTERVAL = 0.1
    total_rotated = 0.0
    last_heading = start_heading

    time.sleep(CHECK_INTERVAL)
    if check_landmarks is not None and check_landmarks():
        return True
    
    while total_rotated < 360.0:
        cur_heading = bot.get_heading()

        delta = (cur_heading - last_heading + 180) % 360 - 180
        total_rotated += abs(delta)
        last_heading = cur_heading

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
        time.sleep(CHECK_INTERVAL)

        if check_landmarks is not None and check_landmarks():
            return True
    
    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)
    return False

def Landmark_checking(bot):

    return rotate_360(bot, direction="left", 
                      check_landmarks=lambda: len(bot.camera.find_landmarks()) > 0)
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled = True, camera_enabled = True)
    Bot.max_motor_speed = 60
    Bot.camera.set_target_colors((119, 134, 54))
    landmark_distance = 250


    if(Landmark_checking(Bot) == True):
        print("Landmark detected! Driving to target distance...")
        target = landmark_distance
        v = min(50, getattr(Bot, "max_motor_speed", 60))
        while True:
            scan = Bot.get_range_image()
            window = [a for a in scan[175:180] if a and a > 0]
            if not window:
                break
            actual = min(window)
            if actual <= target:
                break
            Bot.set_left_motor_speed(v)
            Bot.set_right_motor_speed(v)
            time.sleep(0.03)
            if  (Lab2_Task2.FRONT_STOP_MM <= 240 or Lab2_Task2.FRONT_STOP_MM >= 260):
                Bot.set_left_motor_speed(0)
                Bot.set_right_motor_speed(0)
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
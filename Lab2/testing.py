from HamBot.src.robot_systems.robot import HamBot
import time, math

dt = 0.032
previousError = 0
previousAngleError = 0
integral = 0
angleIntegral = 0
initialX = 2
initialY = -2
prevEncoder = 0
radius = 0

def resetPIDStates():
    global previousError, integral, previousAngleError, angleIntegral
    previousError = 0
    integral = 0
    previousAngleError = 0
    angleIntegral = 0

def safeDistance(value, maxRange = 9500):
    if math.isinf(value) or math.isnan(value):
        return maxRange
    return min(value, maxRange)

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

def forwardPID(Bot, targetDistance = 300):
    global previousError, integral
    kp = 3.0
    ki= 0
    kd = 0.15
    actualDistance = sum([safeDistance(v) for v in Bot.get_range_image()[175:180]]) / 10.0
    error = actualDistance - targetDistance
    P = kp * error
    integral += error * dt
    I = integral * ki
    derivative = (error - previousError) / dt
    D = derivative * kd
    previousError = error
    v = P + I + D
    return saturation(Bot, v)

def sidePID(robot, wallSide):
    global previousAngleError, angleIntegral
    sideDistance = 300
    Kp = 5
    Ki = 0
    Kd = 8
    actualLeftDistance = safeDistance(min(Bot.get_range_image()[90:115]))
    actualRightDistance = safeDistance(min(Bot.get_range_image()[265:290]))
    print("Left Distance: ", actualLeftDistance)
    print("Right Distance: ", actualRightDistance)
    if wallSide == "left" and actualLeftDistance > 2500:
        return 0.0
    if wallSide == "right" and actualRightDistance > 2500:
        return 0.0
    if wallSide == "right":
        error = actualRightDistance - sideDistance
    else:
        error = actualLeftDistance - sideDistance
    P = Kp * error
    angleIntegral += error * dt
    angleIntegral = max(min(angleIntegral, 1.0), -1.0)
    I = angleIntegral * Ki
    derivative = (error - previousAngleError) / dt
    D = derivative * Kd
    previousAngleError = error
    angularVelocity = P + I + D
    return saturation(Bot, angularVelocity)

def rotate(radianAngle):
    resetPIDStates()
    baseSpeed = 1
    if radianAngle < 0:
        leftDirection = -1
        rightDirection = 1
    else:
        leftDirection = 1
        rightDirection = -1
    initialPosition = Bot.get_heading()
    while True:
        currentPos = Bot.get_heading()
        delta = (currentPos - initialPosition + 540) % 360 - 180
        if abs(delta) > abs(math.degrees(radianAngle)):
            Bot.set_left_motor_speed(0)
            Bot.set_right_motor_speed(0)
            break
        Bot.set_left_motor_speed(leftDirection * baseSpeed)
        Bot.set_right_motor_speed(rightDirection * baseSpeed)
    resetPIDStates()

def wallFollow(wallSide):
    leftDistance = safeDistance(min(Bot.get_range_image()[90:115]))
    rightDistance = safeDistance(min(Bot.get_range_image()[265:290]))
    targetDistance = 300
    linearVelocity = forwardPID(Bot, targetDistance)
    angularVelocity = sidePID(Bot, wallSide)
    rightV = leftV = linearVelocity
    if wallSide == "right":
        sideDistance = rightDistance
        searchSign = +1
    else:
        sideDistance = leftDistance
        searchSign = -1
    if sideDistance >= 2500:
        base = 0.6 * linearVelocity
        bias = 0.4
        rightV = base - searchSign * bias
        leftV = base + searchSign * bias
    else:
        if wallSide == "right":
            rightV = linearVelocity + abs(angularVelocity)
            leftV = linearVelocity - abs(angularVelocity)
        elif rightDistance > targetDistance and rightDistance < 2000:
            rightV = linearVelocity - abs(angularVelocity)
            leftV = linearVelocity + abs(angularVelocity)
        else:
            if leftDistance < targetDistance:
                rightV = linearVelocity - abs(angularVelocity)
                leftV = linearVelocity + abs(angularVelocity)
            elif leftDistance > targetDistance and leftDistance < 2000:
                rightV = linearVelocity + abs(angularVelocity)
                leftV = linearVelocity - abs(angularVelocity)
    rightV = saturation(Bot, rightV)
    leftV = saturation(Bot, leftV)
    return rightV, leftV

if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    wallSide = "left"

    while True:
        rightV, leftV = wallFollow(wallSide)
        Bot.set_left_motor_speed(leftV)
        Bot.set_right_motor_speed(rightV)
        frontDistance = safeDistance(min(Bot.get_range_image()[175:185]))
        print("Front Distance: ", frontDistance)
        print("-" * 50)
        if frontDistance < 300 and wallSide == "right":
            rotate(-math.pi / 2)
        elif frontDistance < 300 and wallSide == "left":
            rotate(math.pi / 2)


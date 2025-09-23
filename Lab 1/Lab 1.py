from HamBot.src.robot_systems.robot import HamBot
import math
import time


def StraightLineFormula(x1, x2, y1, y2):
    # the starting point is x1, y1
    # the next point is x2, y2
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))


def ArcFormula(RadiusOfCircle, Radians):
    return RadiusOfCircle * Radians


def InnerCircle(WheelVelocity, RadiusOfCurvature, CarMid):
    value = WheelVelocity * ((RadiusOfCurvature - CarMid) / (RadiusOfCurvature + CarMid))
    return value


class Specs:
    CarWidth = 0.184
    CarMidWidth = CarWidth / 2  # m
    WheelDiameter = 0.09  # m
    WheelRadius = WheelDiameter / 2
    RPM = 50


def LinearSpeedToRPMS(LinearSpeed, Radius=Specs.WheelRadius, PI2=math.pi * 2, Seconds=60):
    return ((LinearSpeed * Seconds) / (PI2 * Radius))


def TurnToPosition(heading_target, tolerance= 2):
    poscurrent = Bot.get_heading()  # Get the current heading from the IMU
    turn_angle = (heading_target - poscurrent)  # Calculate the angle difference

    # Normalize the turn_angle to be between -180 and 180 degrees
    if turn_angle > 180:
        turn_angle -= 360
    elif turn_angle < -180:
        turn_angle += 360

    # Check if the robot is within the tolerance
    if abs(turn_angle) <= tolerance:
        print("Robot is already at the target heading.")
        return  # No need to turn

    # Determine the direction of the turn (positive for counterclockwise, negative for clockwise)
    turn_direction = 1 if turn_angle > 0 else -1

    # Apply a proportional control to turn the robot smoothly (you can adjust the turn speed as needed)
    turn_speed = 15 * turn_direction  # You can adjust the constant for speed

    while abs(turn_angle) > tolerance:  # While the turn angle is not within tolerance
        current_heading = Bot.get_heading()  # Update the current heading
        turn_angle = (heading_target - current_heading)  # Recalculate turn angle

        # Normalize turn angle again
        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360

        # Rotate the robot to adjust heading
        if turn_angle > 0:
            Bot.set_left_motor_speed(-turn_speed)  # Turn counterclockwise (left motor inverted)
            Bot.set_right_motor_speed(-turn_speed)
        else:
            Bot.set_left_motor_speed(turn_speed)  # Turn clockwise (left motor inverted)
            Bot.set_right_motor_speed(turn_speed)

        # Allow some time for the motors to adjust
        time.sleep(0.1)

    # Stop the motors once the target heading is reached
    Bot.stop_motors()
    print(f"Robot has reached the target heading: {heading_target} degrees.")


class Waypoints(Specs):
    TotalTime = 0.0
    TotalPathLength = 0.0

    def __init__(self):
        self.PointName = ""
        self.WheelAngularVelocity = 50 * (2 * math.pi / 60)
        self.RobotLinearVelocity = self.WheelAngularVelocity * 0.045
        self.LeftWheelLinearVelocity = 0.0
        self.RightWheelLinearVelocity = 0.0
        self.DistanceTraveled = 0.0
        self.SegmentTime = 0.0
        self.TurnDistance = Specs.CarMidWidth * (math.pi / 2)
        self.Flag = 0

    def WaypointTotals(self):
        if (self.Flag == 1):
            self.DistanceTraveled += self.TurnDistance
            self.Flag = 0
        self.LinearVelocityTotal = (self.LeftWheelLinearVelocity + self.RightWheelLinearVelocity) / 2
        self.SegmentTime = self.DistanceTraveled / self.LinearVelocityTotal
        Waypoints.TotalTime += self.SegmentTime
        Waypoints.TotalPathLength += self.DistanceTraveled

    @classmethod
    def GetTime(cls):
        return cls.TotalTime

    @classmethod
    def GetPathLength(cls):
        return cls.TotalPathLength

    def PrintAll(self):
        print(f"for point :{self.PointName}\n"
              f"The right wheel angular velocity is: {self.RightWheelLinearVelocity:.2f} meters/sec.\n"
              f"The left wheel {self.LeftWheelLinearVelocity:.2f} meters/sec.\n"
              f"Segment distance {self.DistanceTraveled:.2f} meters\n"
              f"and it took {self.SegmentTime:.2f} seconds to travel.\n"
              f"Total path length is {Waypoints.TotalPathLength:.2f} meters\n"
              f"Total travel time is {Waypoints.GetTime():.2f} seconds\n")

    def PrintDuringNavigation(self):
        print(f"Right wheel linear velocity: {self.RightWheelLinearVelocity:.3f} meters/sec "
              f"Left wheel linear velocity: {self.LeftWheelLinearVelocity:.3f} meters/sec"
              f"Distance: {self.DistanceTraveled:.3f} meters"
              f"Time: {self.SegmentTime:.3f} seconds")


# kinematics calculations
# using 50 RPMs
# add segment time to total time
# use time.time()  0r time.perf_counter for the travel time
if __name__ == "__main__":
    p0 = [2.0, -2.0, math.pi]
    p1 = [-1.5, -2.0, math.pi]
    p2 = [-2.0, -1.5, math.pi / 2]
    p3 = [-2.0, -0.5, math.pi / 2]
    p4 = [-1.0, -0.5, (3 * math.pi) / 2]
    p5 = [-0.5, -1.0, (7 * math.pi) / 4]
    p6 = [2.0, -1.0, 0]
    p7 = [2.0, 0.0, math.pi / 2]
    p8 = [0.0, 0.0, math.pi]
    p9 = [0.0, 1.0, math.pi / 2]
    p10 = [-2.0, 1.0, math.pi]
    p11 = [-1.0, 2.0, 0]
    p12 = [1.5, 2.0, 0]
    p13 = [0, 0, 0]
    # UNKNOWN VALUE

    # robot movement
    Bot = HamBot(lidar_enabled=False, camera_enabled=False)

    # UNCOMMENT THE ABOVE SECTION

    # p0 to p1
    P0toP1 = Waypoints()
    P0toP1.PointName = "P0 to P1"
    P0toP1.LeftWheelLinearVelocity = P0toP1.RobotLinearVelocity
    P0toP1.RightWheelLinearVelocity = P0toP1.RobotLinearVelocity
    P0toP1.DistanceTraveled = StraightLineFormula(p0[0], p1[0], p0[1], p1[1])
    P0toP1.WaypointTotals()

    P0toP1.PrintAll()
    # p1 to p2
    P1toP2 = Waypoints()
    P1toP2.PointName = "P1 to P2"
    P1toP2.LeftWheelLinearVelocity = P1toP2.RobotLinearVelocity
    P1toP2.RightWheelLinearVelocity = InnerCircle(P1toP2.RobotLinearVelocity, 0.5, Specs.CarMidWidth)
    P1toP2.DistanceTraveled = ArcFormula(0.5, math.pi / 2)
    P1toP2.WaypointTotals()
    P1toP2.PrintAll()
    # p2 to p3
    P2toP3 = Waypoints()
    P2toP3.PointName = "P2 to P3"
    P2toP3.LeftWheelLinearVelocity = P2toP3.RobotLinearVelocity
    P2toP3.RightWheelLinearVelocity = P2toP3.RobotLinearVelocity
    P2toP3.DistanceTraveled = StraightLineFormula(p2[0], p3[0], p2[1], p3[1])
    P2toP3.WaypointTotals()
    P2toP3.PrintAll()
    # P3 to P4
    P3toP4 = Waypoints()
    P3toP4.PointName = "P3 to P4"
    P3toP4.LeftWheelLinearVelocity = P3toP4.RobotLinearVelocity
    P3toP4.RightWheelLinearVelocity = InnerCircle(P3toP4.RobotLinearVelocity, 0.5, Specs.CarMidWidth)
    P3toP4.DistanceTraveled = ArcFormula(0.5, math.pi)
    P3toP4.WaypointTotals()
    P3toP4.PrintAll()
    # P4 to P5
    P4toP5 = Waypoints()
    P4toP5.PointName = "P4 to P5"
    P4toP5.LeftWheelLinearVelocity = P4toP5.RobotLinearVelocity
    P4toP5.RightWheelLinearVelocity = P4toP5.RobotLinearVelocity
    P4toP5.DistanceTraveled = StraightLineFormula(p4[0], p5[0], p4[1], p5[1])
    P4toP5.WaypointTotals()
    P4toP5.PrintAll()
    # P5 to P6
    P5toP6 = Waypoints()
    P5toP6.PointName = "P5 to P6"
    P5toP6.LeftWheelLinearVelocity = P5toP6.RobotLinearVelocity
    P5toP6.RightWheelLinearVelocity = P5toP6.RobotLinearVelocity
    P5toP6.DistanceTraveled = StraightLineFormula(p5[0], p6[0], p5[1], p6[1])
    P5toP6.Flag = 1
    P5toP6.WaypointTotals()
    P5toP6.PrintAll()
    # P6 to P7
    P6toP7 = Waypoints()
    P6toP7.PointName = "P6 to P7"
    P6toP7.LeftWheelLinearVelocity = P6toP7.RobotLinearVelocity
    P6toP7.RightWheelLinearVelocity = P6toP7.RobotLinearVelocity
    P6toP7.DistanceTraveled = StraightLineFormula(p6[0], p7[0], p6[1], p7[1])
    # change flag direction
    P6toP7.Flag = 1
    P6toP7.WaypointTotals()
    P6toP7.PrintAll()
    # P7 to P8
    P7toP8 = Waypoints()
    P7toP8.PointName = "P7 to P8"
    P7toP8.LeftWheelLinearVelocity = P7toP8.RobotLinearVelocity
    P7toP8.RightWheelLinearVelocity = P7toP8.RobotLinearVelocity
    P7toP8.Flag = 1
    P7toP8.DistanceTraveled = StraightLineFormula(p7[0], p8[0], p7[1], p8[1])
    # P8 to P9
    P8toP9 = Waypoints()
    P8toP9.PointName = "P8 to P9"
    P8toP9.LeftWheelLinearVelocity = P8toP9.RobotLinearVelocity
    P8toP9.RightWheelLinearVelocity = P8toP9.RobotLinearVelocity
    P8toP9.TurnDistance = (Specs.CarMidWidth * (3 * math.pi) / 2)
    P8toP9.PrintAll()
    # P9 to P10
    P9toP10 = Waypoints()
    P9toP10.PointName = "P7 to P8"
    P9toP10.LeftWheelLinearVelocity = P9toP10.RobotLinearVelocity
    P9toP10.RightWheelLinearVelocity = P9toP10.RobotLinearVelocity
    P9toP10.Flag = 1
    P9toP10.DistanceTraveled = StraightLineFormula(p9[0], p10[0], p9[1], p10[1])
    P9toP10.PrintAll()
    # P10 to P11
    P10toP11 = Waypoints()
    P10toP11.PointName = "P10 to P11"
    P10toP11.LeftWheelLinearVelocity = P10toP11.RobotLinearVelocity
    P10toP11.RightWheelLinearVelocity = InnerCircle(P10toP11.RobotLinearVelocity, 1, Specs.CarMidWidth)
    P10toP11.DistanceTraveled = ArcFormula(1, (math.pi / 2))
    P10toP11.WaypointTotals()
    P10toP11.PrintAll()

    # P11 to P12
    P11toP12 = Waypoints()
    P11toP12.PointName = "P11 to P12"
    P11toP12.LeftWheelLinearVelocity = P11toP12.RobotLinearVelocity
    P11toP12.RightWheelLinearVelocity = P11toP12.RobotLinearVelocity
    P11toP12.DistanceTraveled = StraightLineFormula(p11[0], p12[0], p11[1], p12[1])
    P11toP12.WaypointTotals()
    P11toP12.PrintAll()

    # running the robot
    # p0 to p1
    Bot.run_motors_for_seconds(P1toP2.SegmentTime, LinearSpeedToRPMS(P0toP1.LeftWheelLinearVelocity),
                               LinearSpeedToRPMS(P0toP1.RightWheelLinearVelocity))

    # p1 to p2
    Bot.run_motors_for_seconds(P1toP2.SegmentTime, LinearSpeedToRPMS(P1toP2.LeftWheelLinearVelocity),
                               LinearSpeedToRPMS(P1toP2.RightWheelLinearVelocity))
    # p2 to p3
    Bot.run_motors_for_seconds(P2toP3.SegmentTime, LinearSpeedToRPMS(P2toP3.LeftWheelLinearVelocity),
                               LinearSpeedToRPMS(P2toP3.RightWheelLinearVelocity))
    # p3 to p4
    Bot.run_motors_for_seconds(P3toP4.SegmentTime, LinearSpeedToRPMS(P3toP4.LeftWheelLinearVelocity),
                               LinearSpeedToRPMS(P3toP4.RightWheelLinearVelocity))
    # p4 to p5

    TurnToPosition(315)
    Bot.run_motors_for_seconds(P4toP5.SegmentTime, LinearSpeedToRPMS(P4toP5.LeftWheelLinearVelocity), LinearSpeedToRPMS(P4toP5.RightWheelLinearVelocity))
    #p5 to p6
    TurnToPosition(0)
    Bot.run_motors_for_seconds(P5toP6.SegmentTime, LinearSpeedToRPMS(P5toP6.LeftWheelLinearVelocity), LinearSpeedToRPMS(P5toP6.RightWheelLinearVelocity))
    #p6 to p7
    TurnToPosition(90)
    Bot.run_motors_for_seconds(P6toP7.SegmentTime, LinearSpeedToRPMS(P6toP7.LeftWheelLinearVelocity), LinearSpeedToRPMS(P6toP7.RightWheelLinearVelocity))
    #p7 to p8
    TurnToPosition(180)
    Bot.run_motors_for_seconds(P7toP8.SegmentTime, LinearSpeedToRPMS(P7toP8.LeftWheelLinearVelocity), LinearSpeedToRPMS(P7toP8.RightWheelLinearVelocity))
    #p8 to p9
    TurnToPosition(90)
    Bot.run_motors_for_seconds(P8toP9.SegmentTime, LinearSpeedToRPMS(P8toP9.LeftWheelLinearVelocity), LinearSpeedToRPMS(P8toP9.RightWheelLinearVelocity))
    #p9 to p10
    TurnToPosition(180)
    Bot.run_motors_for_seconds(P9toP10.SegmentTime, LinearSpeedToRPMS(P9toP10.LeftWheelLinearVelocity), LinearSpeedToRPMS(P9toP10.RightWheelLinearVelocity))
    #p10 to p11
    Bot.run_motors_for_seconds(P10toP11.SegmentTime, LinearSpeedToRPMS(P10toP11.LeftWheelLinearVelocity), LinearSpeedToRPMS(P10toP11.RightWheelLinearVelocity))
    #p11 to p12
    Bot.run_motors_for_seconds(P11toP12.SegmentTime, LinearSpeedToRPMS(P11toP12.LeftWheelLinearVelocity), LinearSpeedToRPMS(P11toP12.RightWheelLinearVelocity))

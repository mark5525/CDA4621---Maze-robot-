#from robot_systems import HamBot
import math
def StraightLineFormula(x1, x2, y1, y2):
    #the starting point is x1, y1
    #the next point is x2, y2
    return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2 - y1, 2))


class Specs:
    CarWidth = 0.184
    AxisLength = CarWidth/2 #m
    WheelDiameter = 0.09  # m
    WheelRadius = WheelDiameter / 2
    RPM = 50


class Waypoints(Specs):
    TotalTime = 0.0
    TotalPathLength = 0.0

    def __init__(self):
        self.WheelAngularVelocity = 50 * (2 * math.pi / 60) * 0.045
        self.LeftWheelLinearVelocity = 0.0
        self.RightWheelLinearVelocity = 0.0
        self.DistanceTraveled = 0.0
        self.LinearVelocityTotal = 0.0
        self.SegmentTime = 0.0

    def WaypointTotals(self):
        Waypoints.TotalTime += self.SegmentTime
        Waypoints.TotalPathLength += self.DistanceTraveled

    @classmethod
    def GetTime(cls):
        return cls.TotalTime

    @classmethod
    def GetPathLength(cls):
        return cls.TotalPathLength
    def PrintAll(self):
        print(f"The right wheel angular velocity is: {self.RightWheelLinearVelocity:.2f} meters/sec."
              f"\nThe left {self.LeftWheelLinearVelocity:.2f} meters/sec. "
              f"\nThe robot traveled for {self.DistanceTraveled:.2f} meters"
              f" and it took {self.SegmentTime:.2f} seconds to travel. "
              f"\nThe expected total path length is {Waypoints.TotalPathLength:.2f} meters and "
              f"it's expected to run for {Waypoints.GetTime():.2f} seconds ")

    def PrintDuringNavigation(self):
        print(f"The Right wheel angular velocity is: {self.RightWheelLinearVelocity:.3f} meters/sec "
              f"and the left {self.LeftWheelLinearVelocity:.3f} meters/sec. The robot traveled for {self.DistanceTraveled:.3f} meters"
              f", and it took {self.SegmentTime:.3f} seconds to travel")
# kinematics calculations
# using 50 RPMs
# add segment time to total time
#use time.time()  0r time.perf_counter for the travel time
if __name__ == "__main__":
    p0 = [2.0,-2.0,math.pi]
    p1 = [-1.5, -2.0, math.pi]
    p2 = [-2.0, -1.5, math.pi/2]
    p3 = [-2.0, -0.5, math.pi/2]
    p4 = [-1.0, -0.5, (3 * math.pi)/2]
    p5 = [-0.5, -1.0, (7 * math.pi)/4]
    p6 = [2.0, -1.0, 0]
    p7 = [2.0, 0.0, math.pi/2]
    P8 = [0.0, 0.0, math.pi]
    P9 = [0.0, 1.0, math.pi/2]
    P10 = [-2.0, 1.0, math.pi]
    P11 = [-1.0, 2.0, 0]
    P12 = [1.5, 2.0, 0]
    P13 = [0, 0, 0]
    # UNKNOWN VALUE

    # robot movement
#Bot = HamBot(lidar_enabled=False, camera_enabled=False)

    #UNCOMMENT THE ABOVE SECTION

    #p0 to p1
    P0toP1 = Waypoints()
    P0toP1.LeftWheelLinearVelocity = P0toP1.WheelAngularVelocity
    P0toP1.RightWheelLinearVelocity = P0toP1.WheelAngularVelocity
    P0toP1.DistanceTraveled = StraightLineFormula(p0[0], p1[0], p0[1], p1[1])
    P0toP1.LinearVelocityTotal = (P0toP1.LeftWheelLinearVelocity + P0toP1.RightWheelLinearVelocity) /2
    P0toP1.SegmentTime = P0toP1.DistanceTraveled / P0toP1.LinearVelocityTotal
    P0toP1.WaypointTotals()
    P0toP1.PrintAll()
    #p1 to p2
    P1toP2 = Waypoints()
    P1toP2.LeftWheelLinearVelocity = P1toP2.WheelAngularVelocity
    P1toP2.RightWheelLinearVelocity = P1toP2.WheelAngularVelocity
   # P1toP2.DistanceTraveled =
    #p2 to p3
    P2toP3 = Waypoints()
    P2toP3.LeftWheelLinearVelocity = P2toP3.WheelAngularVelocity
    P2toP3.RightWheelLinearVelocity = P2toP3.WheelAngularVelocity
    P2toP3.DistanceTraveled = StraightLineFormula(p2[0], p3[0], p2[1], p3[1])
    P2toP3.LinearVelocityTotal = (P2toP3.LeftWheelLinearVelocity + P2toP3.RightWheelLinearVelocity) / 2
    P2toP3.SegmentTime = P2toP3.DistanceTraveled / P2toP3.LinearVelocityTotal
    P2toP3.WaypointTotals()
    P2toP3.PrintAll()




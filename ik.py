from typing import Iterable
from math import cos, sin
import math
import numpy as np
from util import clamp


"""
arm diagram

       a3 o
       l2/ \l3
a1 l1 a2/   \ 
 ------o 

a1 is yaw
a2 is pitch/elbow
a3 is wrist

a2 0 is horizontal
a3 0 is folded back
"""



# length of arms
l1 = 0.08
# l2 and l3 should be equal for now
l2 = 0.17
l3 = 0.17

# motor native directions
# yaw: positive counter clockwise
# elbow: positive down
# wrist: positive down

# positive x is at center (90) of yaw servo
# positive y is at right (0) of yaw servo
# negative y is at left (180) of yaw servo
# z is negative up

# angle offsets
offset_1 = -15
offset_2 = 7
# offset_3 = -15
offset_3 = -20

# gets angles for arm joins to meet an arm relative position with the end of l3
# uses analytical inverse kinematics
def getAngles(goal: Iterable[float], margin: float = 0) -> Iterable[float]:
    # if the lengths are not equal it does not produce the correct solution
    assert l2==l3
    # clamp goal to within reachable area
    # distance yaw and pitch from origin (first joint)
    dist = math.hypot(*goal)
    new_dist = clamp(dist, l1, 9999999)
    # recalculate goal
    goal = [new_dist*x/dist for x in goal]
    yaw = math.atan2(goal[1], goal[0])
    if goal[0] < 0:
        # goal is in backwards hemisphere
        # TODO: point behind self
        # currently just clamps, should point 180 and use elbow to get behind
        yaw = clamp(yaw, -math.pi/2, math.pi/2)

    a2_pos = [math.cos(yaw)*l1, math.sin(yaw)*l1, 0]
    # distance from end of l1 to goal from top down view, just x and y
    flat_dist = math.hypot(goal[0]-a2_pos[0], goal[1]-a2_pos[1])
    pitch = math.atan2(goal[2], flat_dist)
    # 3d distance from end of a2 to goal
    dist = math.sqrt(sum((a-b)**2 for a, b, in zip(goal, a2_pos)))
    # print(list(list(round(x, 3) for x in p) for p in [goal, a2_pos]), dist)
    # clamp distance to reachable
    _dist = clamp(dist-margin, 0.00, l2+l3)
    # if dist != _dist:
    #     print("clamped dist", dist, _dist)
    dist = _dist
    # assumes l2==l3
    # print((dist/2)/l2)
    inner_angle = math.asin((dist/2)/l2)*2

    return math.degrees(yaw)+90, math.degrees(pitch)+90+math.degrees(inner_angle/2), 180-math.degrees(inner_angle)

# gets the point of the end effect
def getEndPoint(angles):
    yaw, elbow, wrist = angles
    a2_pos = np.array([cos(yaw)*l1, sin(yaw)*l1, 0])
    a3_pos = a2_pos + np.array([sin(yaw)*sin(elbow)*l2, cos(yaw)*sin(elbow)*l2, cos(elbow)])
    end_pos = a3_pos + np.array([sin(yaw)*sin(elbow+wrist)*l3, cos(yaw)*sin(elbow+wrist)*l3, cos(elbow+wrist)])
    return end_pos


# def toServo(angles):
#     yaw, pitch, wrist = angles
#     return math.degrees(yaw)+90, math.degrees(pitch)+90+math.degrees(inner_angle/2), 180-math.degrees(inner_angle)

if __name__ == "__main__":
    from math import isclose
    # getAngles tests

    def testIk(func):
        angles = func([3, 0, 0])
        assert isclose(angles[0], 90, abs_tol=1e-4), angles[0]
        assert isclose(angles[1], 180, abs_tol=1e-4), angles[1]
        assert isclose(angles[2], 0, abs_tol=1e-4), angles[2]
    
    testIk(getAngles)
    assert getEndPoint(getAngles([0.1, 0, 0])) == [0.1, 0, 0]
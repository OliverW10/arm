from typing import Iterable
import math

from util import clamp



"""
arm diagram

       a3 o
       l2/ \l3
a1 l1 a2/   \ 
 ------o     

a2 is elbow
a3 is wrist
"""



# length of arms
l1 = 0.075
# l2 and l3 should be equal for now
l2 = 0.1475
l3 = 0.1475

# angle offsets
offset_1 = -15
offset_2 = 7
offset_3 = -15

# yaw: positive counter clockwise
# elbow: positive down
# wrist: positive down

# positive x is at center (90) of yaw servo
# positive y is at right (0) of yaw servo
# negative y is at left (180) of yaw servo
# z is negative up

# gets angles for arm joins to meet an arm relative position with the end of l3
# goal[x, y, z]
def getAngles(goal: Iterable[float]) -> Iterable[float]:
    yaw = -math.atan2(goal[1], goal[0])
    if goal[0] < 0:
        # goal is in backwards hemisphere
        # TODO: point behind self
        # currently just clamps, should point 180 and use elbow to get behind
        yaw = clamp(yaw, -math.pi/2, math.pi/2)

    a2_pos = [math.sin(yaw)*l1, math.cos(yaw)*l1, 0]
    # distance from end of l1 to goal, in just x and y
    flat_dist = math.hypot(goal[0], goal[1])-l1
    pitch = math.atan2(goal[2], flat_dist)
    # distance from end of l1 to goal
    dist = math.sqrt(sum((a-b)**2 for a, b, in zip(goal, a2_pos)))
    # clamp distance to reachable
    dist = clamp(dist, 0.02, l2+l3-0.01)
    # assumes l2==l3
    # print((dist/2)/l2)
    inner_angle = math.asin((dist/2)/l2)

    return math.degrees(yaw)+90, math.degrees(inner_angle/2)+90, 180-math.degrees(inner_angle)

# clamps to where the arm can reach
def clampReachable(goal):
    a1 = math.atan2(goal[1], goal[0])
    a2 = math.atan2(goal[2], goal[0])
    leng = math.hypot(goal)
    leng = clamp(leng, l1+0.01, l1+l2+l3-0.01)
    # return [math.sin()]
    return goal


if __name__ == "__main__":
    from math import isclose
    # clampReachable tests
    assert clampReachable([0, l1+l2, 0]) == [0, l1+l2, 0]
    assert clampReachable([])
    # getAngles tests
    assert isclose(getAngles([0, 1, 0])[0], 90, abs_tol=1e-4)
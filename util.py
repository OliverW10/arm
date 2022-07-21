from math import atan2, sqrt

def project(x, y, z, fx, fy):
    return (
        (fx*x)/z,
        (fy*y)/z,
    )

# coverts a 3x3 rotation matrix into euler angles
def rotmatToEuler(mat):
    # https://stackoverflow.com/a/15029416
    x = atan2(mat[2][1], mat[2][2])
    y = atan2(-mat[2][0], sqrt(mat[2][1]**2 + mat[2][2]**2))
    z = atan2(mat[1][0], mat[0][0])
    return (x, y, z)

def clamp(x, low, high):
    # if they are the wrong way swap them
    if low > high:
        high, low = low, high
    return max(low, min(high, x))
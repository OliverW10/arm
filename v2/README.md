### Goal
have a robotic arm you can hold and move around and it will keep the end of the arm still.

Uses a raspberry pi, a realsense t265 camera and pwm servo's.

### Libraries
build system is a bit bad atm, ill learn how cmake works later

realsense install:
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

also needs eigen, [pigpio](https://github.com/joan2937/pigpio/) and [apriltag](https://github.com/AprilRobotics/apriltag/)

### Modules:
- `kinematics`:
    Initially used orocos kdl for the kinematics but decided to write it myself to learn.
    The forward kinematics work by calculating and then multiplying the transformation matricies of each joint. The inverse kinematics works by first getting a rough solution geometrically (rough beacuse it dosent support joint displacments in more than one axis) using a bit of trigonometry and the refining it iteratively with the forwards kinematics. 


- `vision`:
    Use an intel realsense T265 to get a pose estimate, this has some drift as it is just a combination of accelerometer and optical flow. To improve this estimate use apriltags to rezero the pose.


- `arm`:
    Do basic motion planning to smooth the servo's motion and then send the positions to the servo with pwm on the Pi's gpio pins. 


- `main`:

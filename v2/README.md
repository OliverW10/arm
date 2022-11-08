### Goal
have a robotic arm you can hold and move around and it will keep the end of the arm still.

Uses a raspberry pi, a realsense t265 camera and pwm servo's.

### Libraries
- install and build realsense so its in `/usr/local/lib` and `/usr/local/include` 
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

- get the submodules with `git submodule init` then `git submodule update`

- build the apriltag and pigpio submodules with cmake, `mkdir build && cd build && cmake .. && make`

- also needs eigen in `/usr/include/eigen3`

### Modules:
- `kinematics`:
    Initially used orocos kdl for the kinematics but decided to write it myself to learn.
    The forward kinematics work by calculating and then multiplying the transformation matricies of each joint. The inverse kinematics works by first getting a rough solution geometrically (rough beacuse it dosent support joint displacments in more than one axis) using a bit of trigonometry and the refining it iteratively with the forwards kinematics. 


- `vision`:
    Use an intel realsense T265 and apriltags to work out where it is and where the goal position is. Runs a seperate thread for reading from the realsense camera and for running the apriltags stuff.


- `arm`:
    Does motion planning to smooth the servo's motion and then send the positions to the servo with pwm on the Pi's gpio pins. 


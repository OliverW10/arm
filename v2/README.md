## Goal
have a robotic arm you can hold and move around and it will keep the end of the arm still.

Uses a raspberry pi, a realsense t265 camera and pwm servo's.

## Libraries
### Dev
- install and build realsense 

    worked on pc:
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

    worked on pi:
https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435

- get the submodules with `git submodule init` then `git submodule update`

- build the apriltag and pigpio submodules with cmake, `mkdir build && cd build && cmake .. && make` 

- also needs eigen in `/usr/include/eigen3`

### Install on Raspberry Pi
ended up just compiling on pi, ill leave the info on cross compilation here anyway for future me

https://deardevices.com/2019/04/18/how-to-crosscompile-raspi/

cmake for crosscompiling the pi `cmake -D CMAKE_C_COMPILER=/opt/pi/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc -D CMAKE_CXX_COMPILER=/opt/pi/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-g++ ..`

`cmake -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false ..`

TODO: find good power supply, all phone chargers I tried result in undervoltage. can be checked with `vcgencmd get_throttled`. portable charger battery seems to be better but still reports undervoltages 

## Modules:
- `kinematics`:
    Initially used orocos kdl for the kinematics but decided to write it myself to learn.
    The forward kinematics work by calculating and then multiplying the transformation matricies of each joint. The inverse kinematics works by first getting a rough solution geometrically (rough beacuse it dosent support joint displacments in more than one axis) using a bit of trigonometry and the refining it iteratively with the forwards kinematics. 


- `vision`:
    Use an intel realsense T265 and apriltags to work out where it is and where the goal position is. Runs a seperate thread for reading from the realsense camera and for running the apriltags stuff.


- `arm`:
    Does motion planning to smooth the servo's motion and then send the positions to the servo with pwm on the Pi's gpio pins. 

- `bringup`: tests arm and kinematics
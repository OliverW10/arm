#pragma once
#include <iostream>
// #include <pigpio.h>

class Arm{
public:
    Arm(){
        // if(gpioInitialise() < 0){
        //     std::cerr << "Couldn't initialise pigpio\n";
        // }
    }
private:
    // maximum rotation speed (rad/s) and accel (rad/s^2)
    // TODO: measure this
    const double max_speed = 1;
    const double max_accel = 5;
};
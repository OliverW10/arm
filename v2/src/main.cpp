#include <Eigen/Dense>
#include <math.h>
#include <random>
#include <thread>
#include "arm.hpp"
#include "vision.hpp"


int main(){
    int pins[3] = {5, 6, 7};
    Arm arm(pins);
    Vision vision;
    Eigen::Vector3d target;
    target << 0.1, 0, 0;
    bool success = arm.setGoal(target);
    if(!success){
        std::cout << "abcdef failed to set goal, likely beacuse goal is invalid\n";
        return -1;
    }

    int loops = 0;
    long start = std::chrono::steady_clock::now().time_since_epoch().count();
    const double delta = 0.2/100;
    while(true){
        if(vision.isActive()){
            Eigen::Matrix4d arm_pose = vision.getPose();
            std::cout << "pose from main thread:\n" << arm_pose << "\n\n";
            // workout target reletive to arm
            // goal = ??
            // bool success = arm.setGoal(goal);
            // arm.execute();
        }else{
            std::cout << "waiting for vision to start, should take max 10 seconds\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // long finish = std::chrono::steady_clock::now().time_since_epoch().count();
    // std::cout << "microseconds per loop: " << (finish-start)/1000/100 << " microseconds\n";

    return 0;
}

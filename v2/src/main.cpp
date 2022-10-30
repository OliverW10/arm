#include <Eigen/Dense>
#include <math.h>
#include <random>
#include "arm.hpp"
#include "vision.hpp"

int main(){
    int pins[3] = {5, 6, 7};
    Arm arm(pins);
    // Vision vision;
    Eigen::Vector3d goal;
    goal << 0.1, 0, 0;
    bool ret = arm.setGoal(goal);
    if(!ret){
        std::cout << "failed to set goal, likely beacuse goal is invalid\n";
    }else{

        int loops = 0;
        long start = std::chrono::steady_clock::now().time_since_epoch().count();
        const double delta = 0.2/100;
        while(loops < 100){
            // vision.execute(); 
            // Eigen::Matrix4f cameraPose = vision.getPose();
            goal(0) += delta;
            bool ret = arm.setGoal(goal);
            arm.execute();
            // time_sleep(0.01);
            loops ++;
        }

        long finish = std::chrono::steady_clock::now().time_since_epoch().count();
        std::cout << "microseconds per loop: " << (finish-start)/1000/100 << " microseconds\n";
    }

    return 0;
}

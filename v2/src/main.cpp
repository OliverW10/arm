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
    goal << 0.2, 0, 0;
    bool ret = arm.setGoal(goal);
    if(!ret){
        std::cout << "failed to set goal, likely beacuse goal is invalid\n";
    }else{

    int loops = 0;
    while(true){
        loops ++;
        if(loops > 100){
            break;
        }
        // vision.execute(); 
        // Eigen::Matrix4f cameraPose = vision.getPose();
        arm.execute();
        time_sleep(0.02);
    }}
    return 0;
}

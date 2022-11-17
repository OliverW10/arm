#include <Eigen/Dense>
#include <math.h>
#include <random>
#include <thread>
#include "arm.hpp"


int main()
{
    int pins[] = {5, 6, 7};
    Arm arm(pins);

    Eigen::Vector3d target1;
    target1 << 0.1, -0.1, 0;
    Eigen::Vector3d target2;
    target2 << 0.1, 0.1, 0;

    bool success = arm.setGoal(target1);
    if (!success)
    {
        std::cout << "failed to set target1, likely beacuse goal is invalid\n";
        return -1;
    }
    success = arm.setGoal(target2);
    if (!success)
    {
        std::cout << "failed to set target2, likely beacuse goal is invalid\n";
        return -1;
    }

    // #1
    arm.setJoints(JntArray(0, 0, 0));
    // #2
    // arm.setJoints(arm.kinematics.min_angles);
    // #3
    // arm.setJoints(arm.kinematics.max_angles);

    // #4
    // bool target_num = 0;
    // auto last_change = std::chrono::steady_clock::now();
    // while (true)
    // {
    //     auto now = std::chrono::steady_clock::now();
    //     if (now - last_change >= std::chrono::seconds(2)){
    //         if(target_num){
    //             arm.setGoal(target1);
    //         }else{
    //             arm.setGoal(target2);
    //         }
    //         std::cout << "swapped goals\n";
    //         target_num = !target_num;
    //     }
    //     arm.execute();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }


    return 0;
}

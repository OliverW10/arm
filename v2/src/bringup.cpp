#include <thread>
#include <iostream>
#include "arm.hpp"

int main()
{
    int pins[] = {4, 17, 18};
    std::cout << "started\n";
    Arm arm(pins);
    std::cout << "past constructor\n";

    // Eigen::Vector3d target1;
    // target1 << 0.1, -0.1, 0;
    // Eigen::Vector3d target2;
    // target2 << 0.1, 0.1, 0;

    // bool success = arm.setGoal(target1);
    // if (!success)
    // {
    //     std::cout << "failed to set target1, likely beacuse goal is invalid\n";
    //     return -1;
    // }
    // success = arm.setGoal(target2);
    // if (!success)
    // {
    //     std::cout << "failed to set target2, likely beacuse goal is invalid\n";
    //     return -1;
    // }

    bool target_num = 0;
    auto last_change = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    arm.setJoints(JntArray(0, M_PI, -M_PI));
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_change >= std::chrono::seconds(3)){
            if(target_num){
                arm.setJoints(JntArray(-M_PI / 2.0, 0, 0));
            }else{
                arm.setJoints(JntArray(M_PI / 2.0, M_PI, -M_PI));
            }
            std::cout << "swapped goals\n";
            target_num = !target_num;
            last_change = std::chrono::steady_clock::now();
        }
        arm.execute();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (now - start >= std::chrono::seconds(120)){
            break;
        }
    }


    return 0;
}

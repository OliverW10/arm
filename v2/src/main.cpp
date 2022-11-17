#include <Eigen/Dense>
#include <math.h>
#include <random>
#include <thread>
#include "arm.hpp"

int main()
{
    int pins[3] = {5, 6, 7};
    Arm arm(pins);
    Eigen::Vector3d target1;
    target1 << 0.1, -0.1, 0;
    Eigen::Vector3d target2;
    target2 << 0.1, 0.1, 0;
    bool target_num = 0;

    Eigen::Matrix3d camera_to_arm_rotation = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).matrix();
    // make it 4x4 so it can be multiplied easily
    Eigen::Matrix4d camera_to_arm = Eigen::Matrix4d::Identity();
    camera_to_arm.block<3, 3>(0, 0) = camera_to_arm_rotation;

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

    int loops = 0;
    auto last_change = std::chrono::steady_clock::now();
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_change >= std::chrono::seconds(2)){
            if(target_num){
                arm.setGoal(target1);
            }else{
                arm.setGoal(target2);
            }
            target_num = !target_num;
        }
        arm.execute();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // long finish = std::chrono::steady_clock::now().time_since_epoch().count();
    // std::cout << "microseconds per loop: " << (finish-start)/1000/100 << " microseconds\n";

    return 0;
}

#include <Eigen/Dense>
#include <math.h>
#include <random>
#include <thread>
#include "arm.hpp"
#include "vision.hpp"

int main()
{
    int pins[3] = {5, 6, 7};
    Arm arm(pins);
    Vision vision;

    Eigen::Matrix3d camera_to_arm_rotation = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).matrix();
    // make it 4x4 so it can be multiplied easily
    Eigen::Matrix4d camera_to_arm = Eigen::Matrix4d::Identity();
    camera_to_arm.block<3, 3>(0, 0) = camera_to_arm_rotation;

    while (true)
    {
        if(vision.isActive()){
            Eigen::Matrix4d camera_pose = vision.getPose();
            // get target from camera
            // arm.setGoal();
            arm.execute();
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    // long finish = std::chrono::steady_clock::now().time_since_epoch().count();
    // std::cout << "microseconds per loop: " << (finish-start)/1000/100 << " microseconds\n";

    return 0;
}

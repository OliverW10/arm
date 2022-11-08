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
    Eigen::Vector3d target;
    target << 0.1, 0, 0;

    Eigen::Matrix3d camera_to_arm_rotation = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).matrix();
    // make it 4x4 so it can be multiplied easily
    Eigen::Matrix4d camera_to_arm = Eigen::Matrix4d::Identity();
    camera_to_arm.block<3, 3>(0, 0) = camera_to_arm_rotation;

    bool success = arm.setGoal(target);
    if (!success)
    {
        std::cout << "abcdef failed to set goal, likely beacuse goal is invalid\n";
        return -1;
    }

    int loops = 0;
    long start = std::chrono::steady_clock::now().time_since_epoch().count();
    const double delta = 0.2 / 100;
    while (true)
    {
        if (vision.isActive())
        {
            Eigen::Matrix4d camera_pose = vision.getPose();
            std::cout << "pose from main thread:\n"
                      << camera_pose << "\n\n";
            // rotate to arm frame
            auto arm_pose = camera_pose * camera_to_arm;
            // find target relative to the arm
            Eigen::Vector3d goal;
            goal(0) = target(0) - arm_pose(3, 0);
            goal(1) = target(1) - arm_pose(3, 1);
            goal(2) = target(2) - arm_pose(3, 2);
            goal *= arm_pose.block<3, 3>(0, 0).inverse();
            bool success = arm.setGoal(goal);
            arm.execute();
        }
        else
        {
            std::cout << "waiting for vision to start, should take max 10 seconds\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // long finish = std::chrono::steady_clock::now().time_since_epoch().count();
    // std::cout << "microseconds per loop: " << (finish-start)/1000/100 << " microseconds\n";

    return 0;
}

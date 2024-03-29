#include <Eigen/Core>
#include <math.h>
#include <iostream>
#include <thread>
#include "arm.hpp"
#include "vision.hpp"

int main()
{
    int pins[] = {4, 17, 18};
    Arm arm(pins);
    Vision vision;
    Eigen::Vector4d target;
    // target is t265 coordinate system
    target << 0.15, 0, 0, 1;

    arm.setJoints(JntArray(0, M_PI, -M_PI));

    // wait for vision to start
    while (!vision.isActive())
    {
        std::cout << "waiting for vision to start, should take max 10 seconds\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    while (true)
    {
        Eigen::Matrix4d arm_pose = vision.t265_to_camera * vision.getArmPose();
        std::cout << "pose from main thread:\n"
                  << arm_pose << "\n";
        // find target relative to the arm
        Eigen::Vector4d _goal = arm_pose.inverse() * target;
        // fix coordinate system
        Eigen::Vector4d fixed_goal = vision.t265_to_camera * _goal;
        // remove end 1
        Eigen::Vector3d goal = fixed_goal.block<3, 1>(0, 0);
        // std::cout << "goal:\n" << goal << "\n";
        bool success = arm.setGoal(goal);
        if(!success){
            // std::cout << "goal unreachable\n";
        }
        arm.execute();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}

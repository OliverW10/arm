#include <thread>
#include <iostream>
#include "arm.hpp"

int main()
{
    int pins[] = {4, 17, 18};
    std::cout << "started\n";
    Arm arm(pins);
    std::cout << "past constructor\n";

    Eigen::Vector3d target;
    target << 0.1, -0.1, 0;

    bool success = arm.setGoal(target);
    if (!success)
    {
        std::cout << "failed to set target, likely beacuse goal is invalid\n";
        return -1;
    }

    double angle = 0;
    const double r = 0.1;
    auto last_time = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    arm.setJoints(JntArray(0, M_PI, -M_PI));
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        auto delta = now - last_time;
        // 2 radian / second
        angle += 2 * delta.count() / 1000.0 / 1000.0 / 1000.0;
        target << 0.15, sin(angle) * r, 0.1 + cos(angle) * r;
        arm.setGoal(target);
        arm.execute();
        last_time = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // if connection is lost don't continue indefinetly
        if (now - start >= std::chrono::seconds(60)){
            break;
        }
    }


    return 0;
}

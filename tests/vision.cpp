#include "vision.hpp"
#include "Eigen/Core"

rs2_pose createRsPose(float x, float y, float z, Eigen::Matrix3d rot){
    rs2_pose pose;
    pose.translation.x = x;
    pose.translation.y = y;
    pose.translation.z = z;

    Eigen::Quaterniond quat(rot);
    pose.rotation.w = quat.w();
    pose.rotation.x = quat.x();
    pose.rotation.y = quat.y();
    pose.rotation.z = quat.z();
    assert(quat.matrix().isApprox(rot));

    return pose;
}


int main(){
    Eigen::Vector4d target;
    // goal in t265 coordinate system
    target << 0, 0, -0.15, 1;
    Vision vision;

    Eigen::Matrix4d pose;
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Eigen::Vector4d _goal = pose.inverse() * target;
    Eigen::Vector4d fixed_goal = vision.t265_to_camera * _goal;
    std::cout << "goal from 0, 0, 0:\n" << fixed_goal << "\n";

    // 10cm backwards
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.1,
            0, 0, 0, 1;
    _goal = pose.inverse() * target;
    fixed_goal = vision.t265_to_camera * _goal;
    std::cout << "\ngoal from 10cm backwards:\n" << fixed_goal << "\n";

    // -90 degrees around vertical axis
    pose << 0, 0, 1, 0,
            0, 1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;
    _goal = pose.inverse() * target;
    fixed_goal = vision.t265_to_camera * _goal;
    std::cout << "\n-90 around vertical:\n" << fixed_goal << "\n";
}
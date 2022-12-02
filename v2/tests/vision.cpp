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
    Vision vision;

    Eigen::Matrix3d rotation;
    rotation.setIdentity();
    rs2_pose pose_zero = createRsPose(0, 0, 0, rotation);
    Eigen::Matrix4d ret_zero = vision.cameraToArm(vision.poseToTransform(pose_zero));
    std::cout << "arm when camera at 0:\n" << ret_zero << "\n";

    rs2_pose pose1 = createRsPose(0, 0, -0.1, rotation);
    Eigen::Matrix4d ret1 = vision.cameraToArm(vision.poseToTransform(pose1));
    std::cout << "arm when camera at 10cm forwards:\n" << ret1 << "\n";

    rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    std::cout << "rotation3:\n" << rotation <<"\n";
    rs2_pose pose2 = createRsPose(0, 0, 0, rotation);
    Eigen::Matrix4d ret2 = vision.cameraToArm(vision.poseToTransform(pose2));
    std::cout << "arm when camera at 0 and rotated 90 on Y axis:\n" << ret2 << "\n";
}
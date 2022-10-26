#include <Eigen/Dense>
#include "kinematics.hpp"
#include "vision.hpp"
#include <math.h>

int main(){
    ArmKinematics arm;
    Eigen::Matrix<float, num_joints, 1> joint_angles { 0, M_PI/4, -M_PI/2};
    Eigen::Matrix4f end_pose;
	arm.forwards(joint_angles, end_pose);

    Eigen::Vector3f target_pos;
    target_pos << 0, 0.35, 0;
    Eigen::Matrix<float, num_joints, 1> out_joint_angles = Eigen::Matrix<float, num_joints, 1>::Zero();
    arm.backwards(target_pos, out_joint_angles);
    std::cout << out_joint_angles;

    // Vision vision;
    // while(true){
    //     vision.execute(); 
    //     Eigen::Matrix4f cameraPose = vision.getPose();
    // }
}

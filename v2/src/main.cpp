#include <Eigen/Dense>
#include "kinematics.hpp"
#include "vision.hpp"
#include <math.h>
#include <random>

int main(){
    srand((unsigned int) time(0));


    ArmKinematics arm;
    JntArray joint_angles { 0, M_PI/4, -M_PI/2};
    Eigen::Vector3d end_pose = arm.forwards(joint_angles);

    Eigen::Vector3d target_pos;
    target_pos << 0.2, 0, 0;
    JntArray out_joint_angles = JntArray::Zero();
    arm.backwards_geo(target_pos, out_joint_angles);
    std::cout << "geo\n" << out_joint_angles << "\n";

    bool ret = arm.backwards_num(target_pos, out_joint_angles);
    if(!ret){
        std::cout << "numerical failed\n";
    }else{
        std::cout << "num\n" << out_joint_angles << "\n";
    }

    // Vision vision;
    // while(true){
    //     vision.execute(); 
    //     Eigen::Matrix4f cameraPose = vision.getPose();
    // }
    return 0;
}

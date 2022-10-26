#include <Eigen/Dense>
#include "kinematics.hpp"
#include "vision.hpp"
#include <math.h>


int main(){
    ArmKinematics arm;
    Eigen::Matrix<float, num_joints, 1> joint_angles { M_PI/2, 0, 0};
    Eigen::Matrix4f end_pose;
	arm.forwards(joint_angles, end_pose);

    // Vision vision;
    // while(true){
    //     vision.execute(); 
    //     Eigen::Matrix4f cameraPose = vision.getPose();
    // }
}

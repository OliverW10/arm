#include "kinematics.hpp"

ArmKinematics::ArmKinematics()
{
    minAngles = {-M_PI/2, 0, -M_PI};
    maxAngles = {M_PI/2, M_PI, 0};
    // TODO measure these
    displacements[0] << 0.05, 0, 0;
    displacements[1] << 0.15, 0, 0;
    displacements[2] << 0.15, 0, 0;
}

void ArmKinematics::forwards(Eigen::Matrix<float, num_joints, 1> joint_angles, Eigen::Matrix4f out){
    /*
        [   rotation 3x3  , translation 3x1 ]
        [       ...       ,      ...        ]
        [       ...       ,      ...        ]
        [       ...       ,      ...        ]
        [    0,   0,   0  ,       1         ]
    */

    // initialize to identity so the first joint becomes itself
    out = Eigen::Matrix4f::Identity();
    std::cout << "full:\n" << out << "\n";

    for(int i = 0; i < num_joints; ++i){
        // create transform from previous joint to current joint frame
        Eigen::Matrix4f joint_transform;
        // create bottom row to preserve 1 in vector
        joint_transform(3, 0) = 0;
        joint_transform(3, 1) = 0;
        joint_transform(3, 2) = 0;
        joint_transform(3, 3) = 1;
        // create rotation matrix
        joint_transform.block<3,3>(0, 0) = Eigen::AngleAxisf(joint_angles(i), joint_axis[i]).matrix();
        // get transform vector
        joint_transform.block<3, 1>(0, 3) = displacements[i];

        std::cout << "joint:\n" << joint_transform << "\n";

        // multiply into full transform
        out *= joint_transform;
        std::cout << "full:\n" << out << "\n";
    }
}

void ArmKinematics::backwards(Eigen::Vector3f target, Eigen::Matrix<float, num_joints, 1> out){
}

bool ArmKinematics::isReachable(Eigen::Vector3f target){
}

bool ArmKinematics::isAchiveable(Eigen::Matrix<float, num_joints, 1> joint_angles){

}
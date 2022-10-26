#include "kinematics.hpp"

// #define DEBUG_PRINTS

ArmKinematics::ArmKinematics()
{
    minAngles = {-M_PI/2, 0, -M_PI};
    maxAngles = {M_PI/2, M_PI, 0};
    // TODO measure these
    displacements[0] << 0, 0, 0;
    displacements[1] << 0.05, 0, 0;
    displacements[2] << 0.15, 0, 0;
    displacements[3] << 0.15, 0, 0;

    joint_axis[0] = Eigen::Vector3f::UnitZ();
    joint_axis[1] = Eigen::Vector3f::UnitY();
    joint_axis[2] = Eigen::Vector3f::UnitY();
}

void ArmKinematics::forwards(Eigen::Matrix<float, num_joints, 1> joint_angles, Eigen::Matrix4f &out){
    /*
    does the forwards kinematics by multiplying transformation matricies

        [   rotation 3x3  , translation 3x1 ]
        [       ...       ,      ...        ]
        [       ...       ,      ...        ]
        [       ...       ,      ...        ]
        [    0,   0,   0  ,       1         ]
    */

    // initialize to identity so the first joint becomes itself
    out = Eigen::Matrix4f::Identity();

    // each loop add the displacment to get to the joint and the joints rotation
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


        // multiply into full transform
        out *= joint_transform;
        #ifdef DEBUG_PRINTS
        std::cout << "joint:\n" << joint_transform << "\n";
        std::cout << "full:\n" << out << "\n";
        #endif
    }
    // apply final segments displacments
    Eigen::Matrix4f joint_transform = Eigen::Matrix4f::Identity();
    joint_transform.block<3, 1>(0, 3) = displacements[num_joints];
    out *= joint_transform;
    #ifdef DEBUG_PRINTS
    std::cout << "joint:\n" << joint_transform << "\n";
    std::cout << "full:\n" << out << "\n";
    #endif
}

void ArmKinematics::backwards(Eigen::Vector3f target, Eigen::Matrix<float, num_joints, 1> &out){
    // does inverse kinematics geometrically, assumes each joint's displacment is only in x
    // TODO: investigate other ways to do analytic ik
    float x = target(0);
    float y = target(1);
    float z = target(2);

    // get x component of the displacment for the shoulder and elbow joints
    float a = displacements[2](0);
    float b = displacements[3](0);
    // x of displacment for turret joint
    float t = displacements[1](0);

    out(0) = atan2f(y, x);
    // distances from end of the flat segment to the target
    // distance in only the x and y axies
    float dz = sqrtf(x*x + y*y) - t;
    // overall distance
    float d = sqrtf(x*x + y*y + z*z) - t;
    // squared used in cosine rule
    float d_squared = d * d;
    out(1) = atan2f(z, dz) + acosf((a*a + d_squared - b*b)/(2*a*d));
    out(2) = -acosf((a*a + b*b - d_squared)/(2*a*b));
}

bool ArmKinematics::isReachable(Eigen::Vector3f target){
}

bool ArmKinematics::isAchiveable(Eigen::Matrix<float, num_joints, 1> joint_angles){

}
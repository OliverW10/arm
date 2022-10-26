#ifndef KINEMATICS
#define KINEMATICS

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

const int num_joints = 3;

// a class to compute kiematics for my arm
class ArmKinematics{
public:
	ArmKinematics();

    void forwards(Eigen::Matrix<float, num_joints, 1> joint_angles, Eigen::Matrix4f out);
    void backwards(Eigen::Vector3f target, Eigen::Matrix<float, num_joints, 1> out);
    // checks if the position is possible to reach
    bool isReachable(Eigen::Vector3f target);
    // checks if the joint values are within the min and max constraints
    bool isAchiveable(Eigen::Matrix<float, num_joints, 1> joint_angles);
private:

    Eigen::Matrix<float, num_joints, 1> minAngles;
    Eigen::Matrix<float, num_joints, 1> maxAngles;

    // the displacements between each joint frame
    // first item is the displacments between the frame of the first joint (not the base frame)
    // and the second joint
    Eigen::Vector3f displacements[num_joints];
    // the axis around which the joint rotates, use Eigen::Vector3f::Unit?()
    Eigen::Vector3f joint_axis[num_joints];
};

#endif // KINEMATICS

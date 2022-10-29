#ifndef KINEMATICS
#define KINEMATICS

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

const int num_joints = 3;

typedef Eigen::Array<double, num_joints, 1> JntArray;


class ArmKinematics{
public:
	ArmKinematics();

    Eigen::Vector3d forwards(JntArray joint_angles);

    // calculates inverse kinematics geometrically
    // only works for:
    // - 3 joints around: z, y, y
    // - with displacments in only x between joints
    // - a reachable target
    bool backwards_geo(Eigen::Vector3d target, JntArray &out);

    // calculates inverse kinematics numerically
    // should work for any arm with a valid forwards function
    bool backwards_num(Eigen::Vector3d target, JntArray &out);

    // checks if the position is possible to reach
    bool isReachable(Eigen::Vector3d target);

    // checks if the joint values are within the min and max constraints
    bool isJointsValid(JntArray joint_angles);

    // generates a random valid joint array
    void randomJntArray(JntArray &out);

    const JntArray min_angles = {-M_PI / 2, 0, -M_PI};
    const JntArray max_angles = {M_PI / 2, M_PI, 0};
private:
    double getError(JntArray joints, Eigen::Vector3d target);


    // the displacements between each joint frame
    // first item is the displacments between the base frame and the first joint
    Eigen::Vector3d displacements[num_joints+1];
    // the axis around which the joint rotates, use Eigen::Vector3d::Unit?()
    Eigen::Vector3d joint_axis[num_joints];
};

#endif // KINEMATICS

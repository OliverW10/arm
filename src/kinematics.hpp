#ifndef KINEMATICS
#define KINEMATICS

#include <math.h>
#include <Eigen/Core>

const int num_joints = 3;

typedef Eigen::Array<double, num_joints, 1> JntArray;

class ArmKinematics
{
public:
    ArmKinematics();

    Eigen::Vector3d forwards(const JntArray &joint_angles);

    // calculates inverse kinematics geometrically
    // only works for:
    // - 3 joints around: z, y, y
    // - with displacments in only x between joints
    // - a reachable target
    bool backwards_geo(const Eigen::Vector3d &target, JntArray &out);

    // calculates inverse kinematics numerically
    // should work for any arm with a valid forwards function
    bool backwards_num(const Eigen::Vector3d &target, JntArray &out);

    // checks if the position is possible to reach
    bool isReachable(const Eigen::Vector3d &target, bool print = false);

    // moves the position so it is reachable
    void clampToReachable(Eigen::Vector3d &target);

    // checks if the joint values are within the min and max constraints
    bool isJointsValid(const JntArray &joint_angles);

    // generates a random valid joint array
    void randomJntArray(JntArray &out);

    // minimum angles arms can achive
    // min should be less than max
    const JntArray min_angles = {-M_PI / 2, 0, -M_PI};
    const JntArray max_angles = {M_PI / 2, M_PI, 0};

private:
    double getError(const JntArray &joints, const Eigen::Vector3d &target);

    // the displacements between each joint frame
    // first item is the displacments between the base frame and the first joint
    Eigen::Vector3d displacements[num_joints + 1];
    // the axis around which the joint rotates, use Eigen::Vector3d::Unit?()
    Eigen::Vector3d joint_axis[num_joints];
};

#endif // KINEMATICS

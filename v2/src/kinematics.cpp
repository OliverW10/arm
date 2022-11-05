#include "kinematics.hpp"

ArmKinematics::ArmKinematics()
{
    // TODO measure these
    displacements[0] << 0, 0, 0;
    displacements[1] << 0.05, 0.015, 0.01;
    displacements[2] << 0.15, 0.015, 0.01;
    displacements[3] << 0.17, 0, 0;

    joint_axis[0] = Eigen::Vector3d::UnitZ();
    joint_axis[1] = Eigen::Vector3d::UnitY();
    joint_axis[2] = Eigen::Vector3d::UnitY();

    srand((unsigned int) time(0));
}

// #define DEBUG_FORWARDS

Eigen::Vector3d ArmKinematics::forwards(const JntArray &joint_angles)
{
    /*
    does the forwards kinematics by multiplying transformation matricies

        [   rotation 3x3  , translation 3x1 ]
        [       ...       ,      ...        ]
        [       ...       ,      ...        ]
        [       ...       ,      ...        ]
        [    0,   0,   0  ,       1         ]
    */

    // initialize to identity so the first joint becomes itself
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();

    // each loop add the displacment to get to the joint and the joints rotation
    for (int i = 0; i < num_joints; ++i)
    {
        // create transform from previous joint to current joint frame
        Eigen::Matrix4d joint_transform;
        // create bottom row to preserve 1 in vector
        joint_transform(3, 0) = 0;
        joint_transform(3, 1) = 0;
        joint_transform(3, 2) = 0;
        joint_transform(3, 3) = 1;
        // create rotation matrix
        joint_transform.block<3, 3>(0, 0) = Eigen::AngleAxisd(-joint_angles(i), joint_axis[i]).matrix();
        // get transform vector
        joint_transform.block<3, 1>(0, 3) = displacements[i];

        // multiply into full transform
        out *= joint_transform;
#ifdef DEBUG_FORWARDS
        std::cout << "joint:\n"
                  << joint_transform << "\n";
        std::cout << "full:\n"
                  << out << "\n";
#endif
    }
    // apply final segments displacments
    Eigen::Matrix4d joint_transform = Eigen::Matrix4d::Identity();
    joint_transform.block<3, 1>(0, 3) = displacements[num_joints];
    out *= joint_transform;
#ifdef DEBUG_FORWARDS
    std::cout << "joint:\n"
              << joint_transform << "\n";
    std::cout << "full:\n"
              << out << "\n";
#endif
    return out.block<3, 1>(0, 3);
}

bool ArmKinematics::backwards_geo(const Eigen::Vector3d &target, JntArray &out)
{
    // does inverse kinematics geometrically, assumes each joint's displacment is only in x
    // TODO: investigate other ways to do analytic ik
    double x = target(0);
    double y = target(1);
    double z = target(2);

    // get x component of the displacment for the shoulder and elbow joints
    double a = displacements[2](0);
    double b = displacements[3](0);
    // x of displacment for turret joint
    double t = displacements[1](0);

    out(0) = atan2f(y, x);
    // distances from end of the flat segment to the target
    // distance in only the x and y axies
    double dz = sqrt(x * x + y * y) - t;
    // overall distance
    double d = sqrt(x * x + y * y + z * z) - t;
    // squared used in cosine rule
    double d_squared = d * d;
    out(1) = atan2(z, dz) + acos((a * a + d_squared - b * b) / (2 * a * d));
    // gets pi-interiour angle and *-1 all
    out(2) = acos((a * a + b * b - d_squared) / (2 * a * b)) - M_PI;
    return !(out.isNaN().any() || out.isInf().any());
}

bool ArmKinematics::backwards_num(const Eigen::Vector3d &target, JntArray &out)
{
    // delta used to calculate derivative
    const double h = 1e-9;
    // how much to move towards esimated zero
    const double step = 0.03;
    const double allowable_error = 0.00025; // 0.25mm

    int tries = 0;
    int iterations = 0;

    // initalize to result of geometric backwards
    bool ret = backwards_geo(target, out);
    if(!ret){
        return false;
    }

    JntArray next_joints;
    double base_error = 1e10;
    while (base_error > allowable_error)
    {
        // get the error of current joint angles
        base_error = getError(out, target);
        // std::cout << "\nbase error: " << base_error*100 << "cm\n";

        // find the partial derivative with resprect to each joint
        // and move step% of the way to its estimated zero
        for (int i = 0; i < num_joints; ++i)
        {
            out(i) += h;
            double d = (base_error - getError(out, target)) / h;
            out(i) -= h;
            next_joints(i) = out(i) + d * step;
            // std::cout << "joint " << i << " derivative: " << d << "\tangle: " << next_joints(i) << "\n";
        }
        out = next_joints;
        // check if joints have left the achiveable area
        if (isJointsValid(out) == false)
        {
            // reset to random joints
            randomJntArray(out);
            std::cout << "IK: reached invalid joint position, reinitializing randomly\n";
            if(isJointsValid(out) == false){
                std::cout << "created joint is bad\n";
            }
            tries ++;
            iterations = 0;
            // prevent failure if position is unreachable
            if (tries > 5)
            {
                return false;
            }
        }
        iterations ++;
        if (iterations > 500){
            std::cout << "IK: reached iteration cap of 500 with an error of " << base_error*100.0 << "cm\n";
            return true; // still go something so return that
        }
    }
    std::cout << "IK: reached desired accuracy of " << allowable_error*100.0 << "cm in " << iterations << " iterations\n";
    return true;
}

bool ArmKinematics::isReachable(const Eigen::Vector3d &target)
{
	double x = target(0);
	double y = target(1);
	double z = target(2);
	
	// arm can reach behind itself but the ik dosent do that yet
	if(x <= 0)
		return false;

	// check min and max dists
	double dist = target.norm();
	double min_dist = displacements[1].norm();
	double max_dist = min_dist + displacements[2].norm() + displacements[3].norm();
	if(dist < min_dist || dist > max_dist)
		return false;

	// 
    return true;
}

bool ArmKinematics::isJointsValid(const JntArray &joint_angles)
{
    return (
        (joint_angles.array() > min_angles).all() &&
        (joint_angles.array() < max_angles).all());
}

void ArmKinematics::randomJntArray(JntArray &out)
{
    // random gives -1 to 1 so abs to get 0 to 1
    out = JntArray::Random().abs();
    // scale by range of motion and shift
    // does operations elementwise beacuse they are Arrays
    out = out * (max_angles - min_angles) + min_angles;
}

double ArmKinematics::getError(const JntArray &joints, const Eigen::Vector3d &target)
{
    return (forwards(joints) - target).norm();
}

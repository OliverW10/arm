#ifndef KINEMATICS
#define KINEMATICS

#include <iostream>
#include <math.h>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <Eigen/Dense>

class ArmKinematics{
public:
	ArmKinematics();
	void forwards(KDL::JntArray& _joint_array, KDL::Frame& out);
	void forwards(KDL::Frame& out);
	void backwards(KDL::JntArray& initial, KDL::Frame& goal, KDL::JntArray& out);
	void backwards(KDL::Frame& goal, KDL::JntArray& out);

	KDL::JntArray joint_angles;
	static Eigen::MatrixXd weights;
private:
	KDL::ChainFkSolverPos_recursive fk;
	KDL::ChainIkSolverPos_LMA ik;
	KDL::Chain chain;
};

void printFrame(KDL::Frame& frame);
void printJntAngles(KDL::JntArray& jnts);

#endif // KINEMATICS

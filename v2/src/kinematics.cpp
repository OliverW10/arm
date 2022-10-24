#include "kinematics.hpp"
#include <kdl/jntarray.hpp>

void printFrame(KDL::Frame& frame){
	std::cout << "x: " << frame.p.x() << ", y: " << frame.p.y() << ", z: " << frame.p.z() << "\n";
}

void printJntAngles(KDL::JntArray& jnts){
	for(int i = 0; i < jnts.rows(); ++i){
		std::cout << jnts(i) << ", "; 
	}
	std::cout << "\b\b \n";
}

// ArmKinematics::weights = Eigen::MatrixXd::Zero(6, 1);

ArmKinematics::ArmKinematics()
    : fk(KDL::ChainFkSolverPos_recursive(chain))
    , ik(KDL::ChainIkSolverPos_LMA(chain, 0.1, 100))
{
	// define arm segments
	chain.addSegment(
		KDL::Segment(
			KDL::Joint(KDL::Joint::JointType::RotZ),
			KDL::Frame(KDL::Vector(0.05, 0, 0))
	));
	chain.addSegment(
		KDL::Segment(
			KDL::Joint(KDL::Joint::JointType::RotY),
			KDL::Frame(KDL::Vector(0.15, 0, 0))
	));
	chain.addSegment(
		KDL::Segment(
			KDL::Joint(KDL::Joint::JointType::RotY),
			KDL::Frame(KDL::Vector(0.15, 0, 0))
	));


	// set joint angles
	joint_angles = KDL::JntArray(chain.getNrOfJoints());
	KDL::SetToZero(joint_angles);
	joint_angles(0) = 0.0;
	double d = 0.1;
	joint_angles(1) = -M_PI/2 + d + 0.3;
	joint_angles(2) = M_PI - 2*d;
}

void ArmKinematics::forwards(KDL::JntArray& _joint_angles, KDL::Frame& out){
	fk.JntToCart(_joint_angles, out);
}

void ArmKinematics::forwards(KDL::Frame& out){
	forwards(joint_angles, out);
}

void ArmKinematics::backwards(KDL::Frame& target, KDL::JntArray& out){
	ik.CartToJnt(joint_angles, target, out);
	joint_angles = out;
}


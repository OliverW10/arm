#include "kinematics.hpp"
// #include <rs2>


int main(){
    ArmKinematics arm;
	KDL::Frame end_pose;
	arm.forwards(end_pose);
	std::cout << "joint angles: ";
	printJntAngles(arm.joint_angles);
	std::cout << "forwards: " << end_pose.p.x() << ", " << end_pose.p.y() << ", " << end_pose.p.z() << "\n";
}

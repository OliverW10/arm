#ifndef ARMH
#define ARMH

#include <iostream>
#include <pigpio.h>
#include <Eigen/Dense>
#include <chrono>
#include <algorithm>
#include <cmath>
#include "kinematics.hpp"
#include <signal.h>

#define SIM
// #define DEBUG_PRINT

class Arm
{
public:
    Arm(int servo_pins[]);
    ~Arm();
    bool setGoal(Eigen::Vector3d goal);
    void execute();

private:
    // maximum joint rotation speed (rad/s) and accel (rad/s^2)
    // TODO: measure this
    const double MAX_SPEED = 1.0;
    const double MAX_ACCEL = 1.0;

    Eigen::Vector3d pos_goal;
    JntArray jnt_goal;

    // the arm uses servo's which don't give feedback on position or velocity
    // and just move at max speed to their given goal, to have more control the
    // speed and acceleration are simulated and the resulting position is fed
    // into the servo's, this will work given the max speed and accel are below
    // that of the real servo and the servo dosent experience any major
    // distrubrances
    JntArray jnt_positions;
    JntArray jnt_speeds;

    ArmKinematics kinematics;
    int *servo_pins;
    // last iteration in microseconds
    long last_time;

    // offset from
    const JntArray jnt_offsets = {0, 0, 0};
    const JntArray jnt_inverted = {1, 1, 1};

    // sends positions to the servos
    void sendCommands();
};

#endif // ARMH

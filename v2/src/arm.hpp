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

// define sim if not running on a pi
#define SIM
// define debug print if you want lots of debug printing (TODO: idk bout this)
// #define DEBUG_PRINT

class Arm
{
public:
    Arm(int _servo_pins[]);
    ~Arm();
    bool setGoal(Eigen::Vector3d goal);
    bool setJoints(const JntArray &jnts);
    void execute();
    ArmKinematics kinematics;
private:
    // current goal positions
    JntArray jnt_goal;
    // euclidean position goal
    Eigen::Vector3d pos_goal;

    // the arm uses servo's which don't give feedback on position or velocity
    // and just move at max speed to their given goal, to have more control the
    // speed and acceleration are simulated and the resulting position is fed
    // into the servo's, this will work given the max speed and accel are below
    // that of the real servo and the servo dosent experience any major
    // distrubrances
    // current commanded positions
    JntArray jnt_positions;
    // estimated/desired speeds
    JntArray jnt_speeds;

    // maximum joint rotation speed (rad/s) and accel (rad/s^2)
    // TODO: measure this
    const double MAX_SPEED = 1.0;
    const double MAX_ACCEL = 1.0;
    // offsets from physical zero's
    const JntArray jnt_offsets = {0, 0, 0};
    // invert servo direction (1 or -1)
    const JntArray jnt_inverted = {1, 1, 1};
    int *servo_pins;

    // last iteration in microseconds
    long last_time;
    bool first;


    // sends positions to the servos
    void sendCommands();
};

#endif // ARMH

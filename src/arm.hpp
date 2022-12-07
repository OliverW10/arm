#ifndef ARMH
#define ARMH

#include <Eigen/Core>
#include <chrono>
#include "kinematics.hpp"
#include "../real.hpp"

// define debug print if you want lots of debug printing (TODO: idk bout this)
// #define DEBUG_PRINT

// this is kinda arbritary but has to be over 100
#define FAKE_PWM_RANGE 4000
// we get 9.25% of this many steps per 180 degrees

class Arm
{
public:
    Arm(int _servo_pins[]);
    ~Arm();
    // sets euclidean target
    bool setGoal(Eigen::Vector3d goal, bool override = false);
    // set joint goals
    // override: don't smooth motion
    bool setJoints(const JntArray &jnts, bool override = false);
    // if the smoothed position is at the goal position
    bool atGoal();
    void execute();
    ArmKinematics kinematics;
private:
    // current goal positions
    JntArray jnt_goal;

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
    const double MAX_SPEED = 5.0;
    const double MAX_ACCEL = 10.0;
    // pwm values for min and max angles joints can have
    const JntArray jnt_pwm_min = {2400, 2400, 2400};
    const JntArray jnt_pwm_max = {550, 700, 550};
    int *servo_pins;

    // last iteration in microseconds
    long last_time;
    bool first;

    void clean();
    // sends positions to the servos
    void sendCommands();
};

#endif // ARMH

#include "arm.hpp"

// getting weird errors of multple defenitions when putting these in header so they go here
// gets current unix echo time in nanoseconds
long getTimeNs()
{
    return std::chrono::steady_clock::now().time_since_epoch().count();
}

// converts long of nanoseconds to double of seconds
// do this after finding the delta to not lose time from doubles lack of precision
double NsToS(long ns)
{
    //                  micro    milli     s
    return (double)ns / 1000.0 / 1000.0 / 1000.0;
}

Arm::Arm(int _servo_pins[]) : kinematics(ArmKinematics())
{
#ifndef SIM
    // change sample rate and ?peripheral used for timing?
    // gpioCfgClock(4, 1, 0);
    if (gpioInitialise() < 0)
    {
        std::cerr << "Couldn't initialise pigpio\n";
        exit(-1);
    }
    gpioSetSignalFunc(SIGINT, quitHandler);
    for(int i = 0; i < num_joints; ++i){
        // servo's use 50hz pwm
        // gpioSetPWMfrequency(_servo_pins[i], 50);

        // PWM range is the denominator of the duty cycle fraction
        // gpioPwm() sets the numerator
        // gpioSetPWMrange(_servo_pins[i], FAKE_PWM_RANGE);

        // according to this https://abyz.me.uk/rpi/pigpio/cif.html#gpioSetPWMrange
        // we have 1000 (at default sample rate) steps between 100% and 0% duty cycle
        // we are only interested in (550 to 2400)/20000 = 9.25% of that or ~92 steps
    }
#endif
    jnt_goal = JntArray(0, M_PI / 2, -M_PI / 2);
    // assume it starts in startup position since we cant read it actual position
    jnt_positions = jnt_goal;
    jnt_speeds.setZero();

    servo_pins = _servo_pins;

    last_time = getTimeNs();
    first = true;
}

void quitHandler(int signum){
    exit(0);
}

Arm::~Arm()
{
    std::cout << "arm destructor called\n";
#ifndef SIM
    for (int i = 0; i < num_joints; ++i)
    {
        gpioServo(servo_pins[i], 0);
    }
    gpioTerminate();
#endif
}

bool Arm::setGoal(Eigen::Vector3d goal, bool override)
{
    Eigen::Vector3d pre_clamp_goal = goal;
    kinematics.isReachable(goal, true);
    kinematics.clampToReachable(goal);
    // using geo for now cause somehow num broke
    JntArray last_jnt_goal = jnt_goal;
    bool success = kinematics.backwards_geo(goal, jnt_goal);
    // don't set to an invalid goal
    if(!success){
        jnt_goal = last_jnt_goal;
    }
    if(first || override){
        jnt_positions = jnt_goal;
    }
    // std::cout << "goal:\n" << jnt_goal << "\n";
    first = false;
    return success;
}

bool Arm::setJoints(const JntArray &jnts, bool override)
{
    jnt_goal = jnts;
    if(first || override){
        jnt_positions = jnt_goal;
    }
    first = false;
    return kinematics.isJointsValid(jnts);
}

void Arm::execute()
{
    long cur_time = getTimeNs();
    double dt = NsToS(cur_time - last_time);
    last_time = cur_time;

    for (int i = 0; i < num_joints; ++i)
    {
        double error = jnt_goal[i] - jnt_positions[i];
        double a_error = abs(error);
        // calculate the maximum speed joint should be at for its current position
        // distance to stop from a speed u with an acceleration of a is
        // (u^2)/a so max speed would be sqrt(dist * accel)
        double cur_max_speed = sqrt(a_error * MAX_ACCEL);
        // get what speed we want to be going at now
        // min of maximum current speed and maximum achiveable joint speed in direction of error
        double target_speed = copysign(std::min(cur_max_speed, MAX_SPEED), error);
        double accel = std::clamp((target_speed - jnt_speeds[i]) / dt, -MAX_ACCEL, MAX_ACCEL);
        jnt_speeds[i] += accel * dt;
        jnt_positions[i] += jnt_speeds[i] * dt;
        // std::cout << "joint max speed: " << cur_max_speed
        // << ",\tcur speed: " << jnt_speeds[i]
        // << ",\ttarget speed: " << target_speed
        // << ",\taccel: " << accel << "\n";
    }

    sendCommands();
}

void Arm::sendCommands()
{

#ifdef DEBUG_PRINT
    std::cout << "sending [(angle, pwm)]: ";
#endif
    for (int i = 0; i < num_joints; ++i)
    {
        double angle = jnt_positions[i];
        // covert arm coordinate system to servo angle 0-1
        // double servo_angle = (angle - kinematics.min_angles[i]) / (kinematics.max_angles[i] - kinematics.min_angles[i]);
        double servo_angle = (angle-kinematics.min_angles[i]) / (kinematics.max_angles[i]-kinematics.min_angles[i]);
        servo_angle = std::clamp(servo_angle, 0.0, 1.0);
        // scale angle to pulse width, in micro seconds
        unsigned int servo_pulse_width = jnt_pwm_min[i] + servo_angle * (jnt_pwm_max[i]-jnt_pwm_min[i]);
        // work out pwm duty cycle
        // servo pulse width / microseconds per cycle get duty cycle (20 000), then * fake pwm range
        // rearranged so it works with only ints
        unsigned int servo_duty_cycle = (servo_pulse_width * FAKE_PWM_RANGE) / 20000;
#ifndef SIM
        // gpioPWM(servo_pins[i], servo_duty_cycle);
        gpioServo(servo_pins[i], servo_pulse_width);
#endif

#ifdef DEBUG_PRINT
        std::cout << "(" << std::round(jnt_positions[i] * 100.0) / 100.0 << ", " << servo_pulse_width << "), ";
#endif
    }

#ifdef DEBUG_PRINT
    std::cout << "\n";
#endif
}

bool Arm::atGoal()
{
    // TODO: write this
    return true;
}
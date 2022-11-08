#include "arm.hpp"


// getting weird errors of multple defenitions when putting these in header so they go here
// gets current unix echo time in nanoseconds
long getTimeNs(){
    return std::chrono::steady_clock::now().time_since_epoch().count();
}

// converts long of nanoseconds to double of seconds
// do this after finding the delta to not lose time from doubles lack of precision 
double NsToS(long ns){
    //                  micro    milli     s
    return (double)ns / 1000.0 / 1000.0 / 1000.0;
}


Arm::Arm(int _servo_pins[]) : kinematics(ArmKinematics()){
    #ifndef SIM
    // most of the servo code was borrowed from the 'Servo Pulse Generator'
    // in the C examples section here: http://abyz.me.uk/rpi/pigpio/examples.html
    if(gpioInitialise() < 0){
        std::cerr << "Couldn't initialise pigpio\n";
        abort();
    }
    #endif
    jnt_goal = JntArray(0, M_PI/2, -M_PI/2);
    pos_goal = kinematics.forwards(jnt_goal);
    // assume it starts in startup position since we cant read it actual position
    jnt_positions = jnt_goal;
    jnt_speeds.setZero();

    servo_pins = _servo_pins;

    last_time = getTimeNs();
}

Arm::~Arm(){
    #ifndef SIM
    for(int i = 0; i < num_joints; ++i){
        gpioServo(servo_pins[i], 0);
    }
    gpioTerminate();
    #endif
}

bool Arm::setGoal(const Eigen::Vector3d &goal){
    // TODO: clamp goal to reachable
    pos_goal = goal;
    bool success = kinematics.backwards_num(goal, jnt_goal);
    // std::cout << "goal:\n" << jnt_goal << "\n";
    return success;
}

void Arm::execute(){
    long cur_time = getTimeNs();
    double dt = NsToS(cur_time - last_time);
    last_time = cur_time;

    for(int i = 0; i < num_joints; ++i){
        double error = jnt_positions[i] - jnt_goal[i];
        double a_error = abs(error);
        // calculate the maximum speed joint should be at for its current position
        // distance to stop from a speed u with an acceleration of a is
        // (u^2)/a so max speed would be sqrt(dist * accel)
        double cur_max_speed = sqrt(a_error * MAX_ACCEL);
        // get what speed we want to be going at now
        // min of maximum current speed and maximum achiveable joint speed in direction of error
        double target_speed = copysign(std::min(cur_max_speed, MAX_SPEED), -error);
        double accel = std::clamp((target_speed - jnt_speeds[i])/dt, -MAX_ACCEL, MAX_ACCEL);
        jnt_speeds[i] += accel * dt;
        jnt_positions[i] += jnt_speeds[i] * dt;
        // std::cout << "joint max speed: " << cur_max_speed
        // << ",\tcur speed: " << jnt_speeds[i]
        // << ",\ttarget speed: " << target_speed
        // << ",\taccel: " << accel << "\n";
    }

    sendCommands();
}

void Arm::sendCommands(){

    #ifdef DEBUG_PRINT
    std::cout << "sending [(angle, pwm)]: ";
    #endif
    for(int i = 0; i < num_joints; ++i){
        double angle = jnt_positions[i]+jnt_offsets[i];
        // covert arm coordinate system to servo angle 0-1
        double servo_angle = (angle-kinematics.min_angles[i]) / (kinematics.max_angles[i]-kinematics.min_angles[i]);
        unsigned int servo_pulse_width = 1000 + servo_angle * 1000;
        #ifndef SIM
        gpioServo(servo_pins[i], servo_pulse_width);
        #endif

        #ifdef DEBUG_PRINT
        std::cout << "(" << std::round(jnt_positions[i]*100.0)/100.0 << ", " << servo_pulse_width << "), ";
        #endif
    }

    #ifdef DEBUG_PRINT
    std::cout << "\n";
    #endif
}

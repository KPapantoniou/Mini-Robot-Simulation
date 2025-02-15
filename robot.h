#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include "params.h"
#include "motor.h"
#include "force_calculator.h"

class Robot{
private:
    std::vector<Motor*> motors;
    ForceCalculator forceCalculator;
    double mass;
    double g;
    double friction_coefficient;
    double l;
    double pos_x = 0.0, pos_z = 0.0, pos_y = 0.0;
    double vel_x = 0.0, omega = 0.0, vel_y = 0.0;
    double acc_x = 0.0, acc_rot = 0.0, acc_y=0.0;
    double force_x = 0.0, force_z = 0.0;
    double moment_x = 0.0, moment_z = 0.0;
    double theta_displacement = 0.0;
public:
    Robot(const RobotParams &params, ForceCalculator& forceCalculator );
    void addMotor(Motor& motor);
    // std::pair<double,double> calculateMovment(double t);
    // void calculateMovment(double t, bool is_asychronus, double dt);
    std::pair<double,double> calculate_single_motor(double t, double dt);
    std::pair<double,double> calculate_asynchronous_movement(double t, double dt);
    double sign(double x) const;
    double velocity_calculator(double acc, double vel, double t);
    double linear_acceleration_calculator(double force, double mass);
    double position_calculator(double vel, double pos, double accel, double t);
    double friction_calculator(double velocity, double force, double f_z) const;
    double coulob_friction_calculator(double f_z) const;
    double inertia(double radius) const;
    double angular_acceleration_calculator(double torque) const;
    double angular_velocity_calculator(double acceleration, double velocity, double t) const;
    double get_pos_x() const;
    double get_pos_y() const;
    double get_vel_x() const;
    double get_vel_z() const;
    double get_acc_x() const;
    double get_acc_z() const;
    double get_force_x() const;
    double get_force_z() const;
    double get_theta_displacement() const;
    double get_moment_x() const;
    double get_moment_z() const;
    
};

#endif
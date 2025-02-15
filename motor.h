#ifndef MOTOR_H
#define MOTOR_H

#include "params.h"  
#include <cmath>

class Motor {
public:
    Motor(const MotorParams &params);  
    double angular_acceleration(double omega, double theta) const;
    void update(double step);
    double get_omega() const;
    double get_theta() const;
    double get_voltage() const;
    double get_mass() const;

private:
    double resistance;
    double voltage;
    double mass;
    double t_mechanical;
    double omega_rated_speed;
    double start_current;
    double operational_current;
    double radius;
    double g;

    double omega = 0.0;
    double theta = 0.0;


    double sign(double x) const;
    double damping_calculator() const;
    double inertia_calculator() const;
    double coulomb_friction() const;
    double constant_Torque_calculator() const;
    void set_omega(double omega) const;
    void set_theta(double theta) const;
    
};

#endif

#ifndef PARAMS_H
#define PARAMS_H

#include <vector>

#define M_PI 3.14159265358979323846

struct MotorParams {
    double resistance;
    double voltage; 
    double mass;
    double g;
    double t_mechanical;
    double omega_rated_speed;
    double start_current;
    double operational_current;
    double radius;
};

struct ForceParams {
    double mass;
    double radius;
    double g;
};

struct RobotParams{
    double mass;
    double g;
    double friction_coefficient;
    double l;
};
#endif


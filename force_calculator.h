
#ifndef FORCE_CAL_H
#define FORCE_CAL_H

#include <cmath>
#include "params.h"

class ForceCalculator{
public:
    ForceCalculator(const ForceParams &params);
    double horizontal_centrifugal_force(double omega, double theta, double t) const;
    double vertical_centrifugal_force(double omega, double theta, double t) const;
    std::pair<double,double>calculate_robot_forces( double omega, double theta, double t) const;
    std::pair<double, double> calculate_torques(std::pair<double,double> omega, std::pair<double,double>  theta, double d) const;
private:
    double mass, radius, g;
};
#endif
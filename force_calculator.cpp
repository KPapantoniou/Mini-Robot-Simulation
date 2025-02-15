#include "force_calculator.h"
#include <iostream>

ForceCalculator::ForceCalculator(const ForceParams &params)
    :mass(params.mass), radius(params.radius), g(params.g){
        // std::cout<<"radius in constructor f: "<<radius<<"\n";
    }

double ForceCalculator::horizontal_centrifugal_force(double omega, double theta, double t) const{
    double temp = mass*pow(omega,2)*radius*sin(theta);
    
    // std::cout<<"fy in fy: %f"<<mass<<"\n";
    return temp;
}

double ForceCalculator::vertical_centrifugal_force(double omega, double theta,double t) const{
    return -mass*g-mass*pow(omega,2)*radius*cos(theta);
}

std::pair<double,double> ForceCalculator::calculate_robot_forces( double omega, double theta, double t) const{
    double force_X = horizontal_centrifugal_force(omega,theta,t);
    // std::cout<<"t in f in forceCa: "<<t<<"\n";
    double force_Z = vertical_centrifugal_force(omega,theta,t);
    return{force_X,force_Z};
}

std::pair<double, double> ForceCalculator:: calculate_torques(std::pair<double,double> omega, std::pair<double,double>  theta, double d) const {

    double M_x = (d * mass * pow(omega.first, 2) * radius * cos(theta.first)) - (d * mass*pow(omega.second, 2) * radius * cos(theta.second));
    double M_z = -( d * mass * pow(omega.first, 2) * radius * sin(theta.first)) - (d * mass*pow(omega.second, 2) * radius * sin(theta.second));
    // std::cout<<"M_z: "<<M_z<<"\n"<<theta.first<<"\n"<<theta.second<<"\n";
    return {M_x, M_z};
}


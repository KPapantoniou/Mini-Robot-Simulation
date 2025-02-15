#include "motor.h"
#include <iostream>

Motor::Motor(const MotorParams &params)
    : resistance(params.resistance),
      voltage(params.voltage), mass(params.mass),
      g(params.g),
      t_mechanical(params.t_mechanical),
      omega_rated_speed(params.omega_rated_speed),
      start_current(params.start_current),
      operational_current(params.operational_current),
      radius(params.radius),
      omega(0.0),
      theta(0.0){} 

double Motor::angular_acceleration(double omega, double theta) const {    
    return (constant_Torque_calculator() * voltage) / (inertia_calculator() * resistance)
    - ((damping_calculator() * resistance + pow(constant_Torque_calculator(), 2)) * omega) / (inertia_calculator() * resistance)
    - (mass * g * radius * sin(theta)) / inertia_calculator()
    - (coulomb_friction() * sign(omega)) / inertia_calculator();
}

double Motor::sign(double x) const {
    return (x > 0) - (x < 0);
}

double Motor::damping_calculator() const{
    // return (operational_current*constant_Torque_calculator()-coulomb_friction()*sign(omega_rated_speed))/omega_rated_speed;
    return 2.94e-9;
}

double Motor::inertia_calculator() const{
    // return t_mechanical*(pow(constant_Torque_calculator(),2)+resistance*damping_calculator())/resistance;
    return 2.67e-9;
}

double Motor::coulomb_friction() const{
    // return start_current*constant_Torque_calculator();
    return 1.34e-5;
}

double Motor::constant_Torque_calculator() const{
    // return (voltage-resistance*operational_current)/omega_rated_speed;
    return 3.64e-4;
}

void Motor::update(double step){
    // std::cout << "Before update: omega = " << omega << ", theta = " << theta << "\n";

    double k1 = omega*step;
    double l1 = step*(angular_acceleration(omega,theta));

    double k2 = step*(omega +0.5*k1);
    double l2 = step*(angular_acceleration(omega+0.5*l1,theta + 0.5*k1));

    double k3 = step*(omega+0.5*l2);
    double l3 = step*(angular_acceleration(omega+0.5*l2,theta+0.5*k2));

    double k4 = step*(omega+k3);
    double l4 = step*(angular_acceleration(omega+l3,theta + k3));

    theta += (k1+2*k2+2*k3+k4)/6;
    omega += (l1 + 2*l2+ 2*l3+ l4)/6;
    
    // std::cout << "After update: omega = " << omega << ", theta = " << theta << "\n";

    // set_omega(omega);
    // set_theta(theta);
}
// void Motor::update(double step) {
//     // Coefficients for the 5th-order Runge-Kutta method
//     double k1_theta = omega * step;
//     double l1_omega = angular_acceleration(omega, theta) * step;

//     double k2_theta = (omega + 0.25 * l1_omega) * step;
//     double l2_omega = angular_acceleration(omega + 0.25 * l1_omega, theta + 0.25 * k1_theta) * step;

//     double k3_theta = (omega + 3.0/32.0 * l1_omega + 9.0/32.0 * l2_omega) * step;
//     double l3_omega = angular_acceleration(omega + 3.0/32.0 * l1_omega + 9.0/32.0 * l2_omega, theta + 3.0/32.0 * k1_theta + 9.0/32.0 * k2_theta) * step;

//     double k4_theta = (omega + 1932.0/2197.0 * l1_omega - 7200.0/2197.0 * l2_omega + 7296.0/2197.0 * l3_omega) * step;
//     double l4_omega = angular_acceleration(omega + 1932.0/2197.0 * l1_omega - 7200.0/2197.0 * l2_omega + 7296.0/2197.0 * l3_omega,
//                                            theta + 1932.0/2197.0 * k1_theta - 7200.0/2197.0 * k2_theta + 7296.0/2197.0 * k3_theta) * step;

//     double k5_theta = (omega + 439.0/216.0 * l1_omega - 8.0 * l2_omega + 3680.0/513.0 * l3_omega - 845.0/4104.0 * l4_omega) * step;
//     double l5_omega = angular_acceleration(omega + 439.0/216.0 * l1_omega - 8.0 * l2_omega + 3680.0/513.0 * l3_omega - 845.0/4104.0 * l4_omega,
//                                            theta + 439.0/216.0 * k1_theta - 8.0 * k2_theta + 3680.0/513.0 * k3_theta - 845.0/4104.0 * k4_theta) * step;

//     double k6_theta = (omega - 8.0/27.0 * l1_omega + 2.0 * l2_omega - 3544.0/2565.0 * l3_omega + 1859.0/4104.0 * l4_omega - 11.0/40.0 * l5_omega) * step;
//     double l6_omega = angular_acceleration(omega - 8.0/27.0 * l1_omega + 2.0 * l2_omega - 3544.0/2565.0 * l3_omega + 1859.0/4104.0 * l4_omega - 11.0/40.0 * l5_omega,
//                                            theta - 8.0/27.0 * k1_theta + 2.0 * k2_theta - 3544.0/2565.0 * k3_theta + 1859.0/4104.0 * k4_theta - 11.0/40.0 * k5_theta) * step;

//     // Update theta and omega using the weighted average of increments
//     theta += (25.0/216.0 * k1_theta + 1408.0/2565.0 * k3_theta + 2197.0/4104.0 * k4_theta - 1.0/5.0 * k5_theta);
//     omega += (25.0/216.0 * l1_omega + 1408.0/2565.0 * l3_omega + 2197.0/4104.0 * l4_omega - 1.0/5.0 * l5_omega);
// }

double Motor::get_omega() const {
    return omega;
}


double Motor::get_theta() const {
    return theta;
}

void Motor::set_omega(double omega) const{
    omega = omega;
}

void Motor::set_theta(double theta) const{
    theta = theta;
}

double Motor::get_voltage() const{
    return voltage;
}

double Motor::get_mass() const{
    return mass;
}
#include <cmath> 
#include "motor.h"
#include "force_calculator.h"
#include "robot.h"
#include <iostream>

using namespace std;
Robot::Robot(const RobotParams &params, ForceCalculator& forceCalculator)
:   mass(params.mass),
    g(params.g),
    friction_coefficient(params.friction_coefficient),
    l(params.l),
    forceCalculator(forceCalculator),
    pos_x(0.0),
    pos_z(0.0),
    pos_y(0.0),
    vel_x(0.0),
    omega(0.0),
    vel_y(0.0),
    acc_y(0.0),
    acc_x(0.0),
    acc_rot(0.0),
    force_x(0.0),
    force_z(0.0),
    moment_x(0.0),
    moment_z(0.0),
    
    theta_displacement(0.0)
    {}
double force_y = 0.0;
double angular_acc_tilt = 0.0;
double angular_vel_tilt = 0.0;
double angular_acc_rot = 0.0;
double angular_vel_rot = 0.0; 



void Robot::addMotor(Motor& motor) {
    motors.push_back(&motor); 
}

// void Robot::calculateMovment(double t, bool is_asychronus, double dt){
//     // cout<<"t in f in robot alcMovme: "<<t<<"\n";
//     if(!is_asychronus){
//         // calculate_synchronus_movment(t,dt);
//         calculate_asynchronous_movement(t,dt);
//     }else{
//         calculate_asynchronous_movement(t,dt);
//     }
// }

pair<double,double> Robot::calculate_single_motor(double t, double dt){
    force_x = 0.0;
    force_z = 0.0;
    
    double force_motor = 0.0;
    // cout<<"t in f in robot: "<<t<<"\n";
    double omega_left = motors[0]->get_omega();
    double theta_left = motors[0]->get_theta();

    auto forces_left = forceCalculator.calculate_robot_forces(omega_left,theta_left,t);

    double force_x_left = forces_left.first;

    double force_z_left = forces_left.second;

    force_z = force_z_left;
    force_motor = force_x_left;
    
    double normal_force = (mass * g-force_z);
    
    double friction_force = friction_calculator(vel_x,force_motor,normal_force);
    
    force_x = force_motor - friction_force;
    
    acc_x = linear_acceleration_calculator(force_x,mass);

    vel_x = velocity_calculator(acc_x,vel_x,dt);

    pos_x = position_calculator(vel_x,pos_x,acc_x,dt);

    return {friction_force,coulob_friction_calculator(normal_force)};

}

pair<double,double> Robot::calculate_asynchronous_movement(double t, double dt) {
    // cout << "In calculate_asynchronus"<<endl;
    force_x = 0.0;
    force_z = 0.0;
    moment_x = 0.0;
    moment_z = 0.0;

    constexpr double angle_A = M_PI / 4.0;     
    constexpr double angle_B = M_PI;
    constexpr double angle_C = 7.0 * M_PI / 4.0; 
    constexpr double rotational_damping_coefficient = 0.1;
    
    double omega_left = motors[0]->get_omega();
    double theta_left = motors[0]->get_theta();
    // cout << "First motor success"<<endl;
    double omega_right = motors[1]->get_omega();
    double theta_right = motors[1]->get_theta();
    // cout << "Geting the motors values\t" << omega_left << "\t" << omega_right << endl;
    // auto forces_left = forceCalculator.calculate_robot_forces(omega_left, omega_left*t, t);
    // auto forces_right = forceCalculator.calculate_robot_forces(omega_right, omega_right*t, t);
    auto forces_left = forceCalculator.calculate_robot_forces(omega_left, theta_left, t);
    auto forces_right = forceCalculator.calculate_robot_forces(omega_right, theta_right, t);

    double force_x_left = forces_left.first;
    double force_x_right = forces_right.first;
    double force_z_left = forces_left.second;
    double force_z_right = forces_right.second;

    force_x = force_x_left-force_x_right; 
    force_z = force_z_left + force_z_right;

    auto moments = forceCalculator.calculate_torques({omega_left, omega_right}, {theta_left, theta_right}, l / 2);
    // auto moments = forceCalculator.calculate_torques({omega_left, omega_right}, {omega_left*t, omega_right*t}, l / 2);

    moment_x = moments.first;  
    moment_z = moments.second; 

    double total_weight = mass * g - force_z; 
    double normal_force_A = total_weight / 3.0 + (moment_x *  cos(angle_A)) / (3.0 * (l/2));
    double normal_force_B = total_weight / 3.0 + (moment_x *  cos(angle_B)) / (3.0 * (l/2));
    double normal_force_C = total_weight / 3.0 + (moment_x *  cos(angle_C)) / (3.0 * (l/2));
    // double normal_force_A = total_weight / 3.0;
    // double normal_force_B = total_weight / 3.0;
    // double normal_force_C = total_weight / 3.0;

    // double normal_force_A = total_weight / 3.0 ;
    // double normal_force_B = total_weight / 3.0 ;
    // double normal_force_C = total_weight / 3.0 ;

    double sum_normal_forces = normal_force_A + normal_force_B + normal_force_C;
    
    double vel_A_x = vel_x - omega * (l / 2) * sin(angle_A);
    double vel_A_y = vel_y + omega * (l / 2) * cos(angle_A);
    double vel_B_x = vel_x - omega * (l / 2) * sin(angle_B);
    double vel_B_y = vel_y + omega * (l / 2) * cos(angle_B);
    double vel_C_x = vel_x - omega * (l / 2) * sin(angle_C);
    double vel_C_y = vel_y + omega * (l / 2) * cos(angle_C);

    double vel_A = vel_A_x * cos(angle_A) + vel_A_y * sin(angle_A);
    double vel_B = vel_B_x * cos(angle_B) + vel_B_y * sin(angle_B);
    double vel_C = vel_C_x * cos(angle_C) + vel_C_y * sin(angle_C);

    double friction_A = friction_calculator(vel_A, force_x*cos(angle_A), normal_force_A);  
    double friction_B = friction_calculator(vel_B, force_x*cos(angle_B), normal_force_B);
    double friction_C = friction_calculator(vel_C, force_x*cos(angle_B), normal_force_C);
    // cout<<"Friction A: "<<friction_A<<"\n"
    // <<"vel_A_x: "<<vel_A_x<<"\n"
    // <<"vel_A_y: "<<vel_A_y<<"\n";

    force_x += -(friction_A * cos(angle_A) + friction_B * cos(angle_B) + friction_C*cos(angle_C)); 

    force_y = -(friction_A * sin(angle_A) + friction_B * sin(angle_B) + friction_C * sin(angle_C));
    double rotational_friction =- (friction_A * (l / 2) * sin(angle_A)
                             + friction_B * (l / 2) * sin(angle_B)
                             + friction_C * (l / 2) * sin(angle_C));
    double total_torque_z = moment_z + rotational_friction;


    angular_acc_tilt = angular_acceleration_calculator(moment_x); 
    acc_rot = angular_acceleration_calculator(total_torque_z); 

    angular_vel_tilt = angular_velocity_calculator(angular_acc_tilt, angular_vel_rot, dt);
    omega = angular_velocity_calculator(acc_rot, omega, dt);

    theta_displacement += omega * dt + 0.5 * acc_rot * dt * dt;

    acc_x = linear_acceleration_calculator(force_x, mass);
    // acc_rot = linear_acceleration_calculator(force_z, mass);
    acc_y = linear_acceleration_calculator(force_y, mass);

    vel_x = velocity_calculator(acc_x, vel_x, dt);
    // omega = velocity_calculator(acc_rot, omega, dt);
    vel_y = velocity_calculator(acc_y, vel_y, dt);

    pos_x = position_calculator(vel_x, pos_x, acc_x, dt);
    
    pos_y = position_calculator(vel_y, pos_y, acc_y, dt);

    return {force_x_left+force_x_right,force_z_left+force_z_right};
}

double Robot::sign(double x) const {
    return (x > 0) - (x < 0);
}

double Robot::linear_acceleration_calculator(double force, double mass){
    return force/mass;
}

double Robot::velocity_calculator(double acc, double vel, double t){
    return acc*t+vel;
}

double Robot::position_calculator(double vel, double pos, double accel, double t){
    return vel*t + 0.5*accel*pow(t,2)+pos;
}

double Robot::friction_calculator(double velocity, double force, double n) const{
    double f_coulomb = coulob_friction_calculator(n); 
    if (velocity == 0) { 
        if (abs(force) <= f_coulomb) {
            return force; 
        } else {
            return f_coulomb * sign(force); 
        }
    }
    return f_coulomb * sign(velocity); 
}

double Robot::coulob_friction_calculator(double n) const{
    return friction_coefficient * n;
}

double Robot::inertia(double radius) const{
    double inertia_mass = mass*pow(radius,2)/2;
    return inertia_mass;
}

double Robot::angular_velocity_calculator(double acceleration, double velocity, double t) const{
    return acceleration*t+velocity;
}

double Robot::angular_acceleration_calculator(double torque) const{
    return torque/inertia(l/2);
}

double Robot::get_pos_x() const{
    return pos_x;
}

double Robot::get_pos_y() const{
    return pos_y;
}

double Robot::get_vel_x() const{
    return vel_x;
}

double Robot::get_vel_z() const{
    return omega;
}

double Robot::get_acc_x() const{
    return acc_x;
}

double Robot::get_acc_z() const{
    return acc_rot;
}

double Robot::get_force_x() const{
    return force_x;
}

double Robot::get_force_z() const{
    return force_z;
}

double Robot::get_theta_displacement() const{
    return theta_displacement;
}

double Robot::get_moment_x() const{
    return moment_x;
}

double Robot::get_moment_z() const{
    return moment_z;
}
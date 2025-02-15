#include "simulation.h"
using namespace std;

Simulation::Simulation(double step, int iterations, Motor& motor1,Motor& motor2, ForceCalculator& forceCalc, Robot& robot_single, Robot& robot_double)
    : step(step), iterations(iterations), motor1(&motor1), motor2(&motor2), forceCalc(forceCalc), robot_single(&robot_single), robot_double(&robot_double){
   
}


void Simulation::run(){
  
    ofstream file("D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\results_motor.csv");
    ofstream file1("D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\results_robot_single.csv");
    ofstream file2("D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\results_robot.csv");

    if(!file.is_open() || !file1.is_open()||!file2.is_open()){
        cerr << "Failed to open file" << endl;
        return;
    }

    if(!robot_double||!robot_single){
        cerr << "Robot not initialized" << endl;
        return;
    }
    
    file << "Time,Theta_right,Omega_right,Theta_left,Omega_left,Force_y_right,Force_z_right,Force_y_left,Force_z_left\n";
    file1 << "Time,Theta_left,Pos_x,Vel_x,Acc_x,Force_x,Coulomb_Force,Friction_x,Total_Force_Z\n";
    file2 << "Time,Pos_x,Pos_y,Vel_x,Acc_x,Total_Force_X,Total_Force_Z,Force_X_Motors,Force_Z_Motors,Theta_displacement,Moment_x,Moment_z\n";
    
    for(int iteration = 0; iteration<iterations; iteration++){
        double t = iteration * step;
        
       

        double fy = forceCalc.horizontal_centrifugal_force(motor1->get_omega(), motor1->get_theta(), t);
        double fz = forceCalc.vertical_centrifugal_force(motor1->get_omega(), motor1->get_theta(), t);
        double f2y = forceCalc.horizontal_centrifugal_force(motor2->get_omega(), motor2->get_theta(), t);
        double f2z = forceCalc.vertical_centrifugal_force(motor2->get_omega(), motor2->get_theta(), t);


        // Singular motor for robot
        double pos1 = robot_single->get_pos_x();
        double vel1 = robot_single->get_vel_x();
        double accel1 = robot_single->get_acc_x();
        auto friction_forces = robot_single->calculate_single_motor(t, step);
        
        // Asynchronous motors for robot
        double pos2_x = robot_double->get_pos_x();
        double pos2_y = robot_double->get_pos_y();
        double vel2 = robot_double->get_vel_x();
        double accel2 = robot_double->get_acc_x();
        auto forces = robot_double->calculate_asynchronous_movement(t, step);

    
        //write to file
        file << 
            t << "," <<
            motor1->get_theta() << "," <<
            motor1->get_omega() << "," <<
            motor2->get_theta() << "," <<
            motor2->get_omega() << "," <<
            fy << "," <<
            fz<<","<<
            f2y<<","<<
            f2z <<
            "\n";

        //write to file1
        file1 <<
            t << "," <<
            motor1->get_theta() << "," <<
            pos1 << "," <<
            vel1 << "," <<
            accel1 << "," << 
            robot_single->get_force_x()<<","<<
            friction_forces.second<<","<<
            friction_forces.first << "," << 
            robot_single->get_force_z() << 
            "\n";

        //write to file2
        file2 << 
            t << "," << 
            pos2_x << "," <<
            pos2_y << "," << 
            vel2 << "," << 
            accel2 << "," << 
            robot_double->get_force_x() << "," << 
            robot_double->get_force_z()<<","<<
            forces.first << "," <<
            forces.second << "," <<
            robot_double->get_theta_displacement()<<","<<
            robot_double->get_moment_x()<<","<<
            robot_double->get_moment_z() << "\n";

        motor1->update(step);
        motor2->update(step);
    }
    file.close();
    file1.close();
    file2.close();
    cout << "Data saved to results_motor.csv" << endl;
    cout << "Data saved to results_robot_single.csv" << endl;
    cout << "Data saved to results_robot.csv" << endl;
}

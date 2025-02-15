#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <iostream>
#include "motor.h"
#include "force_calculator.h"
#include "robot.h"
#include <vector>
#include <fstream>

class Simulation{
public:
    Simulation(double step, int iterations, Motor& motor1, Motor& motor2, ForceCalculator& forceCalc, Robot& robot_single, Robot& robot_double);
    
    void run();
private:
    double step;
    int iterations;
    Motor* motor1;
    Motor* motor2;
    ForceCalculator forceCalc;
    Robot* robot_single;
    Robot* robot_double;
    // double theta, omega;
};
#endif
#include "simulation.h"
#include "params.h"

using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cerr << "Command: RobotSimulation.exe {voltage_left} {voltage_right}"<<endl;
        return -1;

    }
    double voltage_left = atof(argv[1]);
    double voltage_right = atof(argv[2]);
    double step = 0.001;
    double iterations = 2000;

    MotorParams motor1Params;

    // motorParams.resistance = 34;
    // motorParams.voltage = 0.65;
    // motorParams.mass = 0.001;
    // motorParams.g = 9.81;
    // motorParams.t_mechanical = 0.033;
    // motorParams.omega_rated_speed = (16900 * 2 * M_PI) / 60;
    // motorParams.start_current = 0.1;
    // motorParams.operational_current = 0.064;
    // motorParams.radius = 0.002;
    
    motor1Params.resistance = 10.7;
    motor1Params.voltage = voltage_left;
    motor1Params.mass = 0.00021;
    motor1Params.g = 9.81;
    motor1Params.t_mechanical = 0.175;
    motor1Params.omega_rated_speed = (16900 * 2 * M_PI) / 60;
    motor1Params.start_current = 0.1;
    motor1Params.operational_current = 0.064;
    motor1Params.radius = 0.00177;

    MotorParams motor2Params;


    
    motor2Params.resistance = 10.7;
    motor2Params.voltage =voltage_right;
    motor2Params.mass = 0.00021;
    motor2Params.g = 9.81;
    motor2Params.t_mechanical = 0.175;
    motor2Params.omega_rated_speed = (16900 * 2 * M_PI) / 60;
    motor2Params.start_current = 0.1;
    motor2Params.operational_current = 0.064;
    motor2Params.radius = 0.00177;
    ForceParams forceParams;

    // forceParams.mass = 0.001;
    // forceParams.radius=0.002;
    // forceParams.g = 9.81;
    
    forceParams.mass = 0.00021;
    forceParams.radius=0.00177;
    forceParams.g = 9.81;

    RobotParams RobotParams;
    RobotParams.mass = 0.1;
    RobotParams.g = 9.81;
    RobotParams.friction_coefficient = 0.5;
    RobotParams.l = 0.072;
    
    Motor motor1(motor1Params);
    Motor motor2(motor2Params);

    ForceCalculator forceCalculator(forceParams);

    // std::cout<<motor1.get_omega()<<"\n";
    // return 0;
    Robot robot_single(RobotParams,forceCalculator);
    Robot robot_double(RobotParams,forceCalculator);
    
    robot_single.addMotor(motor1);
    robot_double.addMotor(motor1);
    robot_double.addMotor(motor2);

    Simulation Simulation(step, iterations, motor1, motor2, forceCalculator,robot_single,robot_double);
    Simulation.run();

    // std::cout<<motor1.get_omega()<<"\n";

    // _putenv("PYTHONHOME=");
    // int result = system("python D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\motor_plot.py");
    
    // if (result!=0){
    //     std::cerr << "Failed to execute motor_plot.py" << std::endl;

    // }

    // result = system("python D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\robot_single_motor_plot.py");
    // if(result!=0){
    //     std::cerr << "Failed to execute robot_single_motor_plot.py" << std::endl;
    // }

    // result = system("python D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\robot_double_plot.py");
    // if(result!=0){
    //     std::cerr << "Failed to execute robot_double_plot.py" << std::endl;
    // }
    // if(std::remove("results_motor.csv") != 0){
    //     std::cerr << "Failed to delete results.csv" << std::endl;
    // }else{
    //     std::cout << "results.csv deleted successfully" << std::endl;
    // }
    return 0;

}

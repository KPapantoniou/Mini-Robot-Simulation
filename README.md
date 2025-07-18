# Micro-Robot Simulation Project

This project simulates the behavior of a micro-robot that moves using centrifugal force and vibrations generated by two motors. The core simulation is written in C++ and generates data that can be visualized and analyzed using Python scripts. The simulation allows you to test and evaluate different control algorithms and robot dynamics based on motor behavior.

## Project Overview

The robot operates with two motors that generate centrifugal forces, which are modeled in this simulation. The robot's movement is based on the forces calculated from these motors, and the simulation helps visualize and analyze the system's behavior.

- **Motor dynamics:** The motors generate both horizontal and vertical centrifugal forces, which are calculated based on motor speed (`omega`) and angular displacement (`theta`).
- **Robot behavior:** The robot's position, velocity, and acceleration are calculated and updated based on the motor forces.
- **Control algorithms:** The simulation can be used to test different control algorithms, including feedback loops and motor coordination, for more accurate motion.

## Why This Project?

The aim of this project is to explore robot dynamics using unconventional movement principles (centrifugal forces and vibrations). It's particularly useful for understanding how robots with unconventional propulsion mechanisms, like rotating motors, behave in real-world scenarios. The project can also be extended to implement machine learning-based control algorithms or experiment with hardware-in-the-loop simulations.

## Features

* **Motor Simulation**: Simulates the rotational dynamics of individual DC motors, considering their electrical and mechanical properties.
* **1-DOF Robot Simulation**: Models a simplified robot moving in one dimension, driven by a single centrifugal motor.
* **2-DOF Asynchronous Robot Simulation**: Simulates a more complex robot with two motors, allowing for movement and rotation in a 2D plane based on asynchronous motor operation.
* **PID/PI Controller Implementation**: Includes PID and PI controllers to regulate motor speed, demonstrating their impact on performance.
* **Simulated Annealing (SA) for PID Tuning**: Provides a function to automatically search for optimal PID controller parameters to minimize error.
* **CSV Output**: All simulation data is saved to CSV files, making it easy to analyze and visualize.
* **Python Visualization**: Companion Python scripts are provided to visualize the simulation results using `pandas` and `matplotlib`.

## Simulation Setup

Before running the simulation, ensure that all dependencies are installed. The simulation requires C++ and certain libraries for file I/O and numerical calculations.

### Dependencies

- **C++ Compiler:** GCC or any modern C++ compiler.
- **CMake:** For building the project.
- **Standard Libraries:** Used for I/O and file handling (`iostream`, `fstream`, etc.).
- **Python** 3.x
- **Python** packages: `pandas`, `matplotlib`
## How to Run

 1. **Build the Project with CMake:**

 - Open a terminal and navigate to the project directory.
  Create a build directory:
  ```bash
  mkdir build
  cd build
  ```

- Run CMake to configure the project:
  ```bash
  cmake ..
  ```

- Build the project:
  ```bash
  cmake --build .
  ```

2. **Run the Simulation:**

After building the project, you can run the simulation with the following command:

  ```bash
  ./RobotSimulation.exe
  ```

The program will run and generate CSV files containing the simulation data in the data/ directory.

## Code Structure
- `CMakeLists.txt`: CMake build script.

- `main.cpp`: Entry point of the application. Initializes motors, force calculator, robots, and runs the main simulation.

- `motor.h`, `motor.cpp`: Defines the Motor class, which simulates the dynamics of a DC motor.

- `force_calculator.h`, `force_calculator.cpp`: Defines the ForceCalculator class, responsible for calculating centrifugal forces and torques.

- `robot.h`, `robot.cpp`: Defines the Robot class, which simulates the robot's movement in 1-DOF and 2-DOF, incorporating forces from the motors and environmental factors.

- `pid.h`, `pid.cpp`: Implements the PID controller and functions for simulating motor behavior with PID/PI control, as well as the Simulated Annealing algorithm for parameter optimization.

- `params.h`: Contains structures for defining simulation parameters (MotorParams, ForceParams, RobotParams, Parameters).

- `simulation.h`, simulation.cpp: Manages the overall simulation flow, updates the state of motors and robots, and writes results to CSV files.

## Data Output
The simulation will generate several .csv files in the D:\panepistimio\Thesis\Micro-robot\code\simulation\ directory (as specified in simulation.cpp and pid.cpp). These files contain the simulation results:

- **results_motor.csv**: Contains data related to individual motor behavior (theta, omega, forces).

- **results_robot_single.csv**: Contains data for the 1-DOF robot simulation.

- **results_robot.csv**: Contains data for the 2-DOF asynchronous robot simulation.

- **sim_pid.csv**: Contains data for the PID-controlled motor simulation.

- **sim_pd.csv**: Contains data for the PI-controlled motor simulation.

- **sim_pid_mean.csv**: Contains data for the dual motor PID control focusing on mean speed.

- **sim_pid_diff.csv**: Contains data for the differential PID control of the dual motors.

Note: The current output paths are hardcoded in simulation.cpp and pid.cpp. You may need to modify these paths to a suitable location on your system before compiling.

## Results and Plots

### Python Plotting Scripts

You can use the following Python scripts to plot the data generated by the C++ simulation:

- `motor_plot.py`: Plots motor forces (horizontal and vertical) over time.
- `robot_single_motor_plot.py`: Plots the robot's position, velocity, and acceleration using data from a single motor.
- `robot_double_plot.py`: Plots the robot's position, velocity, acceleration, and forces from a two-motor setup.
- `pid_plot`: Plots the speeds of the motor with the controllers.
- `pid_diff_plot`: Plots the difference in speed of the motors using the controllers.
  
Note: The current input paths are hardcoded in the python scripts. You may need to modify these paths to a suitable location on your system before compiling.

#### Example Usage

**motor_plot.py**:
   - This script plots motor forces over time. Run it with:
   ```bash
   python motor_plot.py
   ```



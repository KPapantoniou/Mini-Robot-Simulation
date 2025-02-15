import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\results_robot_single.csv")

plt.figure(figsize=(10, 5))

plt.subplot(3, 1, 1)
plt.plot(data["Theta_left"],data["Pos_x"], label="Position X (m)", color='blue')
plt.xlabel("theta (dehg)")
plt.ylabel("Position (m)")
plt.title("Position X Over Angle")
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(data["Theta_left"],data["Vel_x"], label="Velocity X (m/s)", color='red')
plt.xlabel("Angle (deg)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity X Over Angle")
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(data["Theta_left"],data["Acc_x"], label="Acceleration X (m/s^2)", color='green')
plt.xlabel("Time (ms)")
plt.ylabel("Acceleration (m/s^2)")
plt.title("Acceleration Over Angle")
plt.grid()

plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(data["Time"],data["Force_x"], label="Force X (N)", color='orange')
plt.plot(data["Time"],data["Total_Force_Z"], label="Force Y (N)", color='red',linestyle='dashed')
plt.xlabel("Time (ms)")
plt.ylabel("Force (N)")
plt.title("Force X Over Time")
plt.legend()
plt.grid()


plt.subplot(2, 1, 2)
plt.plot(data["Time"],data["Coulomb_Force"], label="Culomb Force (N)", color='blue')
plt.plot(data["Time"],data["Friction_x"], label="Friction Force (N)", color='red',linestyle='dashed')
plt.xlabel("Time (ms)")
plt.ylabel("Force (N)")
plt.title("Coulomb Force Over Time")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
# Plotting position, velocity, and acceleration over theta (angular position of motors)
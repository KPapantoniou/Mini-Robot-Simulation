import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\results_motor.csv")


plt.figure(figsize=(10, 5))

# Angular position over time (in radians)
plt.subplot(3, 2, 1)
plt.plot(data["Time"], data["Theta_right"], label="Theta Right (Angular Position in rad)", color='blue')
plt.plot(data["Time"], data["Theta_left"], label="Theta Left (Angular Position in rad)", color='red')
plt.xlabel("Time (ms)")
plt.ylabel("Theta (rad)")
plt.title("Angular Position Over Time")
plt.grid()

# Angular velocity over time (in rad/sec)
plt.subplot(3, 2, 2)
plt.plot(data["Time"], data["Omega_right"], label="Omega Right (Angular Velocity)", color='orange')
plt.plot(data["Time"], data["Omega_left"], label="Omega Left (Angular Velocity)", color='green')
plt.xlabel("Time (ms)")
plt.ylabel("Omega (rad/sec)")
plt.title("Angular Velocity Over Time")
plt.grid()

# Forces on motors (in Newtons)
plt.subplot(3, 2, 3)
plt.plot(data["Time"], data["Force_y_right"], label="fy Right Motor (N)", color='green')
plt.plot(data["Time"], data["Force_y_left"], label="fy Left Motor (N)", color='red')
plt.xlabel("Time (ms)")
plt.ylabel("fy (N)")
plt.title("Y-Axis Centrifugal Force Over Time")
plt.grid()

plt.subplot(3, 2, 4)
plt.plot(data["Time"], data["Force_z_right"], label="fz Right Motor (N)", color='grey')
plt.plot(data["Time"], data["Force_z_left"], label="fz Left Motor (N)", color='blue')
plt.xlabel("Time (ms)")
plt.ylabel("fz (N)")
plt.title("Z-Axis Centrifugal Force Over Time")
plt.grid()
plt.tight_layout()
plt.show()
# Plotting position, velocity, and acceleration over theta (angular position of motors)



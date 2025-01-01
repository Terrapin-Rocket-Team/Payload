import numpy as np
import matplotlib.pyplot as plt

# Constants
x = -200  # PWM value
theta0 = 0.1  # initial angle (radians)
omega0 = 0.0  # initial angular velocity (rad/s)

# Time array
t = np.linspace(0, 0.5, 100)  # simulate for 0.5ms

# Calculate angular acceleration based on PWM value
if x < 0:
    alpha = 10.8 + 0.986 * x + 0.0454 * x**2 - 3.06e-05 * x**3
else:
    alpha = 0.0378 - 2.33 * x - 0.0159 * x**2 - 2.77e-04 * x**3
print("angular acceleration: ",alpha)
# Calculate angular velocity and angle over time
omega = omega0 + alpha * t
theta = theta0 + omega0 * t + 0.5 * alpha * t**2

# Plotting Angular Velocity vs. Time
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(t, omega, label='Angular Velocity (rad/s)', color='b')
plt.xlabel('Time (ms)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity vs. Time')
plt.legend()
plt.grid(True)

# Plotting Angle vs. Time
plt.subplot(2, 1, 2)
plt.plot(t, theta, label='Angle (rad)', color='r')
plt.xlabel('Time (ms)')
plt.ylabel('Angle (rad)')
plt.title('Angle vs. Time')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
import numpy as np
import matplotlib.pyplot as plt
import math

class Car:
    def __init__(self, mass, drag_coefficient, engine_force, brake_force):
        self.mass = mass
        self.drag_coefficient = drag_coefficient
        self.engine_force = engine_force
        self.brake_force = brake_force
        self.velocity = 0
        self.slope_angle = 0

    def update(self, throttle, slope, dt):
        # Calculate forces
        if throttle >= 0:
            engine_force = self.engine_force * (throttle / 10)
            brake_force = 0
        else:
            engine_force = 0
            brake_force = self.brake_force * abs(throttle / 10)
        
        drag_force = self.drag_coefficient * self.velocity**2
        gravity_force = self.mass * 9.81 * np.sin(slope)

        # Net force
        net_force = engine_force - drag_force - gravity_force - brake_force

        # Acceleration
        acceleration = net_force / self.mass

        # Update velocity and slope angle
        self.velocity += acceleration * dt
        if self.velocity < 0:
            self.velocity = 0
        self.slope_angle = slope

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0
        self.first_run = True

    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        if self.first_run:
            derivative = 0
            self.first_run = False
        else:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

def simulate(car, pid, setpoint, terrain, dt, time, use_pid):
    times = np.arange(0, time, dt)
    velocities = []
    heights = []
    throttle_values = []
    height = 0
    
    for t in times:
        slope = terrain
        if use_pid:
            throttle = pid.update(setpoint, car.velocity, dt)
            throttle = np.clip(throttle, -10, 10)
        else:
            throttle = 1  # Full throttle for testing purposes

        car.update(throttle, slope, dt)
        velocities.append(car.velocity)
        height += car.velocity * np.sin(slope) * dt
        heights.append(height)
        throttle_values.append(throttle)
    return times, velocities, heights, throttle_values
    
    
# Define a terrain profile function (example: a constant uphill slope)
def terrain_profile(degree):
    # if 2000 < t < 3000:
    #     return 0.2  # Steeper slope for a longer hill
    # return 0.1 * np.sin(0.01 * t)  # Wavy slope with small amplitude and frequency
    # degree = 10
    return degree * (math.pi / 180) # 10 degrees slope

# Parameters
mass = 1500  # kg
drag_coefficient = 0.3  # dimensionless
engine_force = 4000  # N
brake_force = 8000  # N
kp, ki, kd = 0.1, 0.01, 0.05  # PID coefficients
setpoint = 20  # m/s
dt = 0.1  # time step in seconds
time = 10000  # total simulation time in seconds

# Create car and PID controller instances
car = Car(mass, drag_coefficient, engine_force, brake_force)
pid = PIDController(kp, ki, kd)

# Run simulation
times, velocities, heights, throttle_values = simulate(car, pid, setpoint, terrain_profile(0), dt, time, use_pid=True)

# Plot results
plt.figure(figsize=(12, 6))
plt.subplot(3, 1, 1)
plt.plot(times, velocities, label='Velocity')
plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(times, heights, label='Height')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(times, throttle_values, label='Throttle')
plt.xlabel('Time (s)')
plt.ylabel('Throttle')
plt.legend()

plt.tight_layout()
plt.show()
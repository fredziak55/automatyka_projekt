import numpy as np
import matplotlib.pyplot as plt
import math
from bokeh.plotting import figure, curdoc
from bokeh.io import output_notebook, show
from bokeh.layouts import column, row
from bokeh.models import Slider, ColumnDataSource, Button
from bokeh.models.glyphs import Line
from bokeh.models import Range1d
from bokeh.models import Div
from bokeh.models.widgets import RadioButtonGroup
from bokeh.layouts import column, row
from bokeh.plotting import curdoc
from bokeh.models import HoverTool

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
        self.ti = 0 #ti 
        self.previous_error = 0
        self.first_run = True

    def update(self, setpoint, measured_value, dt): #dt = delta time
        error = setpoint - measured_value
        self.ti += error * dt
        if self.first_run:
            td = 0
            self.first_run = False
        else:
            td = (error - self.previous_error) / dt #td
        self.previous_error = error
        return self.kp * error + self.ki * self.ti + self.kd * td

def simulate(car, pid, setpoint, terrain, dt, time, use_pid):
    times = np.arange(0, time, dt)
    velocities = []
    heights = []
    throttle_values = []
    height = 0

    for t in times:
        slope = terrain(t)
        if use_pid:
            throttle = pid.update(setpoint, car.velocity, dt)
            throttle = np.clip(throttle, -10, 10)
        else:
            throttle = 1  # Full throttle for testing purposes

        car.update(throttle, slope, dt)
        velocities.append(car.velocity)  # Convert m/s to km/h
        height +=  car.velocity * np.sin(slope) * dt
        heights.append(height)
        throttle_values.append(throttle)
    return times, velocities, heights, throttle_values
    
    
# Define a terrain profile function
def terrain_profile(t, degree):
    return degree * (math.pi / 180) # Convert degrees to radians

# Parameters
mass = 1500  # kg
drag_coefficient = 0.3  # dimensionless
engine_force = 8000  # N
brake_force = 8000  # N
kp, ki, kd = 0.3, 0.01, 0.05  # PID coefficients
setpoint = 0  # m/s
dt = 0.1  # time step in seconds
simulation_time = 1000  # total simulation time in seconds

def generate_random_hills(num_hills, max_height, max_slope, total_time):
    hills = []
    for _ in range(num_hills):
        start_time = np.random.uniform(0, total_time)
        duration = np.random.uniform(100, total_time)  # Duration of each hill
        height = np.random.uniform(-max_height, max_height)  # Random height for uphill or downhill
        slope = np.random.uniform(-max_slope, max_slope)  # Random slope for uphill or downhill
        hills.append((start_time, duration, height, slope))
    return hills

def random_terrain_profile(t, hills):
    for start_time, duration, height, slope in hills:
        if start_time <= t < start_time + duration:
            return slope
    return 0 

#bokeh plots
p1 = figure(title="Car Velocity Over Time", x_axis_label='Time (s)', y_axis_label='Velocity (km/h)')
p2 = figure(title="Car position over time", x_axis_label='Time (s)')
p2.yaxis.visible = False
p3 = figure(title="PID controller", x_axis_label='Time (s)', y_axis_label='Throttle')

# Create HoverTool instances for each plot
hover_tool_p1 = HoverTool(
    tooltips=[
        ("Time", "@x{0.0}"),
        ("Velocity", "@y{0.0}")
    ]
)

hover_tool_p2 = HoverTool(
    tooltips=[
        ("Time", "@x{0.0}"),
        ("Car positon", "@y{0.0}")
    ]
)

hover_tool_p3 = HoverTool(
    tooltips=[
        ("Time", "@x{0.0}"),
        ("Throttle", "@y{0.0}")
    ]
)


# Add the HoverTool to each plot
p1.add_tools(hover_tool_p1)
p2.add_tools(hover_tool_p2)
p3.add_tools(hover_tool_p3)


# Kp_slider = Slider(start=0.01, end=1.0, value=kp, step=0.01, title="Kp")
Ki_slider = Slider(start=0.01, end=1.0, value=ki, step=0.01, title="Ki")
# Kd_slider = Slider(start=0.01, end=1.0, value=kd, step=0.01, title="Kd")
setpoint_slider = Slider(start=10, end=210, value=setpoint, step=1, title="Setpoint")
apply_button = Button(label="Apply Changes", button_type="success")
terrian_slope = Slider(start=-60, end=60, value=0, step=15, title="Terrain Slope")
random_hill_button = Button(label="Generate Random Hill", button_type="warning")

radio_button_group = RadioButtonGroup(labels=["Porsche 911 992.2 Carrera S", "Mercedes GLS", "Scania Ciężarówka"], active=0)
def radio_button_handler(attr, old, new):
    print(f"Radio button option selected: {radio_button_group.labels[new]}")

radio_button_group.on_change('active', radio_button_handler)

def update():
    ki = Ki_slider.value
    setpoint = setpoint_slider.value 
    degree = terrian_slope.value
    selected_option = radio_button_group.labels[radio_button_group.active]
    if selected_option == "Porsche 911 992.2 Carrera S":
        mass = 1500
        drag_coefficient = 0.3
        engine_force = 16000
        brake_force = 16000
    elif selected_option == "Mercedes GLS":
        mass = 2500
        drag_coefficient = 0.35
        engine_force = 10000
        brake_force = 10000
    elif selected_option == "Scania Ciężarówka":
        mass = 10000
        drag_coefficient = 0.7
        engine_force = 60000
        brake_force = 30000

    car = Car(mass, drag_coefficient, engine_force, brake_force)
    pid = PIDController(kp, ki, kd)
    times, velocities, heights, throttle_values = simulate(car, pid, setpoint / 3.6, lambda t: terrain_profile(t, degree), dt, simulation_time, use_pid=True)
    velocities = [v * 3.6 for v in velocities]
    
    plots = [p1, p2, p3]
    for plot in plots:
        for r in plot.renderers:
            if isinstance(r.glyph, Line):
                r.glyph.line_color = 'gray'



    p1.line(times, velocities, legend_label="Velocity", line_width=2, line_color='red')
    p1.line([times[0], times[-1]], [setpoint, setpoint], color='red', line_dash='dashed', legend_label="Setpoint")
    p2.line(times, heights, legend_label="Car position", line_width=2, line_color='red')
    p3.line(times, throttle_values, line_width=2, line_color='red')

def update_with_random_hill():
    ki = Ki_slider.value
    setpoint = setpoint_slider.value 
    selected_option = radio_button_group.labels[radio_button_group.active]
    if selected_option == "Porsche 911 992.2 Carrera S":
        mass = 1500
        drag_coefficient = 0.3
        engine_force = 16000
        brake_force = 16000
    elif selected_option == "Mercedes GLS":
        mass = 2500
        drag_coefficient = 0.35
        engine_force = 10000
        brake_force = 10000
    elif selected_option == "Scania Ciężarówka":
        mass = 10000
        drag_coefficient = 0.7
        engine_force = 60000
        brake_force = 30000

    car = Car(mass, drag_coefficient, engine_force, brake_force)
    pid = PIDController(kp, ki, kd)

    num_hills = 10
    max_height = 0.3  # Maximum height of hills in radians
    max_slope = 0.2  # Maximum slope of hills in radians

    hills = generate_random_hills(num_hills, max_height, max_slope, simulation_time)

    car = Car(mass, drag_coefficient, engine_force, brake_force)
    pid = PIDController(kp, ki, kd)

    times, velocities, heights, throttle_values = simulate(car, pid, setpoint / 3.6, lambda t: random_terrain_profile(t, hills), dt, simulation_time, use_pid=True)
    velocities = [v * 3.6 for v in velocities]

    plots = [p1, p2, p3]
    for plot in plots:
        for r in plot.renderers:
            if isinstance(r.glyph, Line):
                r.glyph.line_color = 'gray'

    p1.line(times, velocities, legend_label="Velocity", line_width=2)
    p1.line([times[0], times[-1]], [setpoint, setpoint], color='red', line_dash='dashed', legend_label="Setpoint")
    p2.line(times, heights, legend_label="Car position", line_width=2)
    p3.line(times, throttle_values, line_width=2)

text = Div(text="<h2>Czas próbkowania = 0.1s</h2>")

# Arrange plots in a column
layout = column(row(setpoint_slider, terrian_slope, radio_button_group, apply_button),  row(p1, p2, p3), row(Ki_slider, text),   random_hill_button)

apply_button.on_click(update)
random_hill_button.on_click(update_with_random_hill)

curdoc().add_root(layout)

output_notebook()
show(layout)
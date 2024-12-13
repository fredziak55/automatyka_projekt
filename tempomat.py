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
from bokeh.models import RadioButtonGroup, Div
from bokeh.models import ColumnDataSource, DataTable, TableColumn, Slider, Button
from bokeh.layouts import column
from bokeh.io import curdoc

class Car:
    def __init__(self, mass, drag_coefficient, engine_force, brake_force, frontal_area):
        self.mass = mass
        self.drag_coefficient = drag_coefficient
        self.engine_force = engine_force
        self.brake_force = brake_force
        self.velocity = 0
        self.slope_angle = 0
        self.air_density = 1.225  # kg/m^3
        self.frontal_area = frontal_area
        self.first_run = True


    def update(self, throttle, slope, dt):
        # Calculate forces
        if throttle >= 0:
            engine_force = self.engine_force * (throttle / 10)
            brake_force = 0
        else:
            engine_force = 0
            brake_force = self.brake_force * abs(throttle / 10)
        
        drag_force = 0.5 * self.air_density * self.drag_coefficient * self.frontal_area * self.velocity**2
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

        if self.first_run:
            self.velocity = 0
            self.first_run = False

        return net_force, engine_force, drag_force, brake_force

class PIDController:
    def __init__(self, kp, ti, td):
        self.kp = kp
        self.ti = ti
        self.td = td
        self.integral = 0 
        self.previous_error = 0
        self.first_run = True

    def update(self, setpoint, measured_value, dt): #dt = delta time
        error = setpoint - measured_value
        self.integral += error * dt
        if self.first_run:
            derivative = 0
            self.first_run = False
        else:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error
        # return self.kp * error + self.ki * self.integral + self.kd * derivative
        return self.kp * (error + (1/self.ti * self.integral) + self.td * derivative )

def simulate(car, pid, setpoint, terrain, dt, time, use_pid):
    times = np.arange(0, time, dt)
    velocities = []
    heights = []
    throttle_values = []
    net_force_values = []
    brake_force_values = []
    engine_force_values = []
    drag_force_values = []
    height = 0

    for t in times:
        slope = terrain(t)
        if use_pid:
            throttle = pid.update(setpoint, car.velocity, dt)
            throttle = np.clip(throttle, -10, 10)
        else:
            throttle = 1  # Full throttle for testing purposes
        net_force, engine_force, drag_force, brake_force = car.update(throttle, slope, dt)
        # car.update(throttle, slope, dt)
        velocities.append(car.velocity)  # Convert m/s to km/h
        height +=  car.velocity * np.sin(slope) * dt
        heights.append(height)
        throttle_values.append(throttle)
        net_force_values.append(net_force)
        brake_force_values.append(brake_force)
        engine_force_values.append(engine_force)
        drag_force_values.append(drag_force)

    return times, velocities, heights, throttle_values, net_force_values, engine_force_values, drag_force_values, brake_force_values 
    
    
# Define a terrain profile function
def terrain_profile(t, degree):
    return degree * (math.pi / 180) # Convert degrees to radians

# Parameters
mass = 1500  # kg
drag_coefficient = 0.3  # dimensionless
engine_force = 8000  # N
brake_force = 8000  # N
kp, ti, td = 0.3, 30, 0.166  # PID coefficients
setpoint = 0  # m/s
dt = 0.1  # time step in seconds
simulation_time = 1000  # total simulation time in seconds
frontal_area = 2.2  # m^2

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
p1 = figure(title="Car Velocity", x_axis_label='Time (s)', y_axis_label='Velocity (km/h)', sizing_mode='stretch_width')
p2 = figure(title="Car position", x_axis_label='Time (s)')
p2.yaxis.visible = False
p3 = figure(title="Control Signal", x_axis_label='Time (s)', y_axis_label='Control signal')
p4 = figure(title="Net force", x_axis_label='Time (s)', y_axis_label='Net force (N)')
p5 = figure(title="Slope Angle", y_range=Range1d(start=-500, end=500), x_range=Range1d(start=0, end=1000))
p5.xaxis.visible = False
p5.yaxis.visible = False
forces = figure(title="Forces", x_axis_label='Time (s)', y_axis_label='Force (N)', sizing_mode='stretch_width')

# Create HoverTool instances for each plot
hover_tool_p1 = HoverTool(tooltips=[("Time", "@x{0.0}"), ("Velocity", "@y{0.0}")])
hover_tool_p2 = HoverTool(tooltips=[("Time", "@x{0.0}"), ("Height", "@y{0.0}")])
hover_tool_p3 = HoverTool(tooltips=[("Time", "@x{0.0}"), ("Throttle", "@y{0.0}")])
hover_tool_p4 = HoverTool(tooltips=[("Time", "@x{0.0}"), ("Net Force", "@y{0.0}")])
hover_tool_p5 = HoverTool(tooltips=[("Time", "@x{0.0}"), ("Slope Angle", "@y{0.0}")])
hover_tool_forces = HoverTool(tooltips=[("Time", "@x{0.0}"), ("Force(N)", "@y{0.0}")])

# Add the HoverTool to each plot
p1.add_tools(hover_tool_p1)
p2.add_tools(hover_tool_p2)
p3.add_tools(hover_tool_p3)
p4.add_tools(hover_tool_p4)
p5.add_tools(hover_tool_p5)
forces.add_tools(hover_tool_forces)

Kp_slider = Slider(start=0.01, end=1.0, value=kp, step=0.01, title="Kp")
Ti_slider = Slider(start=0.01, end=100.0, value=ti, step=0.01, title="Td")
Td_slider = Slider(start=0.01, end=1.0, value=td, step=0.01, title="Ti")
setpoint_slider = Slider(start=0, end=210, value=setpoint, step=1, title="Setpoint")
apply_button = Button(label="Apply Changes", button_type="success")
terrian_slope = Slider(start=-45, end=45, value=0, step=15, title="Terrain Slope")
random_hill_button = Button(label="Generate Random Hill", button_type="warning")

labels = ["Porsche 911 992.2 Carrera S", "Mercedes GLS", "Scania Truck"]

radio_button_group = RadioButtonGroup(labels=labels, active=0)

def radio_button_handler(attr, old, new):
    print(f"Radio button option selected: {radio_button_group.labels[new]}")

radio_button_group.on_change('active', radio_button_handler)

def update():
    kp = Kp_slider.value
    ti = Ti_slider.value
    td = Td_slider.value
    setpoint = setpoint_slider.value 
    degree = terrian_slope.value
    selected_option = radio_button_group.labels[radio_button_group.active]
    if selected_option == "Porsche 911 992.2 Carrera S":
        mass = 1500
        drag_coefficient = 0.3
        engine_force = 25000
        brake_force = 16000
        frontal_area = 2.14
    elif selected_option == "Mercedes GLS":
        mass = 2500
        drag_coefficient = 0.35
        engine_force = 20000
        brake_force = 10000
        frontal_area = 2.5
    elif selected_option == "Scania Truck":
        mass = 10000
        drag_coefficient = 0.7
        engine_force = 60000
        brake_force = 30000
        frontal_area = 7.5

    car = Car(mass, drag_coefficient, engine_force, brake_force, frontal_area)
    pid = PIDController(kp, ti, td)
    times, velocities, heights, throttle_values, net_force_values, engine_force_values, drag_force_values, brake_force_values = simulate(car, pid, setpoint / 3.6, lambda t: terrain_profile(t, degree), dt, simulation_time, use_pid=True)
    velocities = [v * 3.6 for v in velocities]
    
    plots = [p1, p2, p3, p4, p5, forces]
    for plot in plots:
        for r in plot.renderers:
            if isinstance(r.glyph, Line):
                r.glyph.line_color = 'gray'

    p1.line(times, velocities, legend_label="Velocity", line_width=2)
    p1.line([times[0], times[-1]], [setpoint, setpoint], color='red', line_dash='dashed', legend_label="Setpoint")
    p2.line(times, heights, legend_label="Car position", line_width=2)
    p3.line(times, throttle_values, line_width=2, legend_label="Control signal")
    p4.line(times, net_force_values, legend_label="Net Force", line_width=2)
    forces.line(times, engine_force_values, legend_label="Engine Force", line_width=2, color='green')
    forces.line(times, brake_force_values, legend_label="Brake Force", line_width=2, color='red')
    forces.line(times, drag_force_values, legend_label="Drag Force", line_width=2, color='yellow')

    theta = math.radians(degree)
    m = math.tan(theta)
    y_vals = [m * x for x in times]
    p5.xaxis.axis_label = " "
    p5.yaxis.axis_label = " "
    p5.line(times, y_vals, color='blue', line_dash='dashed')

def update_with_random_hill():
    kp = Kp_slider.value
    ti = Ti_slider.value
    td = Td_slider.value
    setpoint = setpoint_slider.value 
    selected_option = radio_button_group.labels[radio_button_group.active]
    if selected_option == "Porsche 911 992.2 Carrera S":
        mass = 1500
        drag_coefficient = 0.3
        engine_force = 10000
        brake_force = 16000
        frontal_area = 2.14
    elif selected_option == "Mercedes GLS":
        mass = 2500
        drag_coefficient = 0.35
        engine_force = 20000
        brake_force = 10000
        frontal_area = 2.5
    elif selected_option == "Scania Truck":
        mass = 10000
        drag_coefficient = 0.7
        engine_force = 60000
        brake_force = 30000
        frontal_area = 7.5

    car = Car(mass, drag_coefficient, engine_force, brake_force, frontal_area)
    pid = PIDController(kp, ti, td)

    num_hills = 10
    max_height = 0.3  # Maximum height of hills in radians
    max_slope = 0.2  # Maximum slope of hills in radians

    hills = generate_random_hills(num_hills, max_height, max_slope, simulation_time)

    times, velocities, heights, throttle_values, net_force_values, engine_force_values, drag_force_values, brake_force_values = simulate(car, pid, setpoint / 3.6, lambda t: random_terrain_profile(t, hills), dt, simulation_time, use_pid=True)
    velocities = [v * 3.6 for v in velocities]

    plots = [p1, p2, p3, p4, forces]
    for plot in plots:
        for r in plot.renderers:
            if isinstance(r.glyph, Line):
                r.glyph.line_color = 'gray'

    p1.line(times, velocities, legend_label="Velocity", line_width=2)
    p1.line([times[0], times[-1]], [setpoint, setpoint], color='red', line_dash='dashed', legend_label="Setpoint")
    p2.line(times, heights, legend_label="Car position", line_width=2)
    p3.line(times, throttle_values, line_width=2, legend_label="Control Signal")
    p4.line(times, net_force_values, legend_label="Net Force", line_width=2)
    p5.renderers.clear()
    forces.line(times, engine_force_values, legend_label="Engine Force", line_width=2, color='green')
    forces.line(times, brake_force_values, legend_label="Brake Force", line_width=2, color='red')
    forces.line(times, drag_force_values, legend_label="Drag Force", line_width=2, color='blue')

text = Div(text="<h2>Sampling time (dt) = 0.1s</h2>")

car_data = [
    {"Vehicle": "Porsche 911 992.2 Carrera S", "Engine Power (N)": 25000, "Brake Force (N)": 16000, "Drag Coefficient": 0.3, "Frontal Area": 2.14, "Mass": 1500},
    {"Vehicle": "Mercedes GLS", "Engine Power (N)": 10000, "Brake Force (N)": 10000, "Drag Coefficient": 0.35, "Frontal Area": 2.5, "Mass": 2500},
    {"Vehicle": "Scania Truck", "Engine Power (N)": 60000, "Brake Force (N)": 30000, "Drag Coefficient": 0.7, "Frontal Area": 7.5, "Mass": 10000},
]

# Create an HTML table as a string
table_html = """
<table border="1" cellpadding="5" cellspacing="0" style="text-align: center">
  <tr>
    <th>Vehicle</th>
    <th>Engine Power (N)</th>
    <th>Brake Force (N)</th>
    <th>Drag Coefficient</th>
    <th>Frontal area (m^2)</th>
    <th>Mass (kg)</th>
  </tr>
"""

for car in car_data:
    table_html += f"""
  <tr>
    <td>{car['Vehicle']}</td>
    <td>{car['Engine Power (N)']}</td>
    <td>{car['Brake Force (N)']}</td>
    <td>{car['Drag Coefficient']}</td>
    <td>{car['Frontal Area']}</td>
    <td>{car['Mass']}</td>
  </tr>
"""

table_html += "</table>"

# Create a Div to display the table
table_div = Div(text=table_html)


# Arrange plots in a column
# layout = column(row(setpoint_slider, terrian_slope, radio_button_group, table_div, apply_button),  row(p1, p2), row(p3, p4), row(Kp_slider, text),   random_hill_button)
layout = column(row(p2, column(table_div, radio_button_group, setpoint_slider, terrian_slope, Kp_slider, Ti_slider, Td_slider, text, apply_button, random_hill_button), p3), row(p1, forces, sizing_mode='stretch_width'))

apply_button.on_click(update)
random_hill_button.on_click(update_with_random_hill)

curdoc().add_root(layout)

output_notebook()
show(layout)
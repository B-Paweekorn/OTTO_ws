import yaml
import matplotlib.pyplot as plt

# Load the data from the YAML file
with open('notfull.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Extract the data for plotting
times = [entry['time'] for entry in data]
x_s_values = [entry['x_s'] for entry in data]
theta_values = [entry['theta'] for entry in data]
dtheta_values = [entry['dtheta'] for entry in data]

# Normalize the time to start from zero
initial_time = times[0]
times = [t - initial_time for t in times]

# Create the plots
plt.figure(figsize=(10, 14))

# Plot x_s over time
plt.subplot(3, 1, 1)
plt.plot(times, x_s_values, label='x_s', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('x_s (m)')
plt.title('Position')

# Plot theta over time
plt.subplot(3, 1, 2)
plt.plot(times, theta_values, label='theta', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Theta (rad)')
plt.title('Pitch angle')

# Plot dtheta over time
plt.subplot(3, 1, 3)
plt.plot(times, dtheta_values, label='dtheta', color='red')
plt.xlabel('Time (s)')
plt.ylabel('dtheta (rad/s)')
plt.title('Pitch Rate')

# Adjust the layout and show the plot
plt.tight_layout()
plt.show()

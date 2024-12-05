import yaml
import matplotlib.pyplot as plt

# Load the data from the YAML file
with open('full.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Extract the data for plotting
times = [entry['time'] for entry in data]
x_s_values = [entry['x_s'] for entry in data]
theta_values = [entry['theta'] for entry in data]
dtheta_values = [entry['dtheta'] for entry in data]

# Define custom x-axis ticks with higher granularity
num_ticks = 20  # Specify the number of x ticks
x_ticks = list(range(0, len(times), max(1, len(times) // num_ticks)))

# Create the plots
plt.figure(figsize=(10, 6))

# Plot x_s over time
plt.subplot(3, 1, 1)
plt.plot(times, x_s_values, label='x_s', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('x_s (m)')
plt.title('Position')
# plt.grid(True)
plt.xticks([times[i] for i in x_ticks], rotation=45)  # Set custom x ticks

# Plot theta over time
plt.subplot(3, 1, 2)
plt.plot(times, theta_values, label='theta', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Theta (rad)')
plt.title('Pitch')
# plt.grid(True)
plt.xticks([times[i] for i in x_ticks], rotation=45)  # Set custom x ticks

# # Plot dtheta over time
# plt.subplot(3, 1, 3)
# plt.plot(times, dtheta_values, label='dtheta', color='red')
# plt.xlabel('Time (s)')
# plt.ylabel('dtheta (rad/s)')
# plt.title('Pitch Rate')
# # plt.grid(True)
# plt.xticks([times[i] for i in x_ticks], rotation=45)  # Set custom x ticks

# Adjust the layout and show the plot
plt.tight_layout()
plt.show()

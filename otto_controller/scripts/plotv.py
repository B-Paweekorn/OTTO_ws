import yaml
import matplotlib.pyplot as plt
import numpy as np

# Load the data from the YAML file
with open('vwz.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Extract the data for plotting (limit to the first 20,000 entries)
times = [entry['time'] for entry in data ]
v_xs_values = [entry['v_xs'] for entry in data ]
v_xd_values = [entry['v_xd'] for entry in data ]
w_values = [entry['w'] for entry in data ]
wd_values = [entry['wd'] for entry in data ]

# Normalize the time to start from zero
initial_time = times[0]
times = [t - initial_time for t in times]

# Define moving average window size
window_size = 20  # Example window size for moving average

# Calculate moving average for w_values
w_moving_avg = np.convolve(w_values, np.ones(window_size) / window_size, mode='valid')

# Create the figure
plt.figure(figsize=(12, 12))

# Plot v_xs and v_xd on the first subplot
plt.subplot(2, 1, 1)
plt.plot(times, v_xs_values, label='vx', color='blue', linestyle='-')
plt.plot(times, v_xd_values, label='desired vx', color='black', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend(loc='best')

# Plot w and moving average of w on the second subplot
plt.subplot(2, 1, 2)
plt.plot(times, w_values, label='w', color='blue', linestyle='-', alpha=0.6)
plt.plot(times, wd_values, label='desired w', color='black', linestyle='--', alpha=1)
plt.plot(times[window_size-1:], w_moving_avg, label='Moving Average of w', color='red', linestyle='--', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend(loc='best')

# Adjust layout and show the plots
plt.tight_layout()
plt.show()

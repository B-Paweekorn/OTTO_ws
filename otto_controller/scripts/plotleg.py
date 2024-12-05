import yaml
import matplotlib.pyplot as plt

# Load data from the YAML file
with open('leg.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Extract data for plotting
time = [entry['time'] for entry in data]
targx = [entry['targx'] for entry in data]
targy = [entry['targy'] for entry in data]
pos0_left = [entry['poslx'] for entry in data]
pos0_right = [entry['posrx'] for entry in data]

pos1_left = [entry['posly'] for entry in data]
pos1_right = [entry['posry'] for entry in data]

x = [entry['x'] for entry in data]
theta = [entry['theta'] for entry in data]
dtheta = [entry['dtheta'] for entry in data]

# Create subplots
plt.figure(figsize=(12, 16))

# Subplot 1: targ[0] vs pos[0] for left and right
plt.subplot(5, 1, 1)
plt.plot(time, targx, label='legpose_x_ref', linewidth=1, linestyle='--', color='blue')
plt.plot(time, pos0_left, label='legpose_x_left', linewidth=0.8, color='blue')
plt.plot(time, pos0_right, label='legpose_x_right',linewidth=0.8, color='green')
plt.xlabel('Time (s)')
plt.ylabel('legx (m)')
plt.title('Leg position (+x)')
plt.legend(loc='best')
# plt.grid(True)

# Subplot 2: targ[1] vs pos[1] for left and right
plt.subplot(5, 1, 2)
plt.plot(time, targy, label='legpose_y_ref', linestyle='--', linewidth=1, color='blue')
plt.plot(time, pos1_left, label='legpose_y_left', linewidth=0.8, color='blue')
plt.plot(time, pos1_right, label='legpose_y_right', linewidth=0.8, color='green')
plt.xlabel('Time (s)')
plt.ylabel('legy (m)')
plt.title('Leg position (+z)')
plt.legend(loc='best')
# plt.grid(True)

plt.subplot(5, 1, 3)
plt.plot(time, x, label='robot_pose', linewidth=0.8, linestyle='-', color='black')
plt.axhline(y=0, color='blue', linestyle='--', linewidth=1, label='reference')
plt.xlabel('Time (s)')
plt.ylabel('x (m)')
plt.title('Robot position (+x)')
plt.legend(loc='best')

plt.subplot(5, 1, 4)
plt.plot(time, theta, label='pitch_angle', linewidth=0.8, linestyle='-', color='black')
plt.axhline(y=0, color='blue', linestyle='--', linewidth=1, label='reference')
plt.xlabel('Time (s)')
plt.ylabel('theta (rad)')
plt.title('Pitch')
plt.legend(loc='best')

plt.subplot(5, 1, 5)
plt.plot(time, dtheta, label='pitch_rate', linewidth=0.8, linestyle='-', color='black')
plt.axhline(y=0, color='blue', linestyle='--', linewidth=1, label='reference')
plt.xlabel('Time (s)')
plt.ylabel('dtheta (rad/s)')
plt.title('Pitch rate')
plt.legend(loc='best')

# Adjust layout and display the plots
plt.tight_layout()
plt.show()

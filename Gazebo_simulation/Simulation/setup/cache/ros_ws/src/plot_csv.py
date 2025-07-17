import csv
import matplotlib.pyplot as plt

CSV_FILENAME = "./src/ros_ws/src/drone_positions.csv"
TARGETS_FILE = "./src/ros_ws/src/target_positions.csv"
PLOT_3D = True

# Read drone data from the file
time_data, x_data, y_data, z_data = [], [], [], []
x_ref, y_ref, z_ref = [], [], []
no_drones = 0
padding = 1.5
figsize = (12, 6)

with open(CSV_FILENAME, mode='r') as file:
    reader = csv.reader(file)
    next(reader)  # skip the header
    for row in reader:
        i_drone = int(row[1])
        if i_drone > no_drones:
            no_drones = i_drone
        while len(time_data) < no_drones:  # Bugfix when first CSV line is not drone 1
            time_data.append([])
            x_data.append([]), x_ref.append([])
            y_data.append([]), y_ref.append([])
            z_data.append([]), z_ref.append([])
        time_data[i_drone - 1].append(float(row[0]))
        x_data[i_drone - 1].append(float(row[2]))
        y_data[i_drone - 1].append(float(row[3]))
        z_data[i_drone - 1].append(float(row[4]))
        
with open(TARGETS_FILE, mode='r') as file:
    reader = csv.reader(file)
    next(reader)  # skip the header
    last_time_target = {}  

    for row in reader:
        i_drone = int(row[0])  
        time_target = int(row[1]) 
        target_x, target_y, target_z = float(row[2]), float(row[3]), float(row[4])  

        if i_drone not in last_time_target:
            last_time_target[i_drone] = -1  

        if time_target > last_time_target[i_drone]:
            for t_idx, time_point in enumerate(time_data[i_drone - 1]):
                if time_point < time_target and time_point >= last_time_target[i_drone]:
                    x_ref[i_drone - 1].append(target_x)
                    y_ref[i_drone - 1].append(target_y)
                    z_ref[i_drone - 1].append(target_z)

            last_time_target[i_drone] = time_target

# 3D Plotting
if PLOT_3D:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    colors = ['r', 'g', 'b', 'y', 'm']  # Add more colors if needed

    for i in range(no_drones):
        ax.scatter(x_data[i], y_data[i], z_data[i], color=colors[i % len(colors)], marker='o', label=f'd {i+1}')
        # ax.plot(x_ref[i], y_ref[i], z_ref[i], color='b', linestyle='--', label=f'd {i+1} ref', alpha=0.5)

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('Drones Positions')
    ax.legend()

# 2D Time-Series Plots
for i in range(no_drones):
    fig, axs = plt.subplots(3, 1, figsize=figsize, sharex=True)
    fig.suptitle(f'Drone {i+1}')

    axs[0].plot(time_data[i], x_data[i], label='sim', color='k')
    axs[1].plot(time_data[i], y_data[i], label='sim', color='g')
    axs[2].plot(time_data[i], z_data[i], label='sim', color='b')
    
    # axs[0].plot(time_data[i], x_ref[i], 'r--', label=f'ref')  
    # axs[1].plot(time_data[i], y_ref[i], 'r--', label=f'ref')  
    # axs[2].plot(time_data[i], z_ref[i], 'r--', label=f'ref')

    # axs[0].set_title(f'Drone {i+1} X')
    # axs[1].set_title(f'Drone {i+1} Y')
    # axs[2].set_title(f'Drone {i+1} Z')
    
    y_labels = ['Position X', 'Position Y', 'Position Z']

    for i, ax in enumerate(axs):
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(y_labels[i])  
        ax.legend()
        ax.grid(True)
        # ax.set_ylim(-0.1, 1.1)  
        

plt.tight_layout(pad=padding)
plt.show()

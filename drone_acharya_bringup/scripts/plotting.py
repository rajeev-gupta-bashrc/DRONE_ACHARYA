import numpy as np
import matplotlib.pyplot as plt

def read_data_file(filename):
    # Lists to store each component
    x_vals, y_vals, z_vals = [], [], []
    roll_vals, pitch_vals, yaw_vals = [], [], []
    thrust1_vals, thrust2_vals, thrust3_vals, thrust4_vals = [], [], [], []
    
    with open(filename, 'r') as file:
        for line in file:
            # Remove 'INFO:root:' prefix and split the string
            clean_line = line.replace('INFO:root:', '').strip()
            values = [float(val.strip(',')) for val in clean_line.split()]
            
            # Append values to respective lists
            x_vals.append(values[0])
            y_vals.append(values[1])
            z_vals.append(values[2])
            roll_vals.append(values[3])
            pitch_vals.append(values[4])
            yaw_vals.append(values[5])
            thrust1_vals.append(values[6])
            thrust2_vals.append(values[7])
            thrust3_vals.append(values[8])
            thrust4_vals.append(values[9])
    
    return (x_vals, y_vals, z_vals, roll_vals, pitch_vals, yaw_vals, 
            thrust1_vals, thrust2_vals, thrust3_vals, thrust4_vals)

def plot_data(data):
    time = np.arange(len(data[0]))  # Create time array based on data length
    
    # Position plots
    plt.figure(1)
    plt.title('Position Data')
    plt.plot(time, data[0], label='X')
    plt.plot(time, data[1], label='Y')
    plt.plot(time, data[2], label='Z')
    plt.xlabel('Time Steps')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True)
    
    # Orientation plots
    plt.figure(2)
    plt.title('Orientation Data')
    plt.plot(time, data[3], label='Roll')
    plt.plot(time, data[4], label='Pitch')
    plt.plot(time, data[5], label='Yaw')
    plt.xlabel('Time Steps')
    plt.ylabel('Angle (rad)')
    plt.legend()
    plt.grid(True)
    
    # Thrust plots
    plt.figure(3)
    plt.title('Motor Thrust Data')
    plt.plot(time, data[6], label='Motor 1')
    plt.plot(time, data[7], label='Motor 2')
    plt.plot(time, data[8], label='Motor 3')
    plt.plot(time, data[9], label='Motor 4')
    plt.xlabel('Time Steps')
    plt.ylabel('Thrust')
    plt.legend()
    plt.grid(True)
    
    # 3D Position Plot
    fig = plt.figure(4)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(data[0], data[1], data[2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Position Trajectory')
    
    # Show all plots
    plt.show()

def main():
    # Replace 'your_file.txt' with your actual file name
    filename = '/home/rajeev-gupta/ros2/btp_ws/src/DRONE_ACHARYA/drone_acharya_bringup/logs/ros2_log.txt'
    try:
        data = read_data_file(filename)
        plot_data(data)
    except Exception as e:
        print(f"Error occurred: {e}")

if __name__ == "__main__":
    main()
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

def load_pose_data(file_path):
    """Load pose data from CSV file."""
    df = pd.read_csv(file_path)
    # Convert timestamp to seconds from start for easier plotting
    start_time = df['Timestamp (ns)'].iloc[0]
    df['Time (s)'] = (df['Timestamp (ns)'] - start_time) / 1e9
    return df

def calculate_position_error(df):
    """Calculate position error between start and end."""
    start_pos = df[['X (m)', 'Y (m)', 'Z (m)']].iloc[0].values
    end_pos = df[['X (m)', 'Y (m)', 'Z (m)']].iloc[-1].values
    
    # Calculate Euclidean distance for position error
    pos_error = np.sqrt(np.sum((end_pos - start_pos)**2))
    
    # Calculate component-wise errors
    x_error = end_pos[0] - start_pos[0]
    y_error = end_pos[1] - start_pos[1]
    z_error = end_pos[2] - start_pos[2]
    
    return {
        'total': pos_error,
        'x': x_error,
        'y': y_error,
        'z': z_error,
        'start_pos': start_pos,
        'end_pos': end_pos
    }

def calculate_heading_error(df):
    """Calculate heading error between start and end."""
    start_yaw = df['Yaw (deg)'].iloc[0]
    end_yaw = df['Yaw (deg)'].iloc[-1]
    
    # Handle the circular nature of angles
    yaw_error = (end_yaw - start_yaw) % 360
    if yaw_error > 180:
        yaw_error -= 360
        
    # Do the same for roll and pitch
    start_roll = df['Roll (deg)'].iloc[0]
    end_roll = df['Roll (deg)'].iloc[-1]
    roll_error = (end_roll - start_roll) % 360
    if roll_error > 180:
        roll_error -= 360
        
    start_pitch = df['Pitch (deg)'].iloc[0]
    end_pitch = df['Pitch (deg)'].iloc[-1]
    pitch_error = (end_pitch - start_pitch) % 360
    if pitch_error > 180:
        pitch_error -= 360
    
    return {
        'yaw': yaw_error,
        'roll': roll_error,
        'pitch': pitch_error,
        'start_orientation': [start_roll, start_pitch, start_yaw],
        'end_orientation': [end_roll, end_pitch, end_yaw]
    }

def plot_trajectory(df):
    """Create a 2D plot of the trajectory."""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(df['X (m)'], df['Y (m)'], 'b-', linewidth=1, label='Path')
    ax.scatter(df['X (m)'].iloc[0], df['Y (m)'].iloc[0], color='green', s=100, label='Start')
    ax.scatter(df['X (m)'].iloc[-1], df['Y (m)'].iloc[-1], color='red', s=100, label='End')
    
    # Draw arrows to indicate orientation at regular intervals
    step = max(1, len(df) // 20)  # Show ~20 orientation markers
    for i in range(0, len(df), step):
        x, y = df['X (m)'].iloc[i], df['Y (m)'].iloc[i]
        yaw = np.radians(df['Yaw (deg)'].iloc[i])
        dx, dy = 0.05 * np.cos(yaw), 0.05 * np.sin(yaw)
        ax.arrow(x, y, dx, dy, head_width=0.02, head_length=0.03, fc='black', ec='black')
    
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Rover Trajectory (Top View)')
    ax.legend()
    
    return fig

def plot_trajectory_3d(df):
    """Create a 3D plot of the trajectory."""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot3D(df['X (m)'], df['Y (m)'], df['Z (m)'], 'blue')
    ax.scatter(df['X (m)'].iloc[0], df['Y (m)'].iloc[0], df['Z (m)'].iloc[0], 
               color='green', s=100, label='Start')
    ax.scatter(df['X (m)'].iloc[-1], df['Y (m)'].iloc[-1], df['Z (m)'].iloc[-1], 
               color='red', s=100, label='End')
    
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('3D Rover Trajectory')
    ax.legend()
    
    return fig

def plot_position_over_time(df):
    """Plot position components over time."""
    fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    ax[0].plot(df['Time (s)'], df['X (m)'])
    ax[0].set_ylabel('X (meters)')
    ax[0].grid(True)
    
    ax[1].plot(df['Time (s)'], df['Y (m)'])
    ax[1].set_ylabel('Y (meters)')
    ax[1].grid(True)
    
    ax[2].plot(df['Time (s)'], df['Z (m)'])
    ax[2].set_ylabel('Z (meters)')
    ax[2].set_xlabel('Time (seconds)')
    ax[2].grid(True)
    
    fig.suptitle('Position Components Over Time')
    plt.tight_layout()
    
    return fig

def plot_orientation_over_time(df):
    """Plot orientation components over time."""
    fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    ax[0].plot(df['Time (s)'], df['Roll (deg)'])
    ax[0].set_ylabel('Roll (degrees)')
    ax[0].grid(True)
    
    ax[1].plot(df['Time (s)'], df['Pitch (deg)'])
    ax[1].set_ylabel('Pitch (degrees)')
    ax[1].grid(True)
    
    ax[2].plot(df['Time (s)'], df['Yaw (deg)'])
    ax[2].set_ylabel('Yaw (degrees)')
    ax[2].set_xlabel('Time (seconds)')
    ax[2].grid(True)
    
    fig.suptitle('Orientation Components Over Time')
    plt.tight_layout()
    
    return fig

def plot_position_error_over_time(df):
    """Plot the accumulated position error over time."""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    start_pos = df[['X (m)', 'Y (m)', 'Z (m)']].iloc[0].values
    errors = []
    
    for i in range(len(df)):
        current_pos = df[['X (m)', 'Y (m)', 'Z (m)']].iloc[i].values
        error = np.sqrt(np.sum((current_pos - start_pos)**2))
        errors.append(error)
    
    ax.plot(df['Time (s)'], errors)
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Distance from Start (meters)')
    ax.set_title('Position Error Relative to Starting Point')
    ax.grid(True)
    
    return fig

def main():
    file_path = r"test/data.csv"
    df = load_pose_data(file_path)
    
    # Calculate errors
    pos_error = calculate_position_error(df)
    heading_error = calculate_heading_error(df)
    
    # Print error statistics
    print("Position Error Analysis:")
    print(f"Start position: X={pos_error['start_pos'][0]:.3f}, Y={pos_error['start_pos'][1]:.3f}, Z={pos_error['start_pos'][2]:.3f}")
    print(f"End position: X={pos_error['end_pos'][0]:.3f}, Y={pos_error['end_pos'][1]:.3f}, Z={pos_error['end_pos'][2]:.3f}")
    print(f"Total position error: {pos_error['total']:.3f} meters")
    print(f"X error: {pos_error['x']:.3f} meters")
    print(f"Y error: {pos_error['y']:.3f} meters")
    print(f"Z error: {pos_error['z']:.3f} meters")
    
    print("\nHeading Error Analysis:")
    print(f"Start orientation: Roll={heading_error['start_orientation'][0]:.2f}°, "
          f"Pitch={heading_error['start_orientation'][1]:.2f}°, "
          f"Yaw={heading_error['start_orientation'][2]:.2f}°")
    print(f"End orientation: Roll={heading_error['end_orientation'][0]:.2f}°, "
          f"Pitch={heading_error['end_orientation'][1]:.2f}°, "
          f"Yaw={heading_error['end_orientation'][2]:.2f}°")
    print(f"Roll error: {heading_error['roll']:.2f}°")
    print(f"Pitch error: {heading_error['pitch']:.2f}°")
    print(f"Yaw error: {heading_error['yaw']:.2f}°")
    
    # Generate and show plots one at a time
    # fig1 = plot_trajectory(df)
    # plt.show()
    
    fig2 = plot_trajectory_3d(df)
    plt.show()
    
    fig3 = plot_position_over_time(df)
    plt.show()
    
    fig4 = plot_orientation_over_time(df)
    plt.show()
    
    fig5 = plot_position_error_over_time(df)
    plt.show()

if __name__ == "__main__":
    main()
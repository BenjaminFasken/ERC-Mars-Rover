import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.patches import Wedge

# Set plot style for better visuals
sns.set(style="whitegrid")

try:
    # Load the CSV data
    data = pd.read_csv('src/probe_detection/test/Annotated_filledOut/results.csv')

    # Select vital columns and create a copy to avoid SettingWithCopyWarning
    vital_columns = ['image_id', 'probe_id', 'error', 
                     'x_estimated', 'y_estimated', 'z_estimated', 
                     'x_gt', 'y_gt', 'z_gt']
    vital_data = data[vital_columns].copy()

    # Print probe position statistics (in meters)
    print("Probe Position Statistics (meters):")
    print(vital_data[['x_gt', 'y_gt']].describe())

    # Parameters
    max_depth = 2.2  # meters
    num_bins = 7  # Number of depth bins
    fov_angle = 100  # Field of view angle in degrees
    half_fov_rad = np.radians(fov_angle / 2)  # Half FOV in radians

    # Initialize bins
    bin_edges = np.linspace(0, max_depth, num_bins + 1)  # e.g., [0, 0.214, 0.428, ..., 1.5]
    bin_errors = [[] for _ in range(num_bins)]  # List to store errors for each bin
    bin_probe_counts = [0] * num_bins  # Count probes per bin

    # Iterate through probes to assign to bins and collect errors (only for probes in cone)
    cone_probe_count = 0
    for idx, row in vital_data.iterrows():
        x_gt = row['x_gt']
        y_gt = row['y_gt']
        error = row['error']

        # Check if probe is within cone (X ≥ 0 and |Y| ≤ X * tan(FOV/2))
        if x_gt >= 0 and abs(y_gt) <= x_gt * np.tan(half_fov_rad):
            cone_probe_count += 1
            # Determine bin based on x_gt
            for i in range(num_bins):
                if bin_edges[i] <= x_gt < bin_edges[i + 1]:
                    bin_errors[i].append(error)
                    bin_probe_counts[i] += 1
                    break

    # Calculate mean error per bin
    mean_errors = [np.mean(errors) if errors else 0 for errors in bin_errors]
    mean_errors = np.array(mean_errors)

    # Print diagnostics
    print(f"\nTotal Probes in Dataset: {len(vital_data)}")
    print(f"Probes within Cone (X ≥ 0 and |Y| ≤ X * tan(FOV/2)): {cone_probe_count}")
    print(f"Probes outside Cone: {len(vital_data) - cone_probe_count}")
    print("\nAverage 3D Error and Probe Count per Bin (within cone):")
    for i in range(num_bins):
        print(f"Bin {i} ({bin_edges[i]:.3f}-{bin_edges[i+1]:.3f} m): Mean Error = {mean_errors[i]:.4f}, Probes = {bin_probe_counts[i]}")

    # Visualization: 2D Cone with Radial Average Error
    fig, ax = plt.subplots(figsize=(10, 8))
    cmap = plt.get_cmap('plasma')  # High-contrast colormap
    # Normalize colors based on non-zero errors
    valid_errors = mean_errors[mean_errors > 0]
    vmin = valid_errors.min() if valid_errors.size > 0 else 0
    vmax = valid_errors.max() if valid_errors.size > 0 else 1
    if vmin == vmax:  # Expand range if all non-zero errors are the same
        vmin = max(0, vmin - 0.1)
        vmax = vmin + 0.2
    norm = plt.Normalize(vmin=vmin, vmax=vmax)

    # Plot radial bins as annular sectors (Wedge patches)
    for i in range(num_bins):
        r_inner = bin_edges[i]
        r_outer = bin_edges[i + 1]
        mean_error = mean_errors[i]
        probe_count = bin_probe_counts[i]

        # Color empty bins gray, otherwise use colormap
        color = 'gray' if probe_count == 0 else cmap(norm(mean_error))
        # Wedge: center at (0,0), radius from r_inner to r_outer, angles from -fov_angle/2 to +fov_angle/2
        wedge = Wedge((0, 0), r_outer, -fov_angle/2, fov_angle/2, 
                      width=r_outer - r_inner, facecolor=color, alpha=0.6, edgecolor='black')
        ax.add_patch(wedge)

    # Scatter probes: inside cone (red), outside cone (blue)
    cone_probes = vital_data[
        (vital_data['x_gt'] >= 0) & 
        (np.abs(vital_data['y_gt']) <= vital_data['x_gt'] * np.tan(half_fov_rad))
    ]
    outside_probes = vital_data[
        ~((vital_data['x_gt'] >= 0) & 
          (np.abs(vital_data['y_gt']) <= vital_data['x_gt'] * np.tan(half_fov_rad)))
    ]
    ax.scatter(cone_probes['x_gt'], cone_probes['y_gt'], c='red', s=10, label='Probes in Cone')
    ax.scatter(outside_probes['x_gt'], outside_probes['y_gt'], c='blue', s=10, marker='x', label='Probes Outside Cone')

    # Set plot limits to include all probes
    x_min = min(vital_data['x_gt'].min(), 0)
    x_max = max(vital_data['x_gt'].max(), max_depth)
    y_min = vital_data['y_gt'].min()
    y_max = vital_data['y_gt'].max()
    x_pad = (x_max - x_min) * 0.41
    y_pad = (y_max - y_min) * 0.41
    ax.set_xlim(x_min - x_pad, x_max + x_pad)
    ax.set_ylim(y_min - y_pad, y_max + y_pad)
    ax.set_aspect('equal')
    ax.set_title('Radial Average Error in 2D Cone vs Depth')
    ax.set_xlabel('Depth (X Ground Truth, m)')
    ax.set_ylabel('Lateral Position (Y Ground Truth, m)')
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    plt.colorbar(sm, ax=ax, label='Mean 3D Error (non-empty bins)')
    ax.legend()

    # Save and show plot
    plt.savefig('cone_radial_average_error.png')
    plt.show()

except FileNotFoundError:
    print("Error: 'results.csv' not found. Please ensure the file is in the correct directory.")
except Exception as e:
    print(f"An error occurred: {str(e)}")
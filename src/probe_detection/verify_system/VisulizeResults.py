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

    # Debug: Inspect probes with x_gt near zero
    print("\nProbes with x_gt near zero (x_gt < 0.01):")
    print(vital_data[vital_data['x_gt'] < 0.01][['x_gt', 'y_gt', 'error']])

    # Parameters
    max_depth = 1.8  # meters (maximum radial distance)
    num_bins = 5  # Number of depth bins
    fov_angle = 110  # Field of view angle in degrees
    half_fov_rad = np.radians(fov_angle)/2  # Half FOV in radians
    min_x_gt = 1e-6  # Small threshold to avoid numerical issues

    # Initialize bins (radial distances)
    bin_edges = np.linspace(0, max_depth, num_bins + 1)  # e.g., [0, 0.36, 0.72, ..., 1.8]
    print("\nRadial bin edges (meters):", bin_edges)
    bin_errors = [[] for _ in range(num_bins)]  # List to store errors for each bin
    bin_probe_counts = [0] * num_bins  # Count probes per bin
    bin_probes = [[] for _ in range(num_bins)]  # List to store probe details for each bin

    # Iterate through probes to assign to bins and collect errors (only for probes in cone)
    cone_probe_count = 0
    for idx, row in vital_data.iterrows():
        x_gt = row['x_gt']
        y_gt = row['y_gt']
        error = row['error']

        # Compute radial distance from origin
        r = np.sqrt(x_gt**2 + y_gt**2)

        # Check if probe is within cone (X ≥ min_x_gt and |Y| ≤ X * tan(FOV/2))
        if x_gt >= min_x_gt and abs(y_gt) <= x_gt * np.tan(half_fov_rad):
            cone_probe_count += 1
            # Determine bin based on radial distance r
            for i in range(num_bins):
                if i == num_bins - 1:
                    # Include upper bound for last bin
                    if bin_edges[i] <= r <= bin_edges[i + 1]:
                        bin_errors[i].append(error)
                        bin_probe_counts[i] += 1
                        bin_probes[i].append({
                            'index': idx,
                            'x_gt': x_gt,
                            'y_gt': y_gt,
                            'r': r,
                            'error': error
                        })
                        break
                else:
                    if bin_edges[i] <= r < bin_edges[i + 1]:
                        bin_errors[i].append(error)
                        bin_probe_counts[i] += 1
                        bin_probes[i].append({
                            'index': idx,
                            'x_gt': x_gt,
                            'y_gt': y_gt,
                            'r': r,
                            'error': error
                        })
                        break
            else:
                # Debug: Log probes that fall outside all bins
                print(f"Probe at index {idx} with r={r:.6f} not assigned to any bin.")

    # Calculate mean error per bin
    mean_errors = [np.mean(errors) if errors else 0 for errors in bin_errors]
    std_errors = [np.std(errors) if errors else 0 for errors in bin_errors]
    total_stdev = np.std([error for errors in bin_errors for error in errors])  # Overall standard deviation
    mean_errors = np.array(mean_errors)
    total_mean = np.mean(mean_errors[mean_errors > 0])  # Mean of non-zero errors

    # Print diagnostics
    print(f"\nTotal Probes in Dataset: {len(vital_data)}")
    print(f"Probes within Cone (X ≥ {min_x_gt} and |Y| ≤ X * tan(FOV/2)): {cone_probe_count}")
    print("\nAverage 3D Error and Probe Count per Bin (within cone):")
    for i in range(num_bins):
        print(f"Bin {i} ({bin_edges[i]:.3f}-{bin_edges[i+1]:.3f} m): Mean Error = {mean_errors[i]:.4f}, Probes = {bin_probe_counts[i]}")
        print(f"Bin {i} Std Error = {std_errors[i]:.4f}")
        
    print(f"\nOverall Standard Deviation of Errors: {total_stdev:.4f}")
    print(f"Mean of Non-Zero Errors: {total_mean:.4f}")

    # Visualization: 2D Cone with Radial Average Error
    fig, ax = plt.subplots(figsize=(10, 8))
    cmap = plt.get_cmap('plasma')  # Colormap for wedges
    # Normalize colors for wedges based on non-zero errors
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
        wedge = Wedge((0, 0), r_outer, 90 - fov_angle/2, 90 + fov_angle/2,
                    width=r_outer - r_inner, facecolor=color, alpha=0.6, edgecolor='black')
        ax.add_patch(wedge)

    # Scatter probes: color by bin using viridis colormap
    probe_cmap = plt.get_cmap('viridis')  # Colormap for probes

    # Plot all probes
    ax.scatter(vital_data['y_gt'], vital_data['x_gt'], c='red', s=10, alpha=0.5, label='Ground Truth Probes')

    # Add points at the outermost location of each bin (at y_gt=0, x_gt=bin_edges[i+1])
    outer_points_x = bin_edges[1:]  # Outer radius of each bin
    outer_points_y = [0] * num_bins  # Along cone’s central axis (y_gt=0)

    # Set plot limits to include all probes
    x_min = min(vital_data['y_gt'].min(), 0)
    max_width = max(abs(vital_data['y_gt'].min()), abs(vital_data['y_gt'].max()))
    x_max = max(vital_data['y_gt'].max(), max_width)
    y_min = vital_data['x_gt'].min()
    y_max = vital_data['x_gt'].max()

    x_pad = (x_max - x_min) * 0.5  # Increased padding to ensure annotation visibility
    y_pad = (y_max - y_min) * 0.41
    ax.set_xlim(x_min - x_pad, x_max + x_pad)
    ax.set_ylim(y_min - y_pad, y_max + y_pad)
    ax.set_aspect('equal')
    ax.set_title('Mean XYZ position estimate error vs XY coordinates', fontweight='bold', fontsize=16)
    ax.set_xlabel('Y Ground Truth [m]')
    ax.set_ylabel('X Ground Truth [m]')
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = plt.colorbar(sm, ax=ax, label='Mean XYZ position estimate error [m]', shrink=0.65)
    cbar.ax.yaxis.set_label_coords(5.4, 0.5)  # Move the label to the right
    ax.legend()


    # Save and show plot
    
    plt.savefig('cone_radial_average_error.png', savefig='png', dpi=5, bbox_inches='tight')
    plt.show()
    
    
except FileNotFoundError:
    print("Error: 'results.csv' not found. Please ensure the file is in the correct directory.")
except Exception as e:
    print(f"An error occurred: {str(e)}")
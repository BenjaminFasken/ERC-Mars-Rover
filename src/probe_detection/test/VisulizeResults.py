import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# Set plot style for better visuals
sns.set(style="whitegrid")

try:
    # Load the CSV data
    data = pd.read_csv('src/probe_detection/test/Annotated_filledOut/results.csv')

    # Select vital columns and create a copy to avoid SettingWithCopyWarning
    vital_columns = ['image_id', 'probe_id', 'error', 
                     'x_estimated', 'y_estimated', 'z_estimated', 'x_transformed', 'y_transformed', 'z_transformed',
                     'x_gt', 'y_gt', 'z_gt', 'centroid_x', 'centroid_y']
    vital_data = data[vital_columns].copy()

    # Display the first few rows
    print("Vital Information from results.csv:")
    print(vital_data.head())

    # Basic statistics for error
    print("\nBasic Statistics of Error:")
    print(vital_data['error'].describe())

    # Calculate error without Z coordinate
    vital_data.loc[:, 'error_no_z'] = np.sqrt((vital_data['x_estimated'] - vital_data['x_gt'])**2 +
                                              (vital_data['y_estimated'] - vital_data['y_gt'])**2)
    print("\nBasic Statistics of Error without Z Coordinate:")
    print(vital_data['error_no_z'].describe())
    print("\nError without Z Coordinate (First 5 Rows):")
    print(vital_data[['image_id', 'probe_id', 'error_no_z']].head())
    
 

    # Average error per image
    avg_error_per_image = vital_data.groupby('image_id')[['error', 'error_no_z']].mean().reset_index()
    print("\nAverage Error per Image (Error and Error without Z):")
    print(avg_error_per_image)

    # Average error per probe
    avg_error_per_probe = vital_data.groupby('probe_id')[['error', 'error_no_z']].mean().reset_index()
    print("\nAverage Error per Probe (Error and Error without Z):")
    print(avg_error_per_probe)

    # Check for missing probes
    expected_probes = set([f'probe_{i}' for i in range(1, 6)])
    actual_probes = set(vital_data['probe_id'].unique())
    missing_probes = expected_probes - actual_probes
    if missing_probes:
        print(f"\nWarning: Missing probes: {missing_probes}")

    # Save vital data
    # vital_data.to_csv('vital_results.csv', index=False)
    # print("\nVital data saved to 'vital_results.csv'")

    # Visualization 1: Histogram of Errors (with and without Z)
    plt.figure(figsize=(12, 6))
    sns.histplot(vital_data['error'], bins=20, kde=True, color='blue', label='Error (3D)', alpha=0.5)
    sns.histplot(vital_data['error_no_z'], bins=20, kde=True, color='orange', label='Error (2D)', alpha=0.5)
    plt.title('Distribution of Localization Errors (3D vs 2D)')
    plt.xlabel('Error')
    plt.ylabel('Frequency')
    plt.legend()
    #plt.savefig('error_histogram_combined.png')
    plt.show()

    # # Visualization 2: Error vs X and Y Distances
    # plt.figure(figsize=(10, 8))
    # scatter = sns.scatterplot(x=vital_data['x_estimated'] - vital_data['x_gt'],
    #                           y=vital_data['y_estimated'] - vital_data['y_gt'],
    #                           hue=vital_data['error'], size=vital_data['error'],
    #                           palette='viridis', alpha=0.6, sizes=(20, 200))
    # plt.title('Error vs X and Y Distances')
    # plt.xlabel('X Distance (Estimated - Ground Truth)')
    # plt.ylabel('Y Distance (Estimated - Ground Truth)')
    # plt.grid(False)  # Suppress Matplotlib deprecation warning
    # plt.colorbar(scatter.collections[0], label='3D Error')
    # plt.savefig('error_vs_xy_distances.png')
    # plt.show()

    # # Visualization 3: Bar Plot of Average Error per Image
    # plt.figure(figsize=(12, 6))
    # avg_error_per_image_melted = avg_error_per_image.melt(id_vars='image_id', 
    #                                                       value_vars=['error', 'error_no_z'],
    #                                                       var_name='Error Type', value_name='Average Error')
    # sns.barplot(x='image_id', y='Average Error', hue='Error Type', data=avg_error_per_image_melted)
    # plt.title('Average Localization Error per Image (3D vs 2D)')
    # plt.xlabel('Image ID')
    # plt.ylabel('Average Error')
    # plt.xticks(rotation=45)
    # plt.legend(title='Error Type', labels=['3D Error', '2D Error'])
    # plt.savefig('avg_error_per_image.png')
    # plt.show()

    # Visualization 4: Bar Plot of Average Error per Probe
    plt.figure(figsize=(8, 6))
    avg_error_per_probe_melted = avg_error_per_probe.melt(id_vars='probe_id', 
                                                          value_vars=['error', 'error_no_z'],
                                                          var_name='Error Type', value_name='Average Error')
    sns.barplot(x='probe_id', y='Average Error', hue='Error Type', data=avg_error_per_probe_melted)
    plt.title('Average Localization Error per Probe (3D vs 2D)')
    plt.xlabel('Probe ID')
    plt.ylabel('Average Error')
    plt.legend(title='Error Type', labels=['3D Error', '2D Error'])
    #plt.savefig('avg_error_per_probe.png')
    plt.show()

    # # Visualization 5: Scatter Plot of Estimated vs Ground Truth (X, Y)
    # plt.figure(figsize=(10, 8))
    # plt.scatter(vital_data['x_gt'], vital_data['y_gt'], c='blue', label='Ground Truth', alpha=0.5)
    # plt.scatter(vital_data['x_transformed'], vital_data['y_transformed'], c='red', label='Estimated', alpha=0.5)
    # plt.title('Estimated vs Ground Truth Positions (X, Y)')
    # plt.xlabel('X Coordinate')
    # plt.ylabel('Y Coordinate')
    # plt.legend()
    # plt.savefig('xy_scatter.png')
    # plt.show()

    # # Visualization 6: 3D Scatter Plot of Estimated vs Ground Truth
    # fig = plt.figure(figsize=(12, 8))
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(vital_data['x_gt'], vital_data['y_gt'], vital_data['z_gt'], c='blue', label='Ground Truth', alpha=0.5)
    # ax.scatter(vital_data['x_transformed'], vital_data['y_transformed'], vital_data['z_transformed'], c='red', label='transformed', alpha=0.5)
    # ax.set_title('3D Estimated vs Ground Truth Positions')
    # ax.set_xlabel('X Coordinate')
    # ax.set_ylabel('Y Coordinate')
    # ax.set_zlabel('Z Coordinate')
    # ax.legend()
    # plt.savefig('3d_scatter.png')
    # plt.show()

    # # Visualization 7: Centroid Positions Scatter Plot
    # plt.figure(figsize=(10, 6))
    # sns.scatterplot(x='centroid_x', y='centroid_y', hue='image_id', size='error_no_z', 
    #                 sizes=(20, 200), data=vital_data, alpha=0.6)
    # plt.title('Centroid Positions with 2D Error Magnitude')
    # plt.xlabel('Centroid X')
    # plt.ylabel('Centroid Y')
    # plt.savefig('centroid_scatter.png')
    # plt.show()

    # # Visualization 8: Error vs Centroid Position (X) with Fitted Line
    # plt.figure(figsize=(10, 6))
    # sns.scatterplot(x='centroid_x', y='error_no_z', hue='probe_id', data=vital_data, alpha=0.6)
    # sns.regplot(x='centroid_x', y='error_no_z', data=vital_data, scatter=False, color='red', 
    #             line_kws={"label": "Fitted Line"})
    # plt.title('2D Error vs Centroid X Position with Fitted Line')
    # plt.xlabel('Centroid X')
    # plt.ylabel('2D Error')
    # plt.legend()
    # plt.savefig('error_no_z_vs_centroid_x_with_line.png')
    # plt.show()

except FileNotFoundError:
    print("Error: 'results.csv' not found. Please ensure the file is in the correct directory.")
except Exception as e:
    print(f"An error occurred: {str(e)}")
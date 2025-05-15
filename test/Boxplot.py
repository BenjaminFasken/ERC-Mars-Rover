import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import os

# Read the CSV file
df = pd.read_csv('test\pose_errors.csv')

# Create a column to categorize the test types based on the file paths
def categorize_test(filepath):
    if '1min stone data' in filepath:
        return '1min obstacles'  # 1min with stones/obstacles
    elif '1min data' in filepath:
        return '1min'
    elif '2_5min data' in filepath:
        return '2.5min'
    elif '5min data' in filepath:
        return '5min'
    else:
        return 'Unknown'

df['Test_Type'] = df['File'].apply(categorize_test)

# Convert all numeric columns to their absolute values
numeric_columns = df.select_dtypes(include=[np.number]).columns
df[numeric_columns] = df[numeric_columns].abs()

# Order the categories for consistent visualization
category_order = ['1min', '2.5min', '5min', '1min obstacles']

# Set the style
plt.style.use('ggplot')
sns.set_palette('colorblind')

# 1. Total Position Error Comparison
plt.figure(figsize=(6, 6))
ax = sns.boxplot(x='Test_Type', y='Total_Position_Error', data=df, order=category_order,
                meanline=True, showmeans=True,
                meanprops={"linestyle":"--", "color":"black", "linewidth":2, "marker":""},
                medianprops={"linewidth":2},
            )
# Add individual data points
sns.stripplot(x='Test_Type', y='Total_Position_Error', data=df, 
              order=category_order, color='black', size=4, alpha=0.7)
ax.set_title('Total Position Error by Test Duration', fontsize=22)
ax.set_xlabel('Test Type', fontsize=18)
ax.set_ylabel('Total Position Error [m]', fontsize=18)
ax.tick_params(axis='y', labelsize=14)  # Adjust font size for y-axis ticks
ax.tick_params(axis='x', labelsize=14)  # Adjust font size for x-axis ticks
#plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig('total_position_error.png')

# 2. Position errors by axis (X, Y, Z)
fig, axes = plt.subplots(1, 3, figsize=(18, 6))
for i, (col, title) in enumerate(zip(['X_Error', 'Y_Error', 'Z_Error'], 
                                    ['X-Axis', 'Y-Axis', 'Z-Axis'])):
    sns.boxplot(x='Test_Type', y=col, data=df, order=category_order, ax=axes[i],
                meanline=True, showmeans=True,
                meanprops={"linestyle":"--", "color":"black", "linewidth":2, "marker":""},
                medianprops={"linewidth":2},
                )
    # Add individual data points
    sns.stripplot(x='Test_Type', y=col, data=df, order=category_order, 
                 color='black', size=4, alpha=0.7, ax=axes[i])
    axes[i].set_title(f'{title} Position Error', fontsize=22)
    axes[i].set_xlabel('Test Type', fontsize=18)
    axes[i].set_ylabel(f'Error [m]', fontsize=18)
    axes[i].tick_params(axis='y', labelsize=14)  # Adjust font size for y-axis ticks
    axes[i].tick_params(axis='x', labelsize=14)  # Adjust font size for x-axis ticks
    #need same tick size for all subplots
    axes[i].set_ylim(0, df[col].max() * 1.1)  # Set y-limits to be the same for all subplots
    axes[i].set_yticks(np.arange(0, df[col].max() * 1.1, step=0.5))  # Set y-ticks to be the same for all subplots
plt.tight_layout()
#plt.suptitle('Position Errors by Axis', fontsize=16, y=1.02)
plt.savefig('position_errors_by_axis.png')

# 3. Orientation errors (Roll, Pitch, Yaw)
fig, axes = plt.subplots(1, 3, figsize=(18, 6))
for i, (col, title) in enumerate(zip(['Roll_Error', 'Pitch_Error', 'Yaw_Error'], 
                                    ['Roll', 'Pitch', 'Yaw'])):
    sns.boxplot(x='Test_Type', y=col, data=df, order=category_order, ax=axes[i], 
                meanline=True, showmeans=True,
                meanprops={"linestyle":"--", "color":"black", "linewidth":2, "marker":""},
                medianprops={"linewidth":2},
                )
    # Add individual data points
    sns.stripplot(x='Test_Type', y=col, data=df, order=category_order, 
                 color='black', size=4, alpha=0.7, ax=axes[i])
    axes[i].set_title(f'{title} Orientation Error', fontsize=22)
    axes[i].set_xlabel('Test Type', fontsize=18)
    axes[i].set_ylabel(f'Error [degrees]', fontsize=18)
    axes[i].tick_params(axis='y', labelsize=14)  # Adjust font size for y-axis ticks
    axes[i].tick_params(axis='x', labelsize=14)  # Adjust font size for x-axis ticks
plt.tight_layout()
#plt.suptitle('Orientation Errors', fontsize=16, y=1.02)
plt.savefig('orientation_errors.png')

# 4. Summary table of mean errors and standard deviations
summary = df.groupby('Test_Type')[['Total_Position_Error', 'X_Error', 'Y_Error', 'Z_Error', 
                                  'Roll_Error', 'Pitch_Error', 'Yaw_Error']].agg(['mean', 'std'])

# Print summary statistics
print("Summary Statistics:")
print(summary)

# Show all plots
plt.show()
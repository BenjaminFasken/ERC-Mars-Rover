import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from mpl_toolkits.axes_grid1 import make_axes_locatable

# Load data from CSV
data = pd.read_csv('optuna_trials.csv')

# Load into dataframe
df = pd.DataFrame(data)

# Columns to plot (parameters)
columns = [
    'params_cos_lr',
    'params_lrf',
    'params_momentum',
    'params_warmup_epochs',
    'params_warmup_momentum',
    'params_weight_decay',
]

# Normalize data
norm_df = df.copy()
for col in columns:
    if df[col].dtype == 'bool':
        norm_df[col] = df[col].astype(int)
    else:
        norm_df[col] = (df[col] - df[col].min()) / (df[col].max() - df[col].min())

# Normalize 'value' for coloring
norm = mcolors.Normalize(vmin=df['value'].min(), vmax=df['value'].max())
cmap = cm.get_cmap('coolwarm_r')  # low red to high blue

# Setup plot
fig, ax = plt.subplots(figsize=(12, 6))
# Use axes divider to place colorbar flush with curves
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="3%", pad=-0.5)

x = np.arange(len(columns) + 1)  # Extra for colorbar endpoint

'''
# Plot each solution without cropping y-values
for i in range(len(norm_df)):
    y = norm_df.loc[i, columns].values.astype(float)
    y = np.append(y, norm(df.loc[i, 'value']))  # Endpoint for color blending

    x_new = np.linspace(x.min(), x.max(), 300)
    spl = make_interp_spline(x, y, k=3)
    y_smooth = spl(x_new)

    color = cmap(norm(df.loc[i, 'value']))
    ax.plot(x_new, y_smooth, alpha=0.7, color=color)
'''

for i in range(len(norm_df)):
    y = norm_df.loc[i, columns].values.astype(float)
    y = np.append(y, norm(df.loc[i, 'value']))

    x_new = np.linspace(x.min(), x.max(), 300)
    spl = make_interp_spline(x, y, k=3)
    y_smooth = spl(x_new)

    # <— cap here
    y_smooth = np.clip(y_smooth, 0, 1)

    color = cmap(norm(df.loc[i, 'value']))
    ax.plot(x_new, y_smooth, alpha=0.7, color=color)


# Draw parameter axes and labels
for xi, col in zip(x[:-1], columns):
    ax.axvline(x=xi, color='grey', linestyle='--', alpha=0.5)
    # Show original values
    vals = np.linspace(0, 1, 5)
    if df[col].dtype == 'bool':
        orig = [0, 1]; pos = [0, 1]
    else:
        orig = np.linspace(df[col].min(), df[col].max(), 5); pos = vals
    for o, p in zip(orig, pos):
        label = f"{o:.3f}" if isinstance(o, float) else str(o)
        ax.text(xi, p, label, fontsize=8, ha='center', va='center')

ax.set_xticks(x[:-1])
ax.set_xticklabels(columns, rotation=45, ha='right')
ax.set_ylabel('Normalized Value')
ax.set_title('Parallel Coordinates Plot with Curves Merging into Colorbar')
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_bounds(0, 6)
ax.spines['top'].set_bounds(0, 6)
ax.set_ylim(-0.01, 1.01)


# Draw colorbar flush against curves
sm = cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
cb = fig.colorbar(sm, cax=cax)
cb.set_label('Value')
cb.set_ticks(np.linspace(df['value'].min(), df['value'].max(), 5))
cb.ax.set_yticklabels([f"{v:.3f}" for v in np.linspace(df['value'].min(), df['value'].max(), 5)])

plt.tight_layout()
plt.show()

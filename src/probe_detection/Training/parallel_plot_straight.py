import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from mpl_toolkits.axes_grid1 import make_axes_locatable

# Load data from CSV
data = pd.read_csv('optuna_trials.csv')

# Load data
df = pd.DataFrame(data)
columns = [
    'params_cos_lr', 'params_lrf', 'params_momentum',
    'params_warmup_epochs', 'params_warmup_momentum', 'params_weight_decay'
]

# Normalize parameters
norm_df = df.copy()
for col in columns:
    if df[col].dtype == 'bool':
        norm_df[col] = df[col].astype(int)
    else:
        norm_df[col] = (df[col] - df[col].min()) / (df[col].max() - df[col].min())

# Color normalization for value
dnorm = mcolors.Normalize(vmin=df['value'].min(), vmax=df['value'].max())
cmap = cm.get_cmap('coolwarm_r')

# Plot setup
fig, ax = plt.subplots(figsize=(12, 6))
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="3%", pad=-0.5)

# X positions including endpoint for colorbar overlay
x = np.arange(len(columns) + 1)

# Plot straight-line parallel coordinates
for i in range(len(norm_df)):
    y = norm_df.loc[i, columns].values.astype(float)
    # append end at normalized 'value' for color merge
    y = np.append(y, dnorm(df.loc[i, 'value']))
    # clip to [0,1]
    y = np.clip(y, 0, 1)
    color = cmap(dnorm(df.loc[i, 'value']))
    ax.plot(x, y, alpha=0.7, color=color)

# Draw vertical axes and labels
for xi, col in zip(x[:-1], columns):
    ax.axvline(x=xi, color='grey', linestyle='--', alpha=0.5)
    vals = np.linspace(0, 1, 5)
    if df[col].dtype == 'bool':
        orig = [0, 1]; pos = [0, 1]
    else:
        orig = np.linspace(df[col].min(), df[col].max(), 5); pos = vals
    for o, p in zip(orig, pos):
        label = f"{o:.3f}" if isinstance(o, float) else str(o)
        ax.text(xi, p, label, fontsize=8, ha='center', va='center')

# Spine and ticks
ax.set_xticks(x[:-1])
ax.set_xticklabels(columns, rotation=45, ha='right')
ax.set_ylabel('Normalized Value')
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_bounds(0, len(columns))
ax.spines['top'].set_bounds(0, len(columns))
ax.set_ylim(-0.01, 1.01)

# Colorbar flush
sm = cm.ScalarMappable(cmap=cmap, norm=dnorm)
sm.set_array([])
cb = fig.colorbar(sm, cax=cax)
cb.set_label('Value')
cb.set_ticks(np.linspace(df['value'].min(), df['value'].max(), 5))
cb.ax.set_yticklabels([f"{v:.3f}" for v in np.linspace(df['value'].min(), df['value'].max(), 5)])

plt.tight_layout()
plt.show()

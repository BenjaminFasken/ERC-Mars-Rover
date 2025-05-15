import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from mpl_toolkits.axes_grid1 import make_axes_locatable

# 1) Load data from CSV
df = pd.read_csv('optuna_trials.csv')

# 2) Keep only COMPLETE trials
df = df[df['state'] == 'COMPLETE'].reset_index(drop=True)

# 3) Specify which hyper-parameter columns to plot
columns = [
    'params_cos_lr', 'params_lrf', 'params_momentum',
    'params_warmup_epochs', 'params_warmup_momentum', 'params_weight_decay'
]

# 4) Normalize parameters
norm_df = df.copy()
for col in columns:
    if norm_df[col].dtype == 'bool':
        norm_df[col] = norm_df[col].astype(int)
    else:
        norm_df[col] = (norm_df[col] - norm_df[col].min()) / (norm_df[col].max() - norm_df[col].min())

# 5) Color normalization for the 'value' metric
dnorm = mcolors.Normalize(vmin=df['value'].min(), vmax=df['value'].max())
cmap = cm.get_cmap('coolwarm_r')

# 6) Plot setup
fig, ax = plt.subplots(figsize=(12, 6))
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="3%", pad=-0.5)

# 7) X positions (one per param, +1 for the value colorbar anchor)
x = np.arange(len(columns) + 1)

# 8) Draw each trial’s line in parallel-coords space
for i in range(len(norm_df)):
    y = norm_df.loc[i, columns].values.astype(float)
    # Append the normalized 'value' at the end for color mapping
    y = np.append(y, dnorm(df.loc[i, 'value']))
    y = np.clip(y, 0, 1)
    color = cmap(dnorm(df.loc[i, 'value']))
    ax.plot(x, y, alpha=0.7, color=color)

# 9) Draw the vertical axes and label ticks
for xi, col in zip(x[:-1], columns):
    ax.axvline(x=xi, color='grey', linestyle='--', alpha=0.5)
    ticks = np.linspace(0, 1, 5)
    if df[col].dtype == 'bool':
        orig_vals, pos = [0, 1], [0, 1]
    else:
        orig_vals, pos = np.linspace(df[col].min(), df[col].max(), 5), ticks
    for orig, p in zip(orig_vals, pos):
        label = f"{orig:.3f}" if isinstance(orig, float) else str(orig)
        ax.text(xi, p, label, fontsize=8, ha='center', va='center')

# 10) Final styling
ax.set_xticks(x[:-1])
ax.set_xticklabels(columns, rotation=45, ha='right')
ax.set_ylabel('Normalized Value')
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_bounds(0, len(columns))
ax.spines['top'].set_bounds(0, len(columns))
ax.set_ylim(-0.01, 1.01)

# 11) Colorbar
sm = cm.ScalarMappable(cmap=cmap, norm=dnorm)
sm.set_array([])
cb = fig.colorbar(sm, cax=cax)
cb.set_label('Value')
cb.set_ticks(np.linspace(df['value'].min(), df['value'].max(), 5))
cb.ax.set_yticklabels([f"{v:.3f}" for v in np.linspace(df['value'].min(), df['value'].max(), 5)])

plt.tight_layout()
plt.show()

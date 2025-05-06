import pandas as pd
import matplotlib.pyplot as plt
import os

# Paths to both runs
RUNS_DIR  = "runs"
SGD_DIR   = os.path.join(RUNS_DIR, "showdown_SGD",   "train", "results.csv")
ADAMW_DIR = os.path.join(RUNS_DIR, "showdown_AdamW", "train", "results.csv")

# Load results
sgd   = pd.read_csv(SGD_DIR)
adamw = pd.read_csv(ADAMW_DIR)

# Extract epochs and mask mAP50‑95 (the “(M)” column)
epochs_sgd     = sgd['epoch'].to_numpy()
mask95_sgd     = sgd['metrics/mAP50-95(M)'].to_numpy()

epochs_adamw   = adamw['epoch'].to_numpy()
mask95_adamw   = adamw['metrics/mAP50-95(M)'].to_numpy()

# Plot mask mAP50‑95 over epochs
plt.figure(figsize=(8,5))
plt.plot(epochs_sgd,   mask95_sgd,   label='SGD',   linewidth=2)
plt.plot(epochs_adamw, mask95_adamw, label='AdamW', linewidth=2)

plt.xlabel('Epoch')
plt.ylabel('Mask mAP50‑95')
plt.title('SGD vs AdamW: Mask mAP50‑95 Over Epochs')
plt.legend()
plt.grid(True)
plt.tight_layout()

out_path = 'showdown_mask_mAP50-95_curves.png'
plt.savefig(out_path)
plt.show()

print(f"Saved mask‑mAP50‑95 curve as '{out_path}'")

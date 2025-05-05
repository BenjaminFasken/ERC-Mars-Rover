import pandas as pd
import matplotlib.pyplot as plt
import os

# Paths to both runs
RUNS_DIR = "runs"
SGD_DIR = os.path.join(RUNS_DIR, "showdown_SGD", "train")
ADAMW_DIR = os.path.join(RUNS_DIR, "showdown_AdamW", "train")

# Load results
sgd_results = pd.read_csv(os.path.join(SGD_DIR, "results.csv"))
adamw_results = pd.read_csv(os.path.join(ADAMW_DIR, "results.csv"))

# Extract epochs and mAP50 values
epochs_sgd = sgd_results['epoch'].to_numpy()
map50_sgd = sgd_results['metrics/precision(B)'].to_numpy()  # mAP50 column

epochs_adamw = adamw_results['epoch'].to_numpy()
map50_adamw = adamw_results['metrics/precision(B)'].to_numpy()

# Plot
plt.figure(figsize=(10, 6))
plt.plot(epochs_sgd, map50_sgd, label='SGD', color='blue', linewidth=2)
plt.plot(epochs_adamw, map50_adamw, label='AdamW', color='orange', linewidth=2)

plt.xlabel('Epoch')
plt.ylabel('mAP50')
plt.title('SGD vs AdamW: mAP50 Over Epochs')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig('showdown_map50_curves.png')
plt.show()

print("Saved curve plot as 'showdown_map50_curves.png'")

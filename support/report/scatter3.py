import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_parquet("../../orchestrator/unfiltered.parquet")
df2 = pd.read_parquet("../../orchestrator/filtered.parquet")

df2 = df2.iloc[::20, :]

x = df["time"]
y = df["fy"]

# Determine your data limits EXACTLY
# These will be used in pgfplots axis environment

xmin, xmax = x.min(), x.max()
ymin, ymax = y.min(), y.max()

fig, ax = plt.subplots(figsize=(6, 4))  # Adjust figsize as needed

# Plot ONLY the points
ax.scatter(
    x, y, s=2, c="black", alpha=0.3, edgecolors="none"
)  # s is size, alpha for transparency

ax.plot(df2["time"], df2["fy"], c="black", alpha=0.5)

# Crucial: Remove all decorations from matplotlib
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.axis("off")  # Turn off axes, labels, everything
fig.tight_layout(pad=0)  # Remove padding

# Save as PDF (vector, good for scaling) or PNG (raster)
plt.savefig("scatter_points.pdf", bbox_inches="tight", pad_inches=0)
# plt.savefig("scatter_points.png", dpi=300, bbox_inches='tight', pad_inches=0)
print(
    f"Data limits for pgfplots: xmin={xmin:.3f}, xmax={xmax:.3f}, ymin={ymin:.3f}, ymax={ymax:.3f}"
)
plt.show()

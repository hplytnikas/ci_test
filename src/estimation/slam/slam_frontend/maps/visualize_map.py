import pandas as pd
import matplotlib.pyplot as plt
import sys

# Check for filename argument
if len(sys.argv) < 2:
    print("Usage: python visualize_map.py <filename>")
    sys.exit(1)

filename = sys.argv[1]

# Read the CSV file into a pandas DataFrame
df = pd.read_csv(
    filename,
    header=None,
    names=[
        "x",
        "y",
        "prob_cone",
        "prob_blue",
        "prob_yellow",
        "prob_orange",
        "prob_orange_big",
    ],
)

# Filter cones by probability
df["color"] = "black"  # Default color
df.loc[df["prob_cone"] < 0.6, "color"] = "gray"  # Low probability cones
df.loc[df["prob_blue"] > 0.5, "color"] = "blue"
df.loc[df["prob_yellow"] > 0.5, "color"] = "yellow"
df.loc[df["prob_orange"] > 0.5, "color"] = "orange"
df.loc[df["prob_orange_big"] > 0.5, "color"] = "darkorange"

plt.title("Map visualization")

# Plot the cones
plt.scatter(df["x"], df["y"], c=df["color"])

# Plot the origin
plt.scatter([0], [0], c="green", marker="o")

# Set the aspect of the plot to be equal
plt.axis("equal")

# Show the plot with interactive navigation
plt.show()

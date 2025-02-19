from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
import argparse


def main(file_path):
    # Define the topic names
    VelocityEstimationTopic = "/vcu_msgs/velocity_estimation"

    # List of topics to process
    topics_to_process = [VelocityEstimationTopic]

    # Load the MCAP file into a dictionary of DataFrames
    dataframes = load_mcap_to_dataframe(file_path, topics_to_process)

    # Display the first few rows of the DataFrames
    for topic, df in dataframes.items():
        print(f"Topic: {topic}")
        print(df.head())
        print()

    def extract_velocity_estimation(velocity_df):
        velocity_estimation = {"ax": [], "ay": [], "vx": [], "timestamps": []}

        for idx, row in velocity_df.iterrows():
            msg = row["message"]
            ax = msg.acc.x
            ay = msg.acc.y
            vx = msg.vel.x

            velocity_estimation["ax"].append(ax)
            velocity_estimation["ay"].append(ay)
            velocity_estimation["vx"].append(vx)
            velocity_estimation["timestamps"].append(idx)

        return velocity_estimation

    def plot_accelerations(velocity_estimation):
        fig, ax = plt.subplots(figsize=(10, 7))

        # Create custom colormap from CI green to CI red
        cmap = mcolors.LinearSegmentedColormap.from_list(
            "ci_colormap", ["#008237", "#d20a11"]
        )

        sc = ax.scatter(
            velocity_estimation["ay"],
            velocity_estimation["ax"],
            c=velocity_estimation["vx"],
            cmap=cmap,
            alpha=0.5,
        )

        ax.set_xlim(-15.0, 15.0)
        ax.set_ylim(-15.0, 15.0)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_ylabel("Longitudinal Acceleration (ax)", color="#000000")
        ax.set_xlabel("Lateral Acceleration (ay)", color="#000000")
        ax.set_title(
            "gg-plot",
            fontdict={"fontsize": 12, "fontweight": "bold", "color": "#d20a11"},
        )
        ax.grid(True, color="#e3e3e3")

        cbar = plt.colorbar(sc, ax=ax, label="Longitudinal Velocity (vx)")
        cbar.ax.yaxis.label.set_color("#000000")
        cbar.outline.set_edgecolor("#000000")

        plt.show()

    velocity_df = dataframes[VelocityEstimationTopic]
    velocity_estimation = extract_velocity_estimation(velocity_df)
    plot_accelerations(velocity_estimation)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize velocity estimations from an MCAP file."
    )
    parser.add_argument("file_path", type=str, help="Path to the MCAP file.")
    args = parser.parse_args()
    main(args.file_path)

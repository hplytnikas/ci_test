from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
import numpy as np
import argparse
import pandas as pd


def main(file_paths, labels):
    # Ensure the number of labels matches the number of file paths
    if len(file_paths) != len(labels):
        raise ValueError("The number of labels must match the number of file paths")

    # Define the topic names
    VelocityEstimationTopic = "/vcu_msgs/velocity_estimation"

    # List of topics to process
    topics_to_process = [VelocityEstimationTopic]

    dataframes = []
    for i, file_path in enumerate(file_paths):
        # Load the MCAP file into a dictionary of DataFrames
        dfs = load_mcap_to_dataframe(file_path, topics_to_process)
        df = dfs[VelocityEstimationTopic]
        df["source"] = labels[
            i
        ]  # Add a column to indicate the custom label for each source file
        dataframes.append(df)

    # Concatenate all dataframes
    combined_df = pd.concat(dataframes)

    def extract_velocity_estimation(velocity_df):
        velocity_estimation = {"ax": [], "ay": [], "source": []}

        for idx, row in velocity_df.iterrows():
            msg = row["message"]
            ax = msg.acc.x
            ay = msg.acc.y
            source = row["source"]

            velocity_estimation["ax"].append(ax)
            velocity_estimation["ay"].append(ay)
            velocity_estimation["source"].append(source)

        return velocity_estimation

    def plot_accelerations(velocity_estimation):
        # Enable LaTeX rendering
        plt.rc("text", usetex=True)
        plt.rc("font", family="serif")
        plt.rcParams.update({"font.size": 16})  # Increase the font size

        fig, ax = plt.subplots(figsize=(10, 7))

        # Unique sources and assign colors
        sources = list(set(velocity_estimation["source"]))
        colors = plt.cm.get_cmap("tab10", len(sources))

        # Plot EMBOTECH first to ensure it is in the background
        for i, source in enumerate(sources):
            idxs = [
                j for j, s in enumerate(velocity_estimation["source"]) if s == source
            ]
            if source == "EMBOTECH":
                ax.scatter(
                    np.array(velocity_estimation["ay"])[idxs],
                    np.array(velocity_estimation["ax"])[idxs],
                    label=source,
                    color="#d20a11",
                    alpha=0.2,
                    zorder=1,
                )

        # Plot other sources
        for i, source in enumerate(sources):
            if source != "EMBOTECH":
                idxs = [
                    j
                    for j, s in enumerate(velocity_estimation["source"])
                    if s == source
                ]
                if source == "ACADOS":
                    # ACADOS is always lighter blue
                    ax.scatter(
                        np.array(velocity_estimation["ay"])[idxs],
                        np.array(velocity_estimation["ax"])[idxs],
                        label=source,
                        color="#4169E1",
                        alpha=0.2,
                        zorder=2,
                    )
                else:
                    # rest is according to colors
                    ax.scatter(
                        np.array(velocity_estimation["ay"])[idxs],
                        np.array(velocity_estimation["ax"])[idxs],
                        label=source,
                        color=colors(i),
                        alpha=0.2,
                        zorder=2,
                    )

        ax.set_xlim(-15.0, 15.0)
        ax.set_ylim(-15.0, 15.0)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_ylabel(
            r"Longitudinal Acceleration (ax) [m/s$^2$]", color="#000000", fontsize=16
        )
        ax.set_xlabel(
            r"Lateral Acceleration (ay) [m/s$^2$]", color="#000000", fontsize=16
        )
        ax.set_title(r"gg-plot", fontsize=16)
        ax.grid(True, color="#e3e3e3")
        ax.legend(fontsize=16)
        # Set the font size for the tick labels
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)

        plt.show()

    velocity_estimation = extract_velocity_estimation(combined_df)
    plot_accelerations(velocity_estimation)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize velocity estimations from MCAP files."
    )
    parser.add_argument(
        "--file_paths",
        type=str,
        nargs="+",
        help="Paths to the MCAP files.",
        required=True,
    )
    parser.add_argument(
        "--labels",
        type=str,
        nargs="+",
        help="Labels for each MCAP file.",
        required=True,
    )
    args = parser.parse_args()
    main(args.file_paths, args.labels)

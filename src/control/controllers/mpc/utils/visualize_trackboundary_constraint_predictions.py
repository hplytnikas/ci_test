from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import Slider
import numpy as np
import argparse


def main(file_path):
    # Define the topic names
    MpcLoggingTopic = "/control/mpc_logging"

    # List of topics to process
    topics_to_process = [MpcLoggingTopic]

    # Load the MCAP file into a dictionary of DataFrames
    dataframes = load_mcap_to_dataframe(file_path, topics_to_process)

    # Display the first few rows of the DataFrames
    for topic, df in dataframes.items():
        print(f"Topic: {topic}")
        print(df.head())
        print()

    def extract_mpc_predictions(mpc_df):
        predictions = {
            "n": [],
            "mu": [],
            "progress": [],
            "trackbound_min": [],
            "trackbound_max": [],
            "car_width": [],
            "l_to_front": [],
            "l_to_rear": [],
        }

        for idx, row in mpc_df.iterrows():
            msg = row["message"]
            n = np.array(msg.n_prediction)
            mu = np.array(msg.mu_prediction)
            progress = np.array(msg.progress_horizon)
            trackbound_min = np.array(msg.trackbound_min_horizon)
            trackbound_max = np.array(msg.trackbound_max_horizon)

            predictions["n"].append(n)
            predictions["mu"].append(mu)
            predictions["progress"].append(progress)
            predictions["trackbound_min"].append(trackbound_min)
            predictions["trackbound_max"].append(trackbound_max)
            predictions["car_width"].append(msg.model_car_width)
            predictions["l_to_front"].append(msg.model_l_to_front)
            predictions["l_to_rear"].append(msg.model_l_to_rear)

        return predictions

    def rotate_point(x, y, angle):
        rad = np.radians(angle)
        x_rot = x * np.cos(rad) - y * np.sin(rad)
        y_rot = x * np.sin(rad) + y * np.cos(rad)
        return x_rot, y_rot

    def plot_mpc_predictions(mpc_df, predictions):
        fig, ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(left=0.1, bottom=0.25)

        timestamps = mpc_df.index
        n_predictions = predictions["n"]
        mu_predictions = predictions["mu"]
        progress_predictions = predictions["progress"]
        trackbound_min_predictions = predictions["trackbound_min"]
        trackbound_max_predictions = predictions["trackbound_max"]
        car_width = predictions["car_width"][0]
        l_to_front = predictions["l_to_front"][0]
        l_to_rear = predictions["l_to_rear"][0]

        rects = []
        (trackbound_min_line,) = ax.plot(
            [], [], "--", color="#d20a11", label="Trackbound Min"
        )
        (trackbound_max_line,) = ax.plot(
            [], [], "--", color="#d20a11", label="Trackbound Max"
        )

        for _ in range(len(n_predictions[0])):
            rect = Rectangle(
                (0, 0),
                car_width,
                l_to_front + l_to_rear,
                edgecolor="#000000",
                facecolor="none",
            )
            ax.add_patch(rect)
            rects.append(rect)

        ax.set_xlim(-2, 2)
        ax.set_ylim(-2.0, 15.0)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("Lateral Deviation (n)", color="#000000")
        ax.set_ylabel("Progress (s)", color="#000000")
        ax.set_title(
            "MPC Predictions Over Time",
            fontdict={"fontsize": 14, "fontweight": "bold", "color": "#d20a11"},
        )
        ax.legend()
        ax.grid(True, color="#e3e3e3")

        # Create a slider for time selection
        axcolor = "#ffcc00"
        ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03], facecolor=axcolor)
        slider = Slider(ax_slider, "Time", 0, len(timestamps) - 1, valinit=0, valstep=1)

        def update(val):
            idx = int(slider.val)
            n = n_predictions[idx]
            mu = mu_predictions[idx]
            progress = progress_predictions[idx]

            trackbound_min = trackbound_min_predictions[idx]
            trackbound_max = trackbound_max_predictions[idx]

            trackbound_min_line.set_data(-trackbound_min, progress * 1.5)
            trackbound_max_line.set_data(-trackbound_max, progress * 1.5)

            for i, rect in enumerate(rects):
                x = -n[i]
                y = progress[i]
                rect_center_x, rect_center_y = rotate_point(
                    -car_width / 2, -l_to_rear, np.degrees(mu[i])
                )
                rect.set_xy((x + rect_center_x, y + rect_center_y))
                rect.angle = np.degrees(mu[i])

            fig.canvas.draw_idle()

        slider.on_changed(update)
        plt.show()

    mpc_df = dataframes[MpcLoggingTopic]
    predictions = extract_mpc_predictions(mpc_df)
    plot_mpc_predictions(mpc_df, predictions)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize MPC predictions from an MCAP file."
    )
    parser.add_argument("file_path", type=str, help="Path to the MCAP file.")
    args = parser.parse_args()
    main(args.file_path)

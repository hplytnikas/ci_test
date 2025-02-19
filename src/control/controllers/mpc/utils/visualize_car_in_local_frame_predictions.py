from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
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
            "vx": [],
            "vy": [],
            "dpsi": [],
            "x_refpath": [],
            "y_refpath": [],
            "car_width": [],
            "l_to_front": [],
            "l_to_rear": [],
        }

        for idx, row in mpc_df.iterrows():
            msg = row["message"]
            vx = np.array(msg.vx_prediction)
            vy = np.array(msg.vy_prediction)
            dpsi = np.array(msg.dpsi_prediction)
            x_refpath = np.array(msg.x_refpath)
            y_refpath = np.array(msg.y_refpath)

            predictions["vx"].append(vx)
            predictions["vy"].append(vy)
            predictions["dpsi"].append(dpsi)
            predictions["x_refpath"].append(x_refpath)
            predictions["y_refpath"].append(y_refpath)
            predictions["car_width"].append(msg.model_car_width)
            predictions["l_to_front"].append(msg.model_l_to_front)
            predictions["l_to_rear"].append(msg.model_l_to_rear)

        return predictions

    def integrate_pose(vx, vy, dpsi, dt=0.025):
        x = [0]
        y = [0]
        theta = [0]
        for i in range(len(vx)):
            theta.append(theta[-1] + dpsi[i] * dt)
            x.append(x[-1] + (vx[i] * np.cos(theta[i]) - vy[i] * np.sin(theta[i])) * dt)
            y.append(y[-1] + (vx[i] * np.sin(theta[i]) + vy[i] * np.cos(theta[i])) * dt)
        return np.array(x), np.array(y), np.array(theta)

    def rotate_point(x, y, angle):
        rad = np.radians(angle)
        x_rot = x * np.cos(rad) - y * np.sin(rad)
        y_rot = x * np.sin(rad) + y * np.cos(rad)
        return x_rot, y_rot

    def plot_mpc_predictions(mpc_df, predictions):
        fig, ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(left=0.1, bottom=0.25)

        timestamps = mpc_df.index
        vx_predictions = predictions["vx"]
        vy_predictions = predictions["vy"]
        dpsi_predictions = predictions["dpsi"]
        x_refpaths = predictions["x_refpath"]
        y_refpaths = predictions["y_refpath"]
        car_width = predictions["car_width"][0]
        l_to_front = predictions["l_to_front"][0]
        l_to_rear = predictions["l_to_rear"][0]

        rects = []
        arrows = []
        (ref_path_line,) = ax.plot(
            [], [], "grey", linestyle="--", label="Reference Path"
        )
        (pred_path_line,) = ax.plot([], [], "r-", label="Predicted Path")

        for _ in range(len(vx_predictions[0]) + 1):
            rect = Rectangle(
                (0, 0),
                car_width,
                l_to_front + l_to_rear,
                edgecolor="black",
                facecolor="none",
            )
            ax.add_patch(rect)
            rects.append(rect)
            arrow = FancyArrowPatch((0, 0), (0, 0), mutation_scale=10, color="#d20a11")
            ax.add_patch(arrow)
            arrows.append(arrow)

        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 20)
        ax.set_aspect("equal")
        ax.set_xlabel("Lateral Position (y)", color="#000000")
        ax.set_ylabel("Longitudinal Position (x)", color="#000000")
        ax.set_title(
            "MPC Predictions and Reference Path",
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
            vx = vx_predictions[idx]
            vy = vy_predictions[idx]
            dpsi = dpsi_predictions[idx]
            x_refpath = x_refpaths[idx]
            y_refpath = y_refpaths[idx]

            x_pred, y_pred, theta_pred = integrate_pose(vx, vy, dpsi)

            ref_path_line.set_data(-y_refpath, x_refpath)  # Switch x and y for rotation
            pred_path_line.set_data(-y_pred, x_pred)  # Switch x and y for rotation

            for i, rect in enumerate(rects[:-1]):
                x = x_pred[i]
                y = y_pred[i]

                # Correctly transform the rectangle position and angle
                x_cog = x - (l_to_rear * np.cos(theta_pred[i]))
                y_cog = -y + (l_to_rear * np.sin(theta_pred[i]))

                rect.set_xy((y_cog - car_width / 2.0, x_cog))
                rect.angle = np.degrees(theta_pred[i])  # Adjust angle for rotation

                arrow = arrows[i]
                arrow.set_positions(
                    [-y, x],
                    [
                        -y
                        - vy[i] * np.cos(theta_pred[i])
                        - vx[i] * np.sin(theta_pred[i]),
                        x
                        + vx[i] * np.cos(theta_pred[i])
                        - vy[i] * np.sin(theta_pred[i]),
                    ],
                )

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

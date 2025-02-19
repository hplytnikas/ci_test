from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import pandas as pd
import argparse


def main(file_path):
    # Define the topic names
    CarCommandTopic = "/control/car_command"
    MpcLoggingTopic = "/control/mpc_logging"
    VelocityEstimationTopic = "/vcu_msgs/velocity_estimation"
    SteeringFeedbackTopic = "/vcu_msgs/steering_feedback"

    # List of topics to process
    topics_to_process = [
        CarCommandTopic,
        MpcLoggingTopic,
        VelocityEstimationTopic,
        SteeringFeedbackTopic,
    ]

    # Load the MCAP file into a dictionary of DataFrames
    dataframes = load_mcap_to_dataframe(file_path, topics_to_process)

    # Display the first few rows of the DataFrames
    for topic, df in dataframes.items():
        print(f"Topic: {topic}")
        print(df.head())
        print()

    def interactive_plot(dataframes):
        mpc_df = dataframes["/control/mpc_logging"]
        velocity_df = dataframes["/vcu_msgs/velocity_estimation"]
        steering_feedback_df = dataframes["/vcu_msgs/steering_feedback"]

        fig, axs = plt.subplots(7, 1, figsize=(16, 18), sharex=True)
        plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.25, hspace=0.4)

        # Plot the longitudinal velocity
        vx_data = velocity_df["message"].apply(lambda x: x.vel.x)
        vy_data = velocity_df["message"].apply(lambda x: x.vel.y)
        dpsi_data = velocity_df["message"].apply(lambda x: x.vel.theta)
        velocity_lines = []
        velocity_lines.append(
            axs[0].plot(
                velocity_df.index, vx_data, label="Velocity Estimation", color="black"
            )[0]
        )
        velocity_lines.append(
            axs[1].plot(
                velocity_df.index, vy_data, label="Lateral Velocity", color="black"
            )[0]
        )
        velocity_lines.append(
            axs[2].plot(velocity_df.index, dpsi_data, label="Yaw Rate", color="black")[
                0
            ]
        )

        # Plot the steering feedback
        steering_angle_data = steering_feedback_df["message"].apply(lambda x: x.data)
        (steering_line,) = axs[6].plot(
            steering_feedback_df.index,
            steering_angle_data,
            label="Steering Angle Feedback",
            color="black",
        )

        ax_data = mpc_df["message"].apply(lambda x: x.ax_prediction[0])
        axs[3].plot(mpc_df.index, ax_data, label="ax_prediction", color="black")

        n_data = mpc_df["message"].apply(lambda x: x.n_prediction[0])
        axs[4].plot(mpc_df.index, n_data, label="n_prediction", color="black")

        mu_data = mpc_df["message"].apply(lambda x: x.mu_prediction[0])
        axs[5].plot(mpc_df.index, mu_data, label="mu_prediction", color="black")

        # Initialize the MPC prediction plots
        initial_timestamp = mpc_df.index[0]
        mpc_lines = []
        state_labels = [
            "vx_prediction",
            "vy_prediction",
            "dpsi_prediction",
            "ax_prediction",
            "n_prediction",
            "mu_prediction",
            "dels_prediction",
        ]
        for i, label in enumerate(state_labels):
            mpc_data = mpc_df.iloc[0]["message"].__getattribute__(label)
            prediction_times = [
                initial_timestamp + pd.Timedelta(seconds=j * 0.025)
                for j in range(len(mpc_data))
            ]
            mpc_lines.append(
                axs[i].plot(
                    prediction_times, mpc_data, label=f"MPC {label}", color="#d20a11"
                )[0]
            )

        for ax in axs:
            ax.set_ylabel("Value", color="#000000")
            ax.legend()
            ax.grid(True, color="#e3e3e3")

        axs[-1].set_xlabel("Timestamp", color="#000000")

        # Create a slider for time selection
        axcolor = "#ffcc00"
        ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03], facecolor=axcolor)
        slider = Slider(
            ax_slider, "Time", 0, len(mpc_df.index) - 1, valinit=0, valstep=1
        )

        def update(val):
            idx = int(slider.val)
            timestamp = mpc_df.index[idx]
            for i, label in enumerate(state_labels):
                mpc_data = mpc_df.iloc[idx]["message"].__getattribute__(label)
                prediction_times = [
                    timestamp + pd.Timedelta(seconds=j * 0.025)
                    for j in range(len(mpc_data))
                ]
                mpc_lines[i].set_xdata(prediction_times)
                mpc_lines[i].set_ydata(mpc_data)
            fig.canvas.draw_idle()

        slider.on_changed(update)

        # Maximize the plot window
        manager = plt.get_current_fig_manager()
        manager.window.showMaximized()

        plt.show()

    interactive_plot(dataframes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize MPC predictions from an MCAP file."
    )
    parser.add_argument("file_path", type=str, help="Path to the MCAP file.")
    args = parser.parse_args()
    main(args.file_path)

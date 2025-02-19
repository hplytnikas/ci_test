from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse


def main(file_path):
    # Define the topic names
    CarCommandTopic = "/control/car_command"
    VelocityEstimationTopic = "/vcu_msgs/velocity_estimation"
    SteeringFeedbackTopic = "/vcu_msgs/steering_feedback"

    # List of topics to process
    topics_to_process = [
        CarCommandTopic,
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

    def extract_signals(carcommand_df, velocity_df, steering_feedback_df):
        commands = {
            "ax_command": [],
            "steering_command": [],
            "yaw_rate_command": [],
            "timestamps": [],
        }

        for idx, row in carcommand_df.iterrows():
            msg = row["message"]
            commands["ax_command"].append(msg.a_x[0])
            commands["steering_command"].append(msg.steering_angle[0])
            commands["yaw_rate_command"].append(msg.yaw_rate[0])
            commands["timestamps"].append(row.name)

        velocity_data = {
            "ax_measured": velocity_df["message"].apply(lambda x: x.acc.x),
            "yaw_rate_measured": velocity_df["message"].apply(lambda x: x.vel.theta),
            "timestamps": velocity_df.index,
        }

        steering_data = {
            "steering_measured": steering_feedback_df["message"].apply(
                lambda x: x.data
            ),
            "timestamps": steering_feedback_df.index,
        }

        return commands, velocity_data, steering_data

    def align_signals(commands, velocity_data, steering_data):
        commands_df = pd.DataFrame(commands).set_index("timestamps")
        velocity_df = pd.DataFrame(velocity_data).set_index("timestamps")
        steering_df = pd.DataFrame(steering_data).set_index("timestamps")

        combined_df = commands_df.join(velocity_df, how="outer").join(
            steering_df, how="outer"
        )
        combined_df = combined_df.interpolate(method="time").ffill().bfill()

        return combined_df

    def trim_noninformative_sections(df):
        # Detect the first and last indices where ax_measured is non-zero
        non_zero_ax_measured = df[df["ax_measured"] != 0]

        if not non_zero_ax_measured.empty:
            start_idx = non_zero_ax_measured.index[0]
            end_idx = non_zero_ax_measured.index[-1]
            print(f"Trimming data from {start_idx} to {end_idx}")
            trimmed_df = df.loc[start_idx:end_idx]
        else:
            print("No non-zero ax_measured values found.")
            trimmed_df = df

        return trimmed_df

    def plot_tracking_performance(trimmed_df):
        fig, axs = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

        # Plot ax tracking
        axs[0].plot(
            trimmed_df.index,
            trimmed_df["ax_command"],
            color="#d20a11",
            label="Commanded ax",
        )
        axs[0].plot(
            trimmed_df.index,
            trimmed_df["ax_measured"],
            color="black",
            linestyle="--",
            label="Measured ax",
        )
        axs[0].set_ylabel("Acceleration (ax)", color="#000000")
        axs[0].legend()
        axs[0].grid(True, color="#e3e3e3")

        # Plot steering tracking
        axs[1].plot(
            trimmed_df.index,
            trimmed_df["steering_command"],
            color="#d20a11",
            label="Commanded Steering Angle",
        )
        axs[1].plot(
            trimmed_df.index,
            trimmed_df["steering_measured"],
            color="black",
            linestyle="--",
            label="Measured Steering Angle",
        )
        axs[1].set_ylabel("Steering Angle", color="#000000")
        axs[1].legend()
        axs[1].grid(True, color="#e3e3e3")

        # Plot yaw rate tracking
        axs[2].plot(
            trimmed_df.index,
            trimmed_df["yaw_rate_command"],
            color="#d20a11",
            label="Commanded Yaw Rate",
        )
        axs[2].plot(
            trimmed_df.index,
            trimmed_df["yaw_rate_measured"],
            color="black",
            linestyle="--",
            label="Measured Yaw Rate",
        )
        axs[2].set_ylabel("Yaw Rate", color="#000000")
        axs[2].set_xlabel("Time", color="#000000")
        axs[2].legend()
        axs[2].grid(True, color="#e3e3e3")

        plt.tight_layout()
        plt.show()

    carcommand_df = dataframes[CarCommandTopic]
    velocity_df = dataframes[VelocityEstimationTopic]
    steering_feedback_df = dataframes[SteeringFeedbackTopic]

    commands, velocity_data, steering_data = extract_signals(
        carcommand_df, velocity_df, steering_feedback_df
    )
    combined_df = align_signals(commands, velocity_data, steering_data)

    # Add debugging statements
    print("Combined DataFrame head:\n", combined_df.head())
    print("Combined DataFrame tail:\n", combined_df.tail())

    trimmed_df = trim_noninformative_sections(combined_df)

    # Add debugging statements
    print("Trimmed DataFrame head:\n", trimmed_df.head())
    print("Trimmed DataFrame tail:\n", trimmed_df.tail())

    plot_tracking_performance(trimmed_df)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize tracking performance of ax, steering, and yaw rate."
    )
    parser.add_argument("file_path", type=str, help="Path to the MCAP file.")
    args = parser.parse_args()
    main(args.file_path)

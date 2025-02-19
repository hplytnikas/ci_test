import argparse
import random
from load_mcap_to_dataframe import load_mcap_to_dataframe
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Define the topic names
CarCommandTopic = "/control/car_command"
VelocityEstimationTopic = "/vcu_msgs/velocity_estimation"
SteeringFeedbackTopic = "/vcu_msgs/steering_feedback"


def extract_car_command_signals(carcommand_df):
    commands = {
        "ax_command": [],
        "steering_command": [],
        "yaw_rate_command": [],
        "timestamps": [],
    }

    for idx, row in carcommand_df.iterrows():
        msg = row["message"]
        commands["ax_command"].extend(msg.a_x)
        commands["steering_command"].extend(msg.steering_angle)
        commands["yaw_rate_command"].extend(msg.yaw_rate)
        time_step = pd.Timedelta(milliseconds=25)
        timestamp = row.name
        commands["timestamps"].extend(
            [timestamp, timestamp + time_step, timestamp + 2 * time_step]
        )

    return commands


def plot_car_command_vs_actual(dataframes):
    carcommand_df = dataframes[CarCommandTopic]
    velocity_df = dataframes[VelocityEstimationTopic]
    steering_feedback_df = dataframes[SteeringFeedbackTopic]

    commands = extract_car_command_signals(carcommand_df)
    timestamps = pd.to_datetime(commands["timestamps"])

    fig, axs = plt.subplots(3, 1, figsize=(14, 12))

    # Plot commanded vs actual acceleration
    for i in range(0, len(commands["ax_command"]), 3):
        axs[0].plot(
            timestamps[i : i + 3],
            commands["ax_command"][i : i + 3],
            color="#d20a11",
            label="Commanded ax" if i == 0 else "",
        )
    axs[0].plot(
        velocity_df.index,
        velocity_df["message"].apply(lambda x: x.acc.x),
        color="black",
        label="Actual ax",
    )
    axs[0].set_ylabel("Acceleration (ax)", color="#000000")
    axs[0].legend()
    axs[0].grid(True, color="#e3e3e3")

    # Plot commanded vs actual steering angle
    for i in range(0, len(commands["steering_command"]), 3):
        axs[1].plot(
            timestamps[i : i + 3],
            commands["steering_command"][i : i + 3],
            color="#d20a11",
            label="Commanded Steering Angle" if i == 0 else "",
        )
    axs[1].plot(
        steering_feedback_df.index,
        steering_feedback_df["message"].apply(lambda x: x.data),
        color="black",
        label="Actual Steering Angle",
    )
    axs[1].set_ylabel("Steering Angle", color="#000000")
    axs[1].legend()
    axs[1].grid(True, color="#e3e3e3")

    # Plot commanded vs actual yaw rate
    for i in range(0, len(commands["yaw_rate_command"]), 3):
        axs[2].plot(
            timestamps[i : i + 3],
            commands["yaw_rate_command"][i : i + 3],
            color="#d20a11",
            label="Commanded Yaw Rate" if i == 0 else "",
        )
    axs[2].plot(
        velocity_df.index,
        velocity_df["message"].apply(lambda x: x.vel.theta),
        color="black",
        label="Actual Yaw Rate",
    )
    axs[2].set_ylabel("Yaw Rate", color="#000000")
    axs[2].legend()
    axs[2].grid(True, color="#e3e3e3")

    plt.xlabel("Time", color="#000000")
    plt.tight_layout()
    plt.show()


def main(file_path):
    dataframes = load_mcap_to_dataframe(
        file_path, [CarCommandTopic, VelocityEstimationTopic, SteeringFeedbackTopic]
    )
    plot_car_command_vs_actual(dataframes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compare car command targets to actual signals."
    )
    parser.add_argument("file_path", type=str, help="Path to the MCAP file.")
    args = parser.parse_args()
    main(args.file_path)

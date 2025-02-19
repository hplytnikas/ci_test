import rosbag2_py
import pandas as pd
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt

from std_msgs.msg import String
from control_msgs.msg import MpcLog
from vcu_msgs.msg import VelocityEstimation, CarCommand
from autonomous_msgs.msg import DoubleStamped


# Read and deserialize the messages from the MCAP file
def read_mcap(file_path, topics):
    # Storage Options
    storage_options = rosbag2_py.StorageOptions(uri=file_path, storage_id="mcap")
    # Converter Options
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    # Open the MCAP file
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get all the topics and types in the MCAP file
    topic_types = reader.get_all_topics_and_types()

    # Get the message type from the topic name
    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    # Create a dictionary to store the messages
    data_structure = {topic: {"timestamps": [], "messages": []} for topic in topics}

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # If the topic is not in the list of topics to process, skip it
        if topic not in topics:
            continue

        # Deserialize the message
        try:
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            data_structure[topic]["timestamps"].append(
                pd.to_datetime(timestamp, unit="ns")
            )
            data_structure[topic]["messages"].append(msg)
        except Exception as e:
            print(f"Failed to deserialize message on topic {topic}: {e}")
            continue
    # Close the reader
    del reader

    return data_structure


# Convert the data structure to a pandas DataFrame
def convert_to_dataframe(data_structure):
    dataframes = {}
    for topic, data in data_structure.items():
        df = pd.DataFrame(
            {"timestamp": data["timestamps"], "message": data["messages"]}
        )
        df.set_index("timestamp", inplace=True)
        dataframes[topic] = df
    return dataframes


def load_mcap_to_dataframe(mcap_file_path, topics_to_process):

    # Read the messages from the MCAP file
    data_structure = read_mcap(mcap_file_path, topics_to_process)
    dataframes = convert_to_dataframe(data_structure)

    return dataframes


if __name__ == "__main__":
    mcap_file_path = "rosbag2_2024_06_09-21_25_47/rosbag2_2024_06_09-21_25_47_0.mcap"
    topics_to_process = [
        "/control/car_command",
        "/control/mpc_logging",
        "/vcu_msgs/velocity_estimation",
        "/vcu_msgs/steering_feedback",
    ]
    dataframes = load_mcap_to_dataframe(mcap_file_path, topics_to_process)
    print(dataframes.keys())
    print(dataframes["/control/car_command"].head())
    print(dataframes["/control/mpc_logging"].head())
    print(dataframes["/vcu_msgs/velocity_estimation"].head())
    print(dataframes["/vcu_msgs/steering_feedback"].head())

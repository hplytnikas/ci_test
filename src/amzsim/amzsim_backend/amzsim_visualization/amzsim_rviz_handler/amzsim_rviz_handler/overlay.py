#!/usr/bin/env python3

# AMZ Driverless Project
#
# Copyright (c) 2022 Authors:
#   - Example <example@ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
"""
Overlay for Rviz visualisation inside sim. Allows to display lap info and velocity.
"""

from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Float32
import rospy

# pylint: disable=import-error
from autonomous_msgs.msg import IntStamped, DoubleStamped
from vcu_msgs.msg import VelocityEstimation

velocity_publisher = rospy.Publisher(
    "rviz_overlay_amzsim/velocity", Float32, queue_size=1
)
info_panel_publisher = rospy.Publisher(
    "rviz_overlay_amzsim/info_panel", OverlayText, queue_size=1
)

info_panel = OverlayText()
info_panel.text_size = 14
info_panel.fg_color = ColorRGBA(209 / 255.0, 10 / 255.0, 17 / 255.0, 1)
info_panel.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
info_panel.top = 25
info_panel.left = 35
info_panel.width = 400
info_panel.height = 100
info_panel_content = {"distance": 0, "lap_counter": 0, "lap_time": 0}


def reset_info_panel():
    """
    Resets info panel.
    """
    info_panel_content["distance"] = 0
    info_panel_content["lap_counter"] = 0


def update_info_panel():
    """
    Updates info panel.
    """
    info_panel.text = f'Driven distance: {info_panel_content["distance"]:.2f} [m]'
    info_panel.text += "<br>"
    info_panel.text += f'Completed laps: {info_panel_content["lap_counter"]}'
    info_panel.text += "<br>"
    info_panel.text += f'Last lap time: {info_panel_content["lap_time"]:.2f} [s]'
    info_panel_publisher.publish(info_panel)


def velocity_callback(msg):
    """
    Callback for velocity.
    """
    velocity_publisher.publish(msg.vel.x)


def distance_callback(msg):
    """
    Callback for distance.
    """
    if distance_callback.timestamp is None:
        # First measurement
        distance_callback.timestamp = msg.header.stamp.to_sec()
    elif msg.header.stamp.to_sec() < distance_callback.timestamp:
        # Restart rosbag
        distance_callback.timestamp = msg.header.stamp.to_sec()
        reset_info_panel()
    else:
        d_time = msg.header.stamp.to_sec() - distance_callback.timestamp
        distance_callback.timestamp = msg.header.stamp.to_sec()
        info_panel_content["distance"] += d_time * msg.vel.x
    update_info_panel()


distance_callback.timestamp = None


def lap_counter_callback(msg):
    """
    Callback for lap counter.
    """
    if msg.data >= 0:
        info_panel_content["lap_counter"] = msg.data
        update_info_panel()


def lap_time_callback(msg):
    """
    Callback for laptime.
    """
    info_panel_content["lap_time"] = msg.data
    update_info_panel()


def listener():
    """
    Listener that gets information for overlay panel.
    """
    rospy.init_node("rviz_overlay", anonymous=True)

    velocity_subscriber = rospy.Subscriber(  # pylint: disable=unused-variable
        "/vcu_msgs/velocity_estimation_gt", VelocityEstimation, velocity_callback
    )
    velocity_subscriber.impl.add_callback(distance_callback, None)

    # Different disciplines uses different topics to publish the lap count
    lap_counter_subscriber = rospy.Subscriber(  # pylint: disable=unused-variable
        "/common/lap_counter", IntStamped, lap_counter_callback
    )

    lap_time_subscriber = rospy.Subscriber(  # pylint: disable=unused-variable
        "/common/lap_time", DoubleStamped, lap_time_callback
    )

    rospy.spin()


if __name__ == "__main__":
    listener()

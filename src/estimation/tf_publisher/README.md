# TF Publisher

## Subscriptions

The publisher subscribes to the following topics to gather necessary data for computing transformations:

- **/vcu_msgs/velocity_estimation**

## Broadcast Transformation

### odom -> base_link

The core functionality of this TF publisher is to broadcast the transformation between the `odom` frame and the `base_link` frame.

**Explanation**:

The code subscribes to the `/vcu_msgs/velocity_estimation` topic to receive velocity data, which represents the output of the Kalman Filter algorithm computed within the VCU. By integrating these velocity estimates, the publisher is able to continuously update and publish the current transformation between the `odom` frame and the `base_link` frame, which represents the car's current position and orientation.

**Visualization**:

To visualize the VE integrated pose go to the config/default.yaml and set the mode to 'debug'. THe following topics will be published: `/tf_publisher/pose` and `/tf_publisher/pose_array`, and can be visualized using software as RVIZ or FoxGloves

# Static frame publisher

Publishing static transforms is useful to define the relationship between the car base_link and its sensors or non-moving parts.
This package runs a ROS2 node that publishes static transforms to the `/tf_static` topic.

Static transforms can be loaded in two ways:
- load transforms from yaml files
- load transforms from arguments

## Usage

Example 1:
```python
#load from yaml file
...
new_transform_path = ("package://static_tf_publisher/config/castor/new_transform.yaml")
...
perception_tf_static_node = Node(
    ...
    arguments=[ ... , new_transform_path],
)
...
```

Example 2:
```python
#load from arguments
...
perception_tf_static_node = Node(
    ...
    arguments=[ ... , x, y, z, qx, qy, qz, qw, header, child],
)
...
```

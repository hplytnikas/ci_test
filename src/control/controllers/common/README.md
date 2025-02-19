## All controllers use the same inputs

The idea is that controllers should be interchangeable.
To do so we propose having the same interface which is common for all controllers and then a discipline handler which does all the specific things for each discipline (eg. loading the path in skidpad, or ransac on acceleration, or switching from local to global reference on trackdrive).

The interface we propose for the controllers are the following topics that it subscribes to:
 - Velocity estimation --> State
 - Planning or discipline handler --> Reference (raceline/path, boundaries, velocity)

Then the controller publishes:
 - A car command

Because this is common we created the common ControllerNode class which is a node and has publishers and subscribers to all these common interface. The specific controller class will have a pointer to a ControllerNode and this way the algorithm and the ROS2 code are separated in different classes and packages.

Any controller you create should have a pointer to a ControllerNode.
Furthermore, any controller class you define should have an updateControlCommand() method that returns an vcu_msgs::msg::CarCommand.
The ControllerNode will call this function with a timer at the specified control frequency.

A controller might disregard some of the information. For example MPC doesn't necessarily need a reference velocity. Meanwhile pure pursuit uses the path, but not the boundaries.

Because each specific controller will want to publish specific things for visualization purposes then a visualization node for each controller should be created that publishes the controller specific things. (eg. pure pursuit publishes the lookahead point and MPC publishes the predicted trajectory). This visualization node is also owned by the controller.


For the parameters we have used [the generate parameter library](https://github.com/PickNikRobotics/generate_parameter_library) for convenience.

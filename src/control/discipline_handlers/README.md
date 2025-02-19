## Discipline Handlers

There are four driverless disciplines at a Formula Student competition:
- Acceleration *(straight line acceleration to test the car's longitudinal capabilities on a predefined track)*
- Skidpad *(a 'figure-8' discipline where the car needs to drive pure circles on a predefined track to test its lateral capabilities)*
- Autocross *(unknown track which the car has to drive for one lap)*
- Trackdrive *(unknown track which may be different to Autocross which the car has to drive for 10 laps)*

And since the controller is now decoupled from the discipline we are driving, we have discipline handlers that take care of discipline-specific things to provide the controller with a correct reference.

# Acceleration
The discipline handler gives the car a straight line for tracking and makes sure the car knows when to stop.

# Skidpad
We load a predefined .csv file (in SLAM) which is then used for localization. Using an optimized trajectory defined in another .csv file, the discipline handler then provides the controller with the correct reference path and speeds.

# Autocross
The discipline handler just makes sure that, as soon as a lap is completed, the car stops ba sending a large negative acceleration target.

# Trackdrive
Here the handler listens to the local reference given by the local planner until the global map is available and then shifts to providing the controller that reference. Also it counts laps and stops the car after 10.

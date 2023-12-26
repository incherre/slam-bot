# slam-bot
This is a largely standard implementation of a [Simultaneous Localization And Mapping](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) algorithm using an [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) for 2-D mobile robots.

## The robot
The robot interface and main loop is in the file robot_explorer.py.

### Robot
It has wheels and a distance sensor mounted on a servo so that it can capture sweeps of distance measurements.
<img src="readme_resources/robot.jpg?raw=true" alt="A picture of a wheeled robot with a distance sensor." width="500">

### Map
To generate this map I put the robot in a small enclosure of cardboard and ran the explorer program.
<img src="readme_resources/robot_collision_map_2023-12-26-11-50.png?raw=true" alt="An example of a rendered collision map generated from a real robot." width="500">

## Slam class
The file slam.py contains the main interface to the algorithm. The measurement model is a continuous array of evenly spaced range measurements.
The action model is that at each timestep the robot will turn some angle and then travel forward some distance in a straight line.
The interface allows movement, querying current estimated position, and access to a collision map based off of the measurement history.
This class is supported by classes and functions in ekf.py, collision_map.py, and landmark_extraction.py.

## Explorer class
The file explorer.py contains a simple exploration algorithm built on top of SLAM.
This algorithm will find the nearest location with uncertain status in the collision map and attempt to observe it.
When no accessible locations have uncertain status, it will do nothing.

## Rendering collision maps
The file render_collision_map.py contains a util to render saved collision maps as images.
Mostly for debugging/visualization, but can also be used for extracting human-understandable maps from raw saved collision maps.
### Example simulation collision map
This collision map was generated from a simulated environment.
<img src="readme_resources/tk_sim_collision_map.png?raw=true" alt="An example of a rendered collision map generated from simulation." width="500">

## TK sim
The file tk_sim.py contains a bare-bones simulation framework for qualitative integration testing.
When run as main, it will bring up a simple environment for Explorer to explore, and will save the generated collision map after the window is closed.

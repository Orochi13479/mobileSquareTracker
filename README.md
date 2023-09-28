# Square Following Turtlebot 
This project aims to control the robot to follow a straight line by
using the information observed from a sensor on-board the robot.
Suppose there is a square in front of the robot, and there is a
sensor which can observe the positions of the 4 corners of the
square in the current robot coordinate frame. Control the robot to
follow a straight line which is perpendicular to the square. Use
Turtlebot or Simulator and design the environment to control the
robot with noise in the observation.

# Running the Simulation
To Launch the simulator and run the Square Following script open a terminal and input the below:

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_warehouse.launch
```

Once launched Run the provided script,imageProcessing.m, in MATLAB 2021b using ROS melodic.

PROJECT HAS ONLY BEEN TESTED IN MATLAB 2021b USING ROS melodic.
# Square Following Turtlebot 
This project aims to control the robot to follow a straight line by
using the information observed from a sensor on-board the robot.
Suppose there is a square in front of the robot, and there is a
sensor which can observe the positions of the 4 corners of the
square in the current robot coordinate frame. Control the robot to
follow a straight line which is perpendicular to the square. Use
Turtlebot or Simulator and design the environment to control the
robot with noise in the observation.

<b><i>Required Software:</i></b>
- Ubuntu 18.04
- ROS 1 Melodic
- MATLAB 2021b
- Turtlebot3 Packages

# Running the Simulation
To Launch the simulator and run the Mobile Square Tracker script, open a terminal and input the below:

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_testingTargetBox.launch 
```

Once launched, open the script "main.m" then proceed to run the operation section in MATLAB 2021b using ROS melodic.

The project operates in detecting the features on a square object within the Gazebo world. The Turtlebot is set in the Gazebo environment with objects, walls and surfaces to simulate the environment it acts in. 

During the image processing phase of the square object, features of the square are detected by the RGB-D camera attatched onto the Turtlebot. SURF feature detection is utilised to ensure accurate detection but also to maintain computing cost. A PD controller is implemeneted to adjust the Turtlebot's movement and updates as it detects the square. This will also ensure that movement reduces errors prone to sudden changes in acceleration or deceleration.
The Turtlebot, once processing the square will navigate perpendicular to the square object.

**PROJECT HAS ONLY BEEN TESTED IN MATLAB 2021b USING ROS MELODIC, YOUR RESULTS MAY VARY WITH OTHER DISTROS AND VERSIONS.**

<b>Contribution:</b>
40% Matthew - @Orochi13479\
30% Daniel - @daniell-lo\
30% Rosh - @rosh218


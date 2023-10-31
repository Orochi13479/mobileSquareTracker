# Square Following Turtlebot 
This project aims to demonstrate how a robot can follow a straight line perpendicular to the surface of a feature-rich square. It first constructs a point cloud of the square face using SURF(Speeded-Up Robust Features), from there it determines where the normal of the face and the normal of the robot intersect. The robot then travels to this intersection point and rotates back to face the square. To demonstrate competency in feature recognition algorithms only an RGB-D sensor, combined with robot pose is used to localize and path plan, a PD controller is also used to emulate smooth motion. We use Gazebo combined with turtlebot3 packages for our simulation, ROS for robot communication and MATLAB for our computations.

<b><i>Required Software:</i></b>
- Ubuntu 18.04
- ROS 1 Melodic
- MATLAB 2021b
- Turtlebot3 Packages

# Setup
To install custom environments/launch files, copy turtlebot3_gazebo Directory on top of your current catkin_ws/src/turtlebot3_gazebo and replace files if prompted this should place the required files in the correct Directories. If this fails manually drop and place the files into their corresponding directories.
```
catkin_make
```
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

## YouTube Link
[Video Demonstration and Basic Explanation](https://www.youtube.com/watch?v=w8NGltZLQT4)

<b>Contribution:</b>\
40% Matthew - @Orochi13479\
30% Daniel - @daniell-lo\
30% Rosh - @rosh218


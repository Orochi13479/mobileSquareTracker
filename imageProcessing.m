%% Initialise ROS ONLY RUN ONCE PER SESSION
% Clear and Close all
clc;
clear all;
close all;

% Shutdown ROS and Relaunch using ROS_MASTER_URI AND ROS_IP
rosshutdown;
rosinit();

% Setting up node publisher
node = ros.Node('/driving');
drive = ros.Publisher(node,'/cmd_vel','geometry_msgs/Twist');
msg = rosmessage('geometry_msgs/Twist');

% Setting up all necessary subscribers
rgb = rossubscriber('/camera/rgb/image_raw');
depth = rossubscriber('/camera/depth/image_raw');
odom = rossubscriber('/odom');

% Reading Square pattern
qrCode = imread('FollowMeQr.png');

%% Operation
while true
    disp("Running...")
    % Pull Rotation and transformation matrices
    turtleTF = trvec2tform([odom.LatestMessage.Pose.Pose.Position.X odom.LatestMessage.Pose.Pose.Position.Y odom.LatestMessage.Pose.Pose.Position.Z]);
    turtleQuat = quaternion([odom.LatestMessage.Pose.Pose.Orientation.W odom.LatestMessage.Pose.Pose.Orientation.X odom.LatestMessage.Pose.Pose.Orientation.Y odom.LatestMessage.Pose.Pose.Orientation.Z]);
    rotMatrix = rotm2tform(rotmat(turtleQuat, 'point'));

    % Pull Image and Depth data
    [rgbData,alpha]=readImage(rgb.LatestMessage);
    [depthData,alpha]=readImage(depth.LatestMessage);
    
    % Feature Detection
    
    % Delay
    pause(2);
end







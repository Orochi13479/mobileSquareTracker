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
drive = ros.Publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
msg = rosmessage('geometry_msgs/Twist');

% Setting up all necessary subscribers
rgb = rossubscriber('/camera/rgb/image_raw');
depth = rossubscriber('/camera/depth/image_raw');
odom = rossubscriber('/odom');

% Reading Square pattern
squarePattern = rgb2gray(imread('Initial_image.png'));


% Pause to wait for ROS
pause(2);

%% Operation
while true
    disp("Running...")
    
    [controlInput,averageDepth,orientationError] = dataProcessing(rgb,depth,odom,squarePattern);
    

    % Adjust the robot's movement
    if abs(controlInput) >= 0.01
        msg.Angular.Z = controlInput;
        disp("Turning")
    else
        msg.Angular.Z = 0;
        disp("Stopped")
    end
    
    
    % Publish control commands
    send(drive, msg);
    
    % Display depth and orientation error
    disp(['controlInput: ', num2str(controlInput)]);
    % disp(['Average Depth: ', num2str(averageDepth)]);
    % disp(['Orientation Error: ', num2str(orientationError)]);

    % Delay
    pause(1);
end

function [controlInput,averageDepth,orientationError]  = dataProcessing(rgb,depth,odom,squarePattern)
    % Define target orientation (perpendicular to the pattern)
    targetOrientation = 0; % Adjust as needed
    previousError = 0;

    % Define control parameters (adjust as needed)
    Kp = 0.1; % Proportional gain
    Kd = 0.05; % Derivative gain

    % Pull Rotation and transformation matrices
    turtleTF = trvec2tform([odom.LatestMessage.Pose.Pose.Position.X, odom.LatestMessage.Pose.Pose.Position.Y, odom.LatestMessage.Pose.Pose.Position.Z]);
    turtleQuat = quaternion([odom.LatestMessage.Pose.Pose.Orientation.W, odom.LatestMessage.Pose.Pose.Orientation.X, odom.LatestMessage.Pose.Pose.Orientation.Y, odom.LatestMessage.Pose.Pose.Orientation.Z]);
    rotMatrix = rotm2tform(rotmat(turtleQuat, 'point'));

    % Pull Image and Depth data
    [rgbData, ~] = readImage(rgb.LatestMessage);
    gsData = rgb2gray(rgbData);
    [depthData, ~] = readImage(depth.LatestMessage);

    % Feature Detection
    ptsPattern = detectSURFFeatures(squarePattern);
    ptsData = detectSURFFeatures(gsData);
    [featurePattern, validPtsPattern] = extractFeatures(squarePattern, ptsPattern);
    [featureData, validPtsData] = extractFeatures(gsData, ptsData);

    % Match pairs and display them
    indexPairs = matchFeatures(featurePattern, featureData);
    matchedPattern = validPtsPattern(indexPairs(:, 1));
    matchedData = validPtsData(indexPairs(:, 2));
    
    % Remove outliers
    [~,inlierData,inlierPattern] = estimateGeometricTransform(matchedData, matchedPattern,'similarity');

    % For every valid match, obtain the depth data
    depthPts = zeros(1, inlierData.Count);
    
    for i = 1:inlierData.Count
        depthPts(i) = depthData(round(inlierData.Location(i, 2)), round(inlierData.Location(i, 1)));
    end
    
    % Calculate the average depth
    averageDepth = mean(depthPts);

    % Calculate the orientation error
    currentOrientation = atan2(rotMatrix(2, 1), rotMatrix(1, 1));
    orientationError = targetOrientation - currentOrientation;

    % Apply a PID controller to adjust the robot's movement
    controlInput = Kp * orientationError + Kd * (orientationError - previousError);

    % Update previous error
    previousError = orientationError;
end


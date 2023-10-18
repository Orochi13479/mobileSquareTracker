%% Initialize ROS ONLY ONCE PER SESSION
% Clear and close all
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

% Define target orientation (perpendicular to the pattern)
targetOrientation = 0; % Adjust as needed
previousError = 0;
previousDepth = 0;

% Define control parameters (adjust as needed)
Kp = 0.1; % Proportional gain
Kd = 0.05; % Derivative gain

% Define desired average depth and perpendicular distance
desiredAverageDepth = 1.0; % Adjust as needed
desiredPerpendicularDistance = 0.5; % Adjust as needed

isMovingToPerpendicular = false;
isRotatingToPerpendicular = true;

while true
    disp("Running...")
    
    [controlInput, averageDepth, orientationError, previousError] = dataProcessing(rgb, depth, odom, squarePattern, targetOrientation, Kp, Kd, isRotatingToPerpendicular, desiredPerpendicularDistance, previousError, previousDepth);

    if isRotatingToPerpendicular
        % Continue rotating until facing the perpendicular position
        msg.Linear.X = 0;
        msg.Angular.Z = controlInput;

        % Check if orientation error is within a threshold to stop rotation
        if abs(orientationError) < 0.05
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            isRotatingToPerpendicular = false;
            isMovingToPerpendicular = true;
            disp("Facing Perpendicular")
        end
    elseif isMovingToPerpendicular
        if averageDepth < desiredAverageDepth
            % Move forward until reaching the desired average depth
            msg.Linear.X = 0.1; % Adjust the linear velocity as needed
            msg.Angular.Z = 0;
            disp("Moving Forward")
        else
            % Stop and prepare for further actions
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            isMovingToPerpendicular = false;
            disp("Stopping to Take Further Action")
            
            % Add logic to move forward or take further action based on the desiredPerpendicularDistance
            if desiredPerpendicularDistance > 0
                % Move forward for a specific distance (or any other action)
                % You can adjust the linear velocity and time duration for the forward movement
                msg.Linear.X = 0.1; % Adjust the linear velocity as needed
                pause(5); % Adjust the duration as needed
                msg.Linear.X = 0; % Stop moving forward
            else
                % Take other actions as needed
                % For example, you can implement different behaviors here
                disp("Taking Further Action...")
                % Add your custom logic
            end
        end
    end

    % Publish control commands
    send(drive, msg);

    % Display depth and orientation error
    disp(['controlInput: ', num2str(controlInput)]);
    disp(['Average Depth: ', num2str(averageDepth)]);
    disp(['Orientation Error: ', num2str(orientationError)]);
    disp(['Previous Error: ', num2str(previousError)]);
    previousDepth = averageDepth;
    % Delay
    pause(1);
end


function [controlInput, averageDepth, orientationError, previousError] = dataProcessing(rgb, depth, odom, squarePattern, targetOrientation, Kp, Kd, isRotatingToPerpendicular, desiredPerpendicularDistance, previousError, previousDepth)
    % Define control parameters (adjust as needed)
    
    KpOrientation = 0.1; % Proportional gain for orientation
    KdOrientation = 0.05; % Derivative gain for orientation

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
    disp(matchedData.Count)
    if matchedData.Count > 1
        [~, inlierData, inlierPattern] = estimateGeometricTransform(matchedData, matchedPattern, 'similarity');
        % For every valid match, obtain the depth data
        depthPts = zeros(1, inlierData.Count);
    
        for i = 1:inlierData.Count
            depthPts(i) = depthData(round(inlierData.Location(i, 2)), round(inlierData.Location(i, 1)));
        end
    
        % Calculate the average depth
        averageDepth = mean(depthPts);
    else
        averageDepth = previousDepth;
    end

    

    % Calculate the orientation error
    currentOrientation = atan2(rotMatrix(2, 1), rotMatrix(1, 1));
    orientationError = targetOrientation - currentOrientation;

    % Apply a PID controller to adjust the robot's movement
    if isRotatingToPerpendicular
        % Calculate orientation error towards the perpendicular position
        perpendicularX = odom.LatestMessage.Pose.Pose.Position.X - desiredPerpendicularDistance * cos(targetOrientation);
        perpendicularY = odom.LatestMessage.Pose.Pose.Position.Y - desiredPerpendicularDistance * sin(targetOrientation);
        orientationError = atan2(perpendicularY - odom.LatestMessage.Pose.Pose.Position.Y, perpendicularX - odom.LatestMessage.Pose.Pose.Position.X);

        controlInput = KpOrientation * orientationError + KdOrientation * (orientationError - previousError);
    else
        controlInput = Kp * orientationError + Kd * (orientationError - previousError);
    end

    % Update previous error
    previousError = orientationError;
end

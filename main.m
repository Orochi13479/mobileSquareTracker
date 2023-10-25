%% Initialise ROS ONLY RUN ONCE PER SESSION
% Clear and Close all
clc;
clear all;
close all;
profile clear;
profile on;
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
pause(5);

previousError = 0;
% Define control parameters (adjust as needed)
Kp = 0.5; % Proportional gain
Kd = 0.25; % Derivative gain

% Operation
disp("Running...")

[~, ~, ~, ~, featureDetected] = dataProcessing(rgb, depth, odom, squarePattern);

% Rotate until the feature is found
while featureDetected == false
    msg.Angular.Z = 0.2;
    disp("Finding feature")
    % Publish control commands UNCOMMENT WHEN SENDING DATA
    send(drive, msg);
    pause(0.1);
    [~, ~, ~, ~, featureDetected] = dataProcessing(rgb, depth, odom, squarePattern);
end
msg.Angular.Z = 0;
disp("Feature Found")
% Publish control commands UNCOMMENT WHEN SENDING DATA
send(drive, msg);

pause(1);

% Initial States
[~, targetOrientation, intersectionX, intersectionY, ~] = dataProcessing(rgb, depth, odom, squarePattern);
robotX = odom.LatestMessage.Pose.Pose.Position.X;
robotY = odom.LatestMessage.Pose.Pose.Position.Y;
distance = sqrt((robotX - intersectionX)^2+(robotY - intersectionY)^2);
inIntersection = false;

% Update nessesary variables at 0.1 second intervals
while abs(distance) > 0 && inIntersection == false

    counter = 0;

    turtleQuat = quaternion([odom.LatestMessage.Pose.Pose.Orientation.W, odom.LatestMessage.Pose.Pose.Orientation.X, odom.LatestMessage.Pose.Pose.Orientation.Y, odom.LatestMessage.Pose.Pose.Orientation.Z]);
    rotMatrix = rotm2tform(rotmat(turtleQuat, 'point'));

    robotX = odom.LatestMessage.Pose.Pose.Position.X;
    robotY = odom.LatestMessage.Pose.Pose.Position.Y;

    % Calculate the orientation error
    currentOrientation = atan2(rotMatrix(2, 1), rotMatrix(1, 1));
    orientationError = targetOrientation - currentOrientation;

    % Apply a PID controller to adjust the robot's movement
    controlInput = Kp * orientationError + Kd * (orientationError - previousError);

    % Update previous error
    previousError = orientationError;
    distance = sqrt((robotX - intersectionX)^2+(robotY - intersectionY)^2);
    % Adjust the robot's movement

    % Rotate to intersection point and drive there
    while counter < 100
        if abs(controlInput) >= 0.01
            msg.Angular.Z = controlInput;
            % Publish control commands UNCOMMENT WHEN SENDING DATA
            send(drive, msg);
        else
            msg.Angular.Z = 0;
            disp("Aligned")
            % Publish control commands UNCOMMENT WHEN SENDING DATA
            send(drive, msg);
            while abs(distance) > 0.1
                msg.Linear.X = 0.2;
                send(drive, msg);
                robotX = odom.LatestMessage.Pose.Pose.Position.X;
                robotY = odom.LatestMessage.Pose.Pose.Position.Y;
                distance = sqrt((robotX - intersectionX)^2+(robotY - intersectionY)^2);
                disp("Distance: "+num2str(distance))
                pause(0.05)
            end
            disp("Arrived at Intersection")
            msg.Linear.X = 0;
            send(drive, msg);
            inIntersection = true;
            break;
        end
        counter = counter + 1;
    end
    pause(0.1)
    disp("orientationError: "+num2str(orientationError))

end

[~, ~, ~, ~, featureDetected] = dataProcessing(rgb, depth, odom, squarePattern);

% Rotate until the feature is found
while featureDetected == false
    [~, ~, ~, ~, featureDetected] = dataProcessing(rgb, depth, odom, squarePattern);
    msg.Angular.Z = 0.4; % May need Tuning based on system specs
    disp("Finding feature")
    % Publish control commands UNCOMMENT WHEN SENDING DATA
    send(drive, msg);
    pause(0.1);
end
msg.Angular.Z = 0;
disp("Feature Found")
% Publish control commands UNCOMMENT WHEN SENDING DATA
send(drive, msg);

disp("Finished")

profile viewer
profile off
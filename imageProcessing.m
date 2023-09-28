clc;
clear all;
close all;
rosshutdown;
ip = "http://192.168.211.137:11311";
rosinit(ip)

node = ros.Node('/driving');
rgb = rossubscriber('/camera/rgb/image_raw'); %
depth = rossubscriber('/camera/depth/image_raw'); %
odom = rossubscriber('/odom');
drive = ros.Publisher(node,'/cmd_vel','geometry_msgs/Twist');
msg = rosmessage('geometry_msgs/Twist');
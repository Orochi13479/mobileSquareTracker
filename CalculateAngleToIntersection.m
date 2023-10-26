function [angleToIntersection, intersectionX, intersectionY] = CalculateAngleToIntersection(robotX, robotY, intersectx1, intersecty1, intersectx2, intersecty2)
% Calculate the angle to the closest intersection point from the robot's position.
% Input:
% - robotX: x-coordinate of the robot's position
% - robotY: y-coordinate of the robot's position
% - intersectx1: x-coordinate of the first intersection point
% - intersecty1: y-coordinate of the first intersection point
% - intersectx2: x-coordinate of the second intersection point
% - intersecty2: y-coordinate of the second intersection point
% Output:
% - angleToIntersection: Angle in radians from the robot's position to the closest intersection point.

% Calculate distances from the robot to both intersection points
distance1 = sqrt((robotX - intersectx1)^2+(robotY - intersecty1)^2);
distance2 = sqrt((robotX - intersectx2)^2+(robotY - intersecty2)^2);

% Determine the coordinates of the closest intersection point
if distance1 < distance2
    intersectionX = intersectx1;
    intersectionY = intersecty1;
else
    intersectionX = intersectx2;
    intersectionY = intersecty2;
end

% Calculate the vector from the robot to the closest intersection
vecToIntersection = [intersectionX - robotX, intersectionY - robotY];

% Calculate the angle using the arctangent function
angleToIntersection = atan2(vecToIntersection(2), vecToIntersection(1));
end

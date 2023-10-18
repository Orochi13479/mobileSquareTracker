function angleToIntersection = CalculateAngleToIntersection(robotX, robotY, intersectionX, intersectionY)
    % Calculate the angle to the intersection point from the robot's position.
    % Input:
    % - robotX: x-coordinate of the robot's position
    % - robotY: y-coordinate of the robot's position
    % - intersectionX: x-coordinate of the intersection point
    % - intersectionY: y-coordinate of the intersection point
    % Output:
    % - angleToIntersection: Angle in radians from the robot's position to the intersection point.

    % Calculate the vector from the robot to the intersection
    vecToIntersection = [intersectionX - robotX, intersectionY - robotY];

    % Calculate the angle using the arctangent function
    angleToIntersection = atan2(vecToIntersection(2), vecToIntersection(1));
end

function [xIn, yIn] = LineIntersection(vec1, vec2)
% Calculate the intersection point of two lines defined by vectors.
% Input:
% - vec1: [a1, b1, c1], a vector representing the first line (a1*x + b1*y + c1 = 0)
% - vec2: [a2, b2, c2], a vector representing the second line (a2*x + b2*y + c2 = 0)
% Output:
% - xIn: x-coordinate of the intersection point
% - yIn: y-coordinate of the intersection point

% Calculate the intersection point of two lines
% Intersection point formula derived from the equation of two lines
xIn = (vec2(2) * vec1(3) - vec1(2) * vec2(3)) / (vec2(1) * vec1(2) - vec1(1) * vec2(2));
yIn = (vec2(1) * vec1(3) - vec1(1) * vec2(3)) / (vec2(2) * vec1(1) - vec1(2) * vec2(1));
end

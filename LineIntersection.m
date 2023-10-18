function [xIn, yIn] = LineIntersection(pt1, pt2, vec1, vec2)
    % Calculate the intersection point of two lines defined by points and vectors.
    % Input:
    % - pt1: [x1, y1], a point on the first line
    % - pt2: [x2, y2], a point on the second line
    % - vec1: [a1, b1], a vector representing the first line (a1*x + b1*y + c1 = 0)
    % - vec2: [a2, b2], a vector representing the second line (a2*x + b2*y + c2 = 0)
    % Output:
    % - xIn: x-coordinate of the intersection point
    % - yIn: y-coordinate of the intersection point

    % Define the tolerance for zero checks
    tole = 0.02;

    % Initialize intersection point coordinates
    xIn = [];
    yIn = [];

    % Calculate the slope (m) and y-intercept (b) of the first line
    if abs(vec1(1)) > tole && abs(vec1(2)) > tole
        m1 = vec1(2) / vec1(1);
        b1 = pt1(2) - pt1(1) * m1;
    else
        % Handle the case of a vertical line (infinite slope)
        m1 = 0;
        b1 = pt1(2);
    end

    % Calculate the slope (m) and y-intercept (b) of the second line
    if abs(vec2(1)) > tole && abs(vec2(2)) > tole
        m2 = vec2(2) / vec2(1);
        b2 = pt2(2) - pt2(1) * m2;
    else
        % Handle the case of a vertical line (infinite slope)
        m2 = 0;
        b2 = pt2(2);
    end

    % Check if the lines are not parallel (different slopes)
    if abs(m1 - m2) > tole
        % Calculate the x-coordinate of the intersection point
        xIn = (b2 - b1) / (m1 - m2);

        % Calculate the y-coordinate of the intersection point
        yIn = m1 * xIn + b1;
    elseif abs(m1 - m2) < tole
        % Handle the case when the lines are parallel (no intersection)
        xIn = [];
        yIn = [];
    end
end

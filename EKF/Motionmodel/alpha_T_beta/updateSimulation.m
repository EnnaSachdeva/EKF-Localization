function [] = updateSimulation(params, robot, fig)
% Updates the visualization of the simulation by using data stored in the
% param and robot structs, and displays it in the figure with handle fig.
%
% Inputs:
%   params: parameter struct containing simulation parameters (map, etc.)
%   robot: struct containing robot parameters (pose, covariance, etc.)


% Plot on the figure with handle fig
figure(fig);
cla;

% Plot the map
scatter(params.map(:,1), params.map(:,2), 'g*');

% Plot the robot
hold on;

% Get robot's location
loc = [robot.pose.x; robot.pose.y];
% Plot an isoceles triangle centered at that location
% Length of the triangle
l = 2;
% Vertices of the triangle (centered at the origin)
triangle = [-l/sqrt(2), -l/sqrt(2), l/sqrt(2); l/sqrt(2), -l/sqrt(2), 0];
% Rotate it by the robot's heading, after translating it from the origin to
% the current location (mean)
R = [cos(robot.pose.theta), -sin(robot.pose.theta); ...
    sin(robot.pose.theta), cos(robot.pose.theta)];
triangle = R*triangle + repmat(loc, 1, size(triangle,2));
% Draw the triangle
fill(triangle(1,:), triangle(2,:), [1,0,0]);

% Plot the uncertainty ellipse around the robot
[ellipse_x, ellipse_y] = get_error_ellipse(loc, robot.covariance(1:2,1:2), params.confidenceLevel);
plot(ellipse_x, ellipse_y);


end
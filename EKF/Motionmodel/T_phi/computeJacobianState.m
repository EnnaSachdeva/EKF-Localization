function F = computeJacobianState(pose, control)
% COMPUTEJACOBIANSTATE  Computes the jacobian of the motion model with
% respect to the current state.

% Compute the jacobian F
F = [1,0,-control(1)*sin(pose.theta+control(2));0,1,control(1)*cos(pose.theta+control(2));0,0,1];

end

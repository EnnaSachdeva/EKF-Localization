function G = computeJacobianControl(pose, control)
% COMPUTEJACOBIANSTATE  Computes the jacobian of the motion model with
% respect to the current control.


% Compute the jacobian F
G = [cos(pose.theta+control(2)),-1*control(1)*sin(pose.theta+control(2));sin(pose.theta+control(2)),control(1)*cos(pose.theta+control(2));0,1];

end

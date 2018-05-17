function G = computeJacobianControl(pose, control)
% COMPUTEJACOBIANSTATE  Computes the jacobian of the motion model with
% respect to the current control.

%%% control(1)=alpha, control(2)=T, control(3)=beta

% Compute the jacobian F
G = [1,0;0,1;0,0];

end

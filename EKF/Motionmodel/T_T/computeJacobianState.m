function F = computeJacobianState(pose, control)
% COMPUTEJACOBIANSTATE  Computes the jacobian of the motion model with
% respect to the current state.
%%% control(1)=alpha, control(2)=T, control(3)=beta


% Compute the jacobian F
F = [1,0,0;0,1,0;0,0,1];

end

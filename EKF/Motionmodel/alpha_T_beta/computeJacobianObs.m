function H = computeJacobianObs(pose, z, m)
% COMPUTEJACOBIANOBS  Computes the jacobian of the observation with respect
% to the state. z contains the measurement, and m contains the landmark
% coordinates corresponding to the measurement.

 r=sqrt(((m(1)-pose.x).^2)+((m(2)-pose.y).^2));
% f=z(2);

% Compute the observation jacobian
% H = [-cos(z(3)+pose.theta),-sin(z(3)+pose.theta),0; sin(z(3)+pose.theta)/z(2),-cos(z(3)+pose.theta)/z(2),-1];
 H = [cos(z(3)+pose.theta),sin(z(3)+pose.theta),0; -sin(z(3)+pose.theta)/z(2),cos(z(3)+pose.theta)/z(2),-1];


end

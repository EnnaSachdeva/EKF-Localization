function pose = runMotionModel(pose, control)
% RUNMOTIONMODEL  Takes in the robot's current state and the next control
% action, and predicts the updated pose and uncertainty of the robot's
% state.

%%% control(1)=alpha, control(2)=T, control(3)=beta

% Update the mean
pose.x = pose.x + control(2)*cos(pose.theta + control(1));
pose.y = pose.y + control(2)*sin(pose.theta + control(1));
pose.theta = pose.theta + control(1)+ control(3);

end

function pose = runMotionModel(pose, control)
% RUNMOTIONMODEL  Takes in the robot's current state and the next control
% action, and predicts the updated pose and uncertainty of the robot's
% state.


% Update the mean
pose.x = pose.x + control(1)*cos(pose.theta + control(2));
pose.y = pose.y + control(1)*sin(pose.theta + control(2));
pose.theta = pose.theta + control(2);

end

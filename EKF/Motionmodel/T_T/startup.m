%% Startup script
% Runs automatically when Matlab is run with the current directory as the
% default directory


% Declaring global variables (runs the globals.m script, contained in the
% current directory)
globals;

% Initialize baseDir to hold the root directory of the codebase
baseDir = pwd;

% Directory containing maps
mapDir = fullfile(baseDir, 'maps');

% Directory containing robot control commands
controlDir = fullfile(baseDir, 'controls');

% Initialize a parameter struct to hold important parameters for running
% the code
params = struct;

% Initialize a struct to hold the robot's pose
pose = struct('x', 0.0, 'y', 0.0, 'theta', 0.0);

% Initialize a robot struct that holds its pose
robot = struct;
robot.pose = pose;


% Initialize various configuration files that specify the type of robot,
% sensor, control commands, and the map being used.


% Initialize the control covariance Q and the observation covariance R


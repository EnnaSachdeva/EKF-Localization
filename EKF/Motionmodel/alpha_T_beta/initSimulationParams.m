 function params = initSimulationParams()
% INITSIMULATIONPARAMS  Initializes the simulation parameters for running
% the code. The simulation parameters include the map file (structure of
% the environment) and the initial location of the robot.


% Declaring global variables
globals;

% Initialize the map file to the default map
params.mapFile = fullfile(mapDir, 'default.map');

% Initialize the controls file to the default controls
params.controlFile = fullfile(controlDir, 'default.control');

% Initialize the robot's spawn location to the origin, and heading along
% the positive X-axis
params.spawnPose = [0; 0; 0];


end

%% Script to test simulation


% Use this template for runIncrementalEKF.m and runBatchEKF.m


%% Setting up parameters for running EKF

% Seed Matlab's pseudorandom number generator. This is done to ensure
% repeatability of experiments when random numbers are used.
rng(48302);

% Set simulation parameters (via the param struct). Specifically set the
% map of the environment and the robot's spawn location.
params = initSimulationParams();

% If needed, change the map and the robot's spawn location
params.mapFile = fullfile(mapDir, 'default.map');
params.controlFile = fullfile(controlDir, 'default.control');
params.spawnPose.x = 0;
params.spawnPose.y = 0;
params.spawnPose.theta = 0;
% Confidence level to be used for all uncertainty ellipses
params.confidenceLevel = 0.99;

% Initialize robot parameters (pose, sensing range, uncertainty in state)
robot.pose = params.spawnPose;
robot.sensingRange = 100000;
robot.covariance = diag([0; 0; 0]);

% Read in the map and the controls
params.map = importdata(params.mapFile);
params.controls = importdata(params.controlFile);

% Initialize the control and observation covariances
robot.controlCovariance = diag([0.01, 0.01, 0.00002]);
robot.observationCovariance = diag([0.001, 0.001]);

% Visualize the simulation
fig = figure(1);
updateSimulation(params, robot, fig);


%% Execute control commands and update state using EKF


% Run a loop for executing all control commands
numCommands = size(params.controls,1);
actualPose = robot.pose;
for i = 1:numCommands
    
    
    %% Prediction Step
    
    % Current control command (command that would get executed in an
    % 'ideal' world
    idealControl = params.controls(i,:)';
    
    % TODO: Add noise to the control command, by sampling from a
    % multivariate gaussian with zero mean and specified covariance.
    % actualControl = idealControl + controlNoise;
    
    % TODO: Update the robot's state (mean) using the motion model (where the
    % robot thinks it is)
   
    % TODO: Update the robot's state (mean) using the actual control (where the
    % robot actually is)
    
    % TODO: Compute the jacobian of the motion model, to linearize it
    
    % TODO: Update the robot's state covariance using the linearize model
    
   
    
    % TODO: Fire sensor and make observations
    
    % TODO: From the predicted current position, compute predicted 
    % measurements   
    
    % TODO: Corrupt each measurement by the observation noise
    
    % TODO: Perform an incremental/batch update of the EKF. In the end, you
    % must have the robot's updated state (mean, covariance).
    
    updateSimulation(params, robot, fig);
    pause(0.2);
    
end

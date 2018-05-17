%%%%% RunIncrementalEKF.m
%% Script to test simulation
clear all;
close all;
clc;

format long;

% Use this template for runIncrementalEKF.m and runBatchEKF.m


%% Setting up parameters for running EKF

% Seed Matlab's pseudorandom number generator. This is done to ensure
% repeatability of experiments when random numbers are used.
rng(48302);

% Set simulation parameters (via the param struct). Specifically set the
% map of the environment and the robot's spawn location.
params = initSimulationParams();

% If needed, change the map and the robot's spawn location
params.mapFile = fullfile('default.map');   %% changed
params.controlFile = fullfile('default.control'); %% changed
%  params.mapFile = fullfile('random_small.map');   %% changed
%  params.controlFile = fullfile('random_small.control'); %% changed
params.spawnPose.x = 0;
params.spawnPose.y = 0;
params.spawnPose.theta = 0;

% Confidence level to be used for all uncertainty ellipses
params.confidenceLevel = 0.9;

% Initialize robot parameters (pose, sensing range, uncertainty in state)
robot.pose = params.spawnPose;
robot.sensingRange = 100000;
robot.covariance = diag([0; 0; 0]);

% Read in the map and the controls
params.map = importdata(params.mapFile);
params.controls = importdata(params.controlFile);

% Initialize the control and observation covariances
robot.controlCovariance = diag([0.5,0.5]);  %%% delta_x,delta_y
robot.observationCovariance = diag([10, 1]);

% current_pose{1} = [ 0 , 0];
% Visualize the simulation
fig = figure(1);
%position{1}=[0,0];

updateSimulation(params, robot, fig);

fig_x=figure(2);  %%%    for error plot mu_x

fig_y=figure(3); %%%     for error plot mu_y

fig_theta=figure(4); %%% for error plot mu_theta


%% Execute control commands and update state using EKF


% Run a loop for executing all control commands
numCommands = size(params.controls,1);

for m = 1:numCommands
    
    actualPose = robot.pose; %% mu_t
    %% Prediction Step 
%    current_pose{m}=[robot.pose.x,robot.pose.y]; 
    % Current control command (command that would get executed in an
    % 'ideal' world
    idealControl = params.controls(m,:);
    
    % TODO: Add noise to the control command, by sampling from a
    % multivariate gaussian with zero mean and specified covariance.
    % actualControl = idealControl + controlNoise;
     
     actualControl= idealControl + sane_mvnrnd(zeros(size(idealControl,2),1),robot.controlCovariance,params.confidenceLevel,1); 
%    actualControl= sane_mvnrnd(idealControl',robot.controlCovariance,params.confidenceLevel,1);
    
 
    % TODO: Update the robot's state (mean) using the motion model (where the
    % robot thinks it is)
    
    thinkPose=runMotionModel(actualPose, idealControl);  %%%%% mu_hat_t+1
           
    % TODO: Update the robot's state (mean) using the actual control (where the
    % robot actually is)
    
    newactualPose=runMotionModel(actualPose, actualControl);   
%     position{m}=[newactualPose.x,newactualPose.y]; 
    % TODO: Compute the jacobian of the motion model, to linearize it
       
    F = computeJacobianState(actualPose, actualControl);    %%%%% actualPose at time t
    G = computeJacobianControl(actualPose, actualControl);  %%%%% actualPose at time t
        
    % TODO: Update the robot's state covariance using the linearize model
    
    a=robot.covariance;
    robot.covariance=(F*robot.covariance*F')+(G*robot.controlCovariance*G');  %%% sigma_hat_t+1
    
%     if (det(a)<=det(robot.covariance))
%        display('TRUE')
%     else display('FALSE')
%     end
        
    % TODO: Fire sensor and make observations
    
    sensor_measurements = getMeasurements(newactualPose, params, robot.sensingRange); %%%%% z_t+1 using info about actual pose at time t, from the map 
%     sensor_measurements  
   % TODO: From the predicted current position, compute predicted 
   % measurements   
    
    pred_measurements = getMeasurements(thinkPose, params, robot.sensingRange);  %%% z_hat_t+1 %%% not sure about this
   
    % TODO: Corrupt each measurement by the observation noise
    
    sensor_measurements(:,(2:size(sensor_measurements,2)))=sensor_measurements(:,(2:size(sensor_measurements,2)))+sane_mvnrnd(zeros(size(sensor_measurements,2)-1,1),robot.observationCovariance,params.confidenceLevel,size(sensor_measurements,1));

    % TODO: Perform an incremental/batch update of the EKF. In the end, you
    % must have the robot's updated state (mean, covariance).
    
    %%%%%% Incremental Mode
   
    for p=1:size(sensor_measurements,1)

        H = computeJacobianObs(thinkPose, pred_measurements(p,:), params.map(p,:));
        S = (H*robot.covariance*H')+robot.observationCovariance; 
        K = robot.covariance*H'/S;   %%% faster method to calculate A*inv(B) = A/B
        inc= (sensor_measurements(p,2:size(sensor_measurements,2))-pred_measurements(p,2:size(sensor_measurements,2)))*K';
        robot.pose.x = thinkPose.x+ inc(1);          %% updated mu_t+1
        robot.pose.y = thinkPose.y+ inc(2);          %% updated mu_t+1
        robot.pose.theta = thinkPose.theta+ inc(3);  %% updated mu_t+1
        robot.covariance=(eye(size(K*H))-K*H)*robot.covariance;  % updated sigma_t+1
%         robot.covariance=(eye(size(K*H))-K*H)*robot.covariance*(eye(size(K*H))-K*H)' + K*robot.observationCovariance*K';%joseph form of covariance update
%       robot.covariance=0.5*(robot.covariance+robot.covariance');
             
    end
    
    %%%%% localization error: abs(mu_hat − mu_true )
     error(m,:)= [abs(thinkPose.x-newactualPose.x);...
                  abs(thinkPose.y-newactualPose.y);...
                  abs(thinkPose.theta-newactualPose.theta)];

     
      robot.covariance=0.5*(robot.covariance+robot.covariance'); %%%%% making covariance matrix symmetric 
%       updateSimulation(params, robot, fig, position);
    updateSimulation(params, robot, fig);
    
   
    
pause(0.2);
    
end
x_axis=1:1:numCommands;
perfAnalysis(x_axis,error,fig_x, fig_y,fig_theta);





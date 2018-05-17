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
 
% params.mapFile = fullfile('default.map');   %% changed
% params.controlFile = fullfile('default.control'); %% changed
params.mapFile = fullfile('random_small.map');   %% changed
 params.controlFile = fullfile('random_small.control'); %% changed
% params.controlFile = fullfile('smooth_circle.control');
params.spawnPose.x = 0;
params.spawnPose.y = 0;
params.spawnPose.theta = 0;
 
% Confidence level to be used for all uncertainty ellipses
params.confidenceLevel = 0.7;
 
% Initialize robot parameters (pose, sensing range, uncertainty in state)
robot.pose = params.spawnPose;
robot.sensingRange = 10000;
robot.covariance = diag([0; 0; 0]);
 
% Read in the map and the controls
params.map = importdata(params.mapFile);
params.controls = importdata(params.controlFile);
 
% Initialize the control and observation covariances
robot.controlCovariance = diag([0.5, 0.002]);
robot.observationCovariance = diag([10, 1]);
%  
% robot.controlCovariance = diag([10, 1]);
% robot.observationCovariance = diag([0.5, 0.02]);
 
% current_pose{1} = [ 0 , 0];
% Visualize the simulation
fig = figure(1);
% position{1}=[0,0];
% title('Robot trajectory')
% xlabel('mu_x ') % x-axis label
% ylabel('mu_y') % y-axis label
% legend('Robot in 2D plane')
% hold on
% updateSimulation(params, robot, fig, position);
 
updateSimulation(params, robot, fig);
 
fig_x=figure(2);  %%%    for error plot mu_x
 
fig_y=figure(3); %%%     for error plot mu_y
 
fig_theta=figure(4); %%% for error plot mu_theta
 
 
%% Execute control commands and update state using EKF
 
 
% Run a loop for executing all control commands
numCommands = size(params.controls,1);
 actualPose = robot.pose; %% mu_t
for m = 1:numCommands
   updatePose = robot.pose;  
    
    %% Prediction Step 
%    current_pose{m}=[robot.pose.x,robot.pose.y]; 
    % Current control command (command that would get executed in an
    % 'ideal' world
    idealControl = params.controls(m,:);
     
    % TODO: Add noise to the control command, by sampling from a
    % multivariate gaussian with zero mean and specified covariance.
    % actualControl = idealControl + controlNoise;
      
     actualControl= idealControl + sane_mvnrnd(zeros(size(idealControl,2),1),robot.controlCovariance,params.confidenceLevel,1) ;
%    actualControl= sane_mvnrnd(idealControl',robot.controlCovariance,params.confidenceLevel,1);
     
  
    % TODO: Update the robot's state (mean) using the motion model (where the
    % robot thinks it is)
     
    predPose=runMotionModel(updatePose, idealControl);  %%%%% mu_hat_t+1
     
    % TODO: Update the robot's state (mean) using the actual control (where the
    % robot actually is)
     
    actualPose=runMotionModel(updatePose, actualControl); 
%     actual(m,:)=[newactualPose.x newactualPose.y newactualPose.theta];
%     position{m}=[newactualPose.x,newactualPose.y]; 
    % TODO: Compute the jacobian of the motion model, to linearize it
        
    F = computeJacobianState(actualPose, idealControl);    %%%%% actualPose at time t
    G = computeJacobianControl(updatePose, idealControl);  %%%%% actualPose at time t
         
    % TODO: Update the robot's state covariance using the linearize model
     
     
    robot.covariance=(F*robot.covariance*F')+(G*robot.controlCovariance*G');  %%% sigma_hat_t+1
     
%     if (det(a)<=det(robot.covariance))
%        display('TRUE')
%     else display('FALSE')
%     end
         
    % TODO: Fire sensor and make observations
     
    sensor_measurements = getMeasurements(actualPose, params, robot.sensingRange); %%%%% z_t+1 using info about actual pose at time t, from the map 
%     sensor_measurements  
   % TODO: From the predicted current position, compute predicted 
   % measurements   
     
    pred_measurements = getMeasurements(predPose, params, robot.sensingRange);  %%% z_hat_t+1 %%% not sure about this
    
    % TODO: Corrupt each measurement by the observation noise
     
    sensor_measurements(:,(2:size(sensor_measurements,2)))=sensor_measurements(:,(2:size(sensor_measurements,2)))+sane_mvnrnd(zeros(size(sensor_measurements,2)-1,1),robot.observationCovariance,params.confidenceLevel,size(sensor_measurements,1));
     
    % back calculating the robot state from the noisy measurements
 
     
    % TODO: Perform an incremental/batch update of the EKF. In the end, you
    % must have the robot's updated state (mean, covariance).
     
    %%%%%% Incremental Mode
         
    
    for p=1:size(sensor_measurements,1)
 
         
        H = computeJacobianObs(predPose, pred_measurements(p,:), params.map(p,:));
        S = (H*robot.covariance*H')+robot.observationCovariance; 
        K = robot.covariance*H'/S;   %%% faster method to calculate A*inv(B) = A/B
        inc= (sensor_measurements(p,2:size(sensor_measurements,2))-pred_measurements(p,2:size(sensor_measurements,2)))*K';
        robot.pose.x = predPose.x+ inc(1);          %% updated mu_t+1
        robot.pose.y = predPose.y+ inc(2);          %% updated mu_t+1
        robot.pose.theta = predPose.theta+ inc(3);  %% updated mu_t+1
        robot.covariance=(eye(size(K*H))-K*H)*robot.covariance;  % updated sigma_t+1
%       robot.covariance=(eye(size(K*H))-K*H)*robot.covariance*(eye(size(K*H))-K*H)' + K*robot.observationCovariance*K';%joseph form of covariance update
%        robot.covariance=0.5*(robot.covariance+robot.covariance');
        predPose.x=robot.pose.x;          
        predPose.y=robot.pose.y;          
        predPose.theta=robot.pose.theta;  
                      
 
 
    end
     
    %%%%% localization error: abs(mu_hat − mu_true )
%      error(m,:)= [abs(predPose.x-newactualPose.x);...
%                   abs(predPose.y-newactualPose.y);...
%                   abs(predPose.theta-newactualPose.theta)];
%  
%       
%               %%%%% localization error: abs(mu_hat − mu_true )
%      error1(m,:)= [abs(predPose.x-robot.pose.x);...
%                   abs(predPose.y-robot.pose.y);...
%                   abs(predPose.theta-robot.pose.theta)];
%  
      robot.covariance=0.5*(robot.covariance+robot.covariance'); %%%%% making covariance matrix symmetric 
%       updateSimulation(params, robot, fig, position);
    updateSimulation(params, robot, fig);
     
    
     
%     update(m,:)=[robot.pose.x  robot.pose.y robot.pose.theta];
     
pause(0.2);
  
end
 
 
% a=[think  update]
 
% x_axis=1:1:numCommands;
% perfAnalysis(x_axis,update,actual,think, fig_x,fig_y,fig_theta);
 
%%%% The final result will be that the EKF will reduce the error of the
%%%% difference between the predicted state and the state that will be back
%%%% calculated, given sensor measurement(with noise), which is not calculated here since, given noisy
%%%% sensor data we can not measure the mu_x,mu_y,mu_theta.( 2equations, 3
%%%% unknowns). Hence, EKF updated pose value will tend to be close to the
%%%% value (back calculated from sensors' measurements).

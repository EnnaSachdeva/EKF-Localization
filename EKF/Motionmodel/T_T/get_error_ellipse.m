function [ellipse_x, ellipse_y] = get_error_ellipse(mu_hat, sigma_hat, confidence)
% GET_ERROR_ELLIPSE  Plots the error ellipse for a two dimensional
% gaussian distribution, given the mean, covariance, and the confidence
% level for which the ellipse is to be plotted.
%
% Inputs:
%   mu_hat: 2-by-1 mean vector
%   sigma_hat: 2-by-2 covariance matrix
%   confidence: value in the range (0,1), denoting the confidence level for
%   which the error ellipse is to be plotted
% Outputs:
%   ellipse_x: X coordinates of the error ellipse
%   ellipse_y: Y coordinates of the error ellipse
% 
% This code closely follows the method outlined in
% http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/


% We're only dealing with 2D gaussians here
numDims = 2;

% TODO: check if avg is a 2-by-1 vector. Print an error message and return
% if check fails.
if size(mu_hat,1) ~=2 || size(mu_hat,2) ~= 1
    disp('Mean must be a 2-by-1 vector');
end

% TODO: check if covariane is a 2-by-2 matrix, and if it is symmetric, and
% if its determinant is < 0. Print an error message and return if the check 
% fails.
if size(sigma_hat,1) ~= size(sigma_hat,2)
    disp('Covariance must be a square matrix');
    return;
end

if ~issymmetric(sigma_hat)
    disp('Covariance matrix must be symmetric');
    return;
end

if size(sigma_hat,1) ~= numDims
    disp('Mean and covariance must be of compatible dimensions');
    return;
end

if det(sigma_hat) < 0
    disp('Covariance matrix must be Positive Semi-Definite');
    return;
end

% TODO: compute the eigenvalues and eigenvectors of the ellipse. Store the
% the largest eigenvalue and the smallest eigenvalue in the
% variable lambda_large and lambda_small respectively.
[eigVec, eigVal] = eig(sigma_hat);
lambda_large = 1;
lambda_small = 1;
eigVec_large = eigVec(:,1);
eigVec_small = eigVec(:,2);
if eigVal(1,1) >= eigVal(2,2)
    lambda_large = eigVal(1,1);
    lambda_small = eigVal(2,2);
    eigVec_large = eigVec(:,1);
    eigVec_small = eigVec(:,2);
else
    lambda_large = eigVal(2,2);
    lambda_small = eigVal(1,1);
    eigVec_large = eigVec(:,2);
    eigVec_small = eigVec(:,1);
end

% Compute the angle between the largest and the smallest eigenvector
phi = atan2(eigVec_large(2), eigVec_large(1));
% Get it in the range [0, 2*pi) as opposed to (-pi, pi]
if phi < 0
    phi = phi + 2*pi;
end

% TODO: Generate 100 uniformly spaced samples in the range 0 to 2*pi (i.e., 
% with a pi/50 increment). You'll have to modify the below line. The below
% line generates a vector of 100 zeros. Instead, generate a vector ranging
% from values 0 to 2*pi (2*pi excluded, since it is the same as 0).
theta = 0:pi/50:(2*pi)-(pi/50);

% TODO: Determine the value of the mahalanobis distance at the required
% confidence level and store it in the variable mahalanobisThresh. Note use
% chi2inv for the same. Do not 'hard' initialize it to a constant.
mahalanobisThresh = chi2inv(confidence, numDims);

% TODO: Ensure that the variables lambda_large, lambda_small, theta, and
% mahalanobisThresh are set appropriately.
ellipse_x = mahalanobisThresh*sqrt(lambda_large)*cos(theta);
ellipse_y = mahalanobisThresh*sqrt(lambda_small)*sin(theta);

% Define the rotation matrix to rotate the ellipse by phi
R = [cos(phi), -sin(phi); sin(phi), cos(phi)];

% Perform the rotation
rotatedEllipse = repmat([mu_hat(1); mu_hat(2)], 1, size(ellipse_x,2)) + R*[ellipse_x; ellipse_y];
ellipse_x = rotatedEllipse(1,:);
ellipse_y = rotatedEllipse(2,:);


end
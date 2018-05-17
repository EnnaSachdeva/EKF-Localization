function R = sane_mvnrnd(avg, sigma, alpha, numSamples)
% SANE_MVNRND  'Sane' random vectors from the multivariate normal
% distribution.
%   R = SANE_MVNRND(AVG, SIGMA, ALPHA, NUMSAMPLES) returns an N-by-D matrix 
%   R of random vectors from the multivariate normal distribution with mean
%   vector AVG and covariance matrix SIGMA, such that all samples returned
%   will be contained in the ALPHA-confidence ellipse of the distribution.
%   ALPHA is a variable indicating confidence level. ALPHA typically lies
%   in the range [0,1], but the boundary values are not of significance, so
%   ALPHA for all practical purposes is in the range (0,1).
%
%   Inputs:
%       avg: D-by-1 mean vector
%       sigma: D-by-D covariance matrix
%       alpha: scalar confidence level (0 < alpha < 1)
%       numSamples: number of samples to be drawn
%   Output:
%       R: N-by-D matrix of random vectors



% Get the number of dimensions of the multivariate gaussian
numDims = size(avg,1);

% Initialize R to be an N-by-D zero matrix
R = zeros(numSamples, numDims);

% TODO: Check if the dimensions of the mean vector and covariance matrix
% are compatible. Print an error message and return if incompatible.
if size(sigma,1) ~= size(sigma,2)
    disp('Covariance must be a square matrix');
    return;
    if issymmetric(sigma)
        disp('Covariance matrix must be symmetric');
        return;
    end
    if size(sigma,1) ~= numDims
        disp('Mean and covariance must be of compatible dimensions');
        return;
    end
    if det(sigma) < 0
        disp('Covariance matrix must be Positive Semi-Definite');
        return;
    end
end

% TODO: Check if alpha lies in the range (0,1) (note the open interval).
% Print an error message and return if check fails.
if alpha <= 0 || alpha >= 1
    disp('Confidence level (alpha) must be in the range (0,1)');
    return;
end

% TODO: Look up the confidence level in the inverse chi squared table, and
% retrieve the Mahalanobis distance threshold. Hint: use the chi2inv
% function. For details type 'help chi2inv' in the command window.
mahalanobisThresh = chi2inv(alpha, numDims);

% Note that the following is a simple, but possibly inefficient strategy to
% generate large numbers of samples. Feel free to innovate here.
% TODO: Run a loop for 'numSamples' iterations. In each iteration of the
% loop, keep sampling from mvnrnd and compute the Mahalanobis norm for the
% sample until you get atleast one sample whose Mahalanobis norm is less
% than the threshold computed from the inverse chi squared lookup table.
for i = 1:numSamples
    isWithinThresh = false;
    curSample = zeros(size(avg));
    while ~isWithinThresh
        curSample = mvnrnd(avg, sigma, 1);
        mahalDist = sqrt((curSample'-avg)'*inv(sigma)*(curSample'-avg));
        if mahalDist <= mahalanobisThresh
            isWithinThresh = true;
        else
            isWithinThresh = false;
        end
    end
    R(i,:) = curSample';
end


end
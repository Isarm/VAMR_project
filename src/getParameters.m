function parameters = getParameters(ds)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Dataset Selection
% 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % TODO
    parameters = 0;
elseif ds == 1
    % TODO
    parameters = 0;
elseif ds == 2
    % Bootstrapping
    parameters.bootstrapFrame1 = 0;
    parameters.bootstrapFrame2 = 3;
    
    % Harris Features
    parameters.MinQuality = 2e-5; % Bootstrapping
    parameters.MinQualityC = 1e-4; % Continuous
    parameters.FilterSize = 9;
    
    % KLT
    parameters.MaxBidirectionalError = 3;
    parameters.MaxIterations = 50;
    parameters.BlockSize = [21 21];
    
    % P3P
    parameters.MaxNumTrials = 25000;
    parameters.Confidence = 99; % Default = 99
    parameters.MaxReprojectionError = 2;
    
    % Triangulation
    parameters.alpha = 6 * pi / 180; % Radians
end
end


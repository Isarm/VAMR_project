function parameters = getParameters()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Bootstrapping
parameters.bootstrapFrame1 = 1;
parameters.bootstrapFrame2 = 4;

% Harris Features
parameters.MinQuality = 1e-4; % Bootstrapping
parameters.MinQualityC = 1e-4; % Continuous
parameters.FilterSize = 3;
parameters.tolerance = 1;
parameters.HarrisMaxFeatures = 300;
parameters.HarrisMaxSuppression = 15;

% KLT
parameters.NumPyramidLevels = 3;
parameters.MaxBidirectionalError = 1;
parameters.MaxIterations = 2000;
parameters.BlockSize = [31 31];

% P3P
parameters.MaxNumTrials = 25000;
parameters.Confidence = 99.99; % Default = 99
parameters.MaxReprojectionError = 4;

% Triangulation
parameters.alpha = 4 * pi / 180; % Radians
parameters.MaxReprojectionErrorTriangulate = 8;
end

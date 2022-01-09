function parameters = getParameters(ds)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Dataset Selection
% 0: KITTI, 1: Malaga, 2: parking 3: custom 4: custom wide

if ds == 0
    % Bootstrapping
    parameters.bootstrapFrame1 = 1;
    parameters.bootstrapFrame2 = 5;
    
    % Harris Features
    parameters.MinQuality = 1e-4; % Bootstrapping
    parameters.MinQualityC = 1e-5; % Continuous
    parameters.FilterSize = 7;
    parameters.tolerance = 2;
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

elseif ds == 1
    % Bootstrapping
    parameters.bootstrapFrame1 = 1;
    parameters.bootstrapFrame2 = 5;
    
    % Harris Features
    parameters.MinQuality = 2e-5; % Bootstrapping
    parameters.MinQualityC = 1e-7; % Continuous
    parameters.FilterSize = 15;
    parameters.tolerance = 2;
    parameters.HarrisMaxFeatures = 200;
    parameters.HarrisMaxSuppression = 15;

    % KLT
    parameters.NumPyramidLevels = 3;
    parameters.MaxBidirectionalError = 0.6;
    parameters.MaxIterations = 50;
    parameters.BlockSize = [31 31];
    
    % P3P
    parameters.MaxNumTrials = 25000;
    parameters.Confidence = 99; % Default = 99
    parameters.MaxReprojectionError = 2;
    
    % Triangulation
    parameters.alpha = 5 * pi / 180; % Radians
    parameters.MaxReprojectionErrorTriangulate = 3;

elseif ds == 2
    % Bootstrapping
    parameters.bootstrapFrame1 = 1;
    parameters.bootstrapFrame2 = 5;
    
    % Harris Features
    parameters.MinQuality = 2e-5; % Bootstrapping
    parameters.MinQualityC = 1e-3; % Continuous
    parameters.FilterSize = 9;
    parameters.tolerance = 1;
    parameters.HarrisMaxFeatures = 200;
    parameters.HarrisMaxSuppression = 15;

    % KLT
    parameters.NumPyramidLevels = 3;
    parameters.MaxBidirectionalError = 3;
    parameters.MaxIterations = 50;
    parameters.BlockSize = [31 31];
    
    % P3P
    parameters.MaxNumTrials = 25000;
    parameters.Confidence = 99; % Default = 99
    parameters.MaxReprojectionError = 2;
    
    % Triangulation
    parameters.alpha = 6 * pi / 180; % Radians
    parameters.MaxReprojectionErrorTriangulate = 3;

elseif ds == 3
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

elseif ds == 4
    % Bootstrapping
    parameters.bootstrapFrame1 = 20;
    parameters.bootstrapFrame2 = 22;
    
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
end

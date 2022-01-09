%% testTrackPoints
% this script tests point tracking using KLT
params = getParameters();

% get first images of dataset, as well as camera instrinsics
[img1, img2, intrinsics] = getInitialFrames(0, 0, 2);

% Extract ORB features in first frame, to give us an initial set of points to track
[~, img1Points] = getHarrisFeatures(img1, params);

% track points
[img2Points, validity] = trackPoints(img1, img2, img1Points, parameters);

% get matched points from first image
img1Points = img1Points(validity, :);

% compare 
figure; showMatchedFeatures(img1, img2, img1Points, img2Points);

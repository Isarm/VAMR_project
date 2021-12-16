%% testTrackPoints
% this script tests point tracking using KLT

% get first images of dataset, as well as camera instrinsics
[img1, img2, intrinsics] = getInitialFrames(0, 0, 2);

% initialization/bootstrap, get initial translation, rotation,
% orb features of matches points, and landmarks
% [keyPoints, landmarks3D, R, T] = bootstrap(img1, img2, intrinsics);

% Extract ORB features in first frame, to give us an initial set of points to track
[~, img1PointStruct] = getORBFeatures(img1);
img1Points = img1PointStruct.Location;

% track points
[img2Points, validity] = trackPoints(img1, img2, img1Points);

% get matched points from first image
img1Points = img1Points(validity, :);

% compare 
figure; showMatchedFeatures(img1, img2, img1Points, img2Points);

% build state of the second frame which was used during initialization (S_j)
% must tranpose first two since given as row vectors from bootstrap function
S_j = struct;
S_j.P = keyPoints';     % 2xK keypoint pixel locations
S_j.X = landmarks3D';    % 3xK 3D landmarks
S_j.C = [];             % no candidate keypoints (yet)
S_j.F = [];             % no candidate keypoints (yet)
S_j.T = [];             % no candidate keypoints (yet)

%[S_i, T_WC_i] = processFrame(img2, S_j, intrinsics);
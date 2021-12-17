%% Tests processFrame
% Can be used to step through the processFrame function

% get first images of dataset, as well as camera instrinsics
[img1, img2, intrinsics] = getInitialFrames(0, 0, 2);

[keyPoints, landmarks3D, R, T] = bootstrap(img1, img2, intrinsics);

S = struct;

S.P = keyPoints;
S.X = landmarks3D;
S.C = [];
S.F = [];
S.T = [];

% Reusing this because why not
[img3, ~, ~] = getInitialFrames(0, 3, 0);


[S, T] = processFrame(img3, img2, S, intrinsics);
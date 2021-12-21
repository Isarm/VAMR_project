%% Tests processFrame
% Can be used to step through the processFrame function

% get first images of dataset, as well as camera instrinsics
SECOND = 2;
[img1, img2, intrinsics] = getInitialFrames(0, 0, SECOND);

[keyPoints, landmarks3D, R, T] = bootstrap(img1, img2, intrinsics);

S = struct;

S.P = keyPoints;
S.X = landmarks3D;
S.C = [];
S.F = [];
S.T = [];

i = SECOND + 1;

while true
    img1 = img2;
    [img2, ~, ~] = getInitialFrames(0, i, 0);
%     imshow(img2);
    [S, T] = processFrame(img1, img2, S, intrinsics);
    T
    i = i + 1
end

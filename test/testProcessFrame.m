%% Tests processFrame
% Can be used to step through the processFrame function

numFrames = 20;
% x and z since matlab uses x and y for the horizontal plane, and y for up
% and down
[fig, topViewLandmarksX, topViewLandmarksZ, topViewCarX, topViewCarZ] = createFigure(50, numFrames);
topViewLastCell = 1;

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
    % I need to track both S1 and S2 temporarily to find
    % the new 3D landmark points only
    [S2, T] = processFrame(img1, img2, S, intrinsics);
    T;
    i = i + 1;

    P1 = S.P;
    P2 = S2.P;
    [row1,~] = size(P1);
    [row2,~] = size(P2);
    numNewKeypoints = row2 - row1;

    X1 = S.X;
    X2 = S2.X;
    [row1,~] = size(X1);
    [row2,~] = size(X2);
    numNewLandmarks = row2 - row1;


    if numNewLandmarks > 0
        topViewLandmarksX{topViewLastCell} = X2(end-numNewLandmarks+1:end,1)';
        topViewLandmarksZ{topViewLastCell} = X2(end-numNewLandmarks+1:end,3)';
        topViewCarX{topViewLastCell} = T(1,4); % x
        topViewCarZ{topViewLastCell} = T(3,4); % z
        topViewLastCell = mod(topViewLastCell + 1, numFrames+1);
        if topViewLastCell == 0
            topViewLastCell = 1;
        end
    end

    updateFigure(fig, img2, P2, row2,...
                [T(1,4),T(3,4)], topViewLandmarksX, topViewLandmarksZ, ...
                topViewCarX, topViewCarZ);
    S = S2;
end

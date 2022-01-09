clc
clear
close all

%% Tests processFrame
% Can be used to step through the processFrame function
ds = 0;

parameters = getParameters(ds);

% Get Ground Truth Poses for Parking Datatset
if ds == 2
    fid = fopen('data/parking/poses.txt') ;  % open the text file
    rawPoses = textscan(fid, '%f');   % text scan the data
    fclose(fid);      % close the file
    groundTruth = reshape(rawPoses{1}, 12, [])';
    groundTruthPose = groundTruth(:,[4,8,12]);
else
    groundTruthPose = [];
end

numFrames = 20;
% x and z since matlab uses x and y for the horizontal plane, and y for up
% and down
[fig, topViewLandmarksX, topViewLandmarksZ, topViewCarX, topViewCarZ] = createFigure(50, numFrames, ds);
topViewLastCell = 1;

% get first images of dataset, as well as camera instrinsics
[img1, img2, intrinsics] = getInitialFrames(ds, parameters.bootstrapFrame1, parameters.bootstrapFrame2);

[keyPoints, landmarks3D, R, T] = bootstrap(img1, img2, intrinsics, parameters);

% Bundle Adjustment
ba = struct;
ba.vSet = imageviewset;
ba.window = 0; % Set to 0 to turn off bundle adjustment
ba.cameraPoses.ViewId = [];
ba.cameraPoses.Orientation = {};
ba.cameraPoses.Location = {};
ba.xyzPoints = {}; % Organized by frame
ba.xyzPointsRefined = {};
ba.sizexyzPoints = [];

ba.refinedPoses = []; 
ba.reprojectionErrors = [];

if ba.window
    % Note: FREAK used because extractFeatures cornerPoint objects
    % default uses FREAK method
    [F,~] = extractFeatures(img2, keyPoints, 'Method', 'FREAK');
    ba.vSet = addView(ba.vSet,uint32(parameters.bootstrapFrame2),'Features',F,'Points', keyPoints);
    ba.xyzPoints = [ba.xyzPoints; landmarks3D];
    ba.xyzPointsRefined = [ba.xyzPointsRefined; landmarks3D];
    ba.sizexyzPoints = [ba.sizexyzPoints; size(landmarks3D,1)];
    % TODO: When parameters.bootstrapFrame2 is not equal to 1
    ba.cameraPoses.ViewId = [ba.cameraPoses.ViewId; uint32(parameters.bootstrapFrame2)];
    ba.cameraPoses.Orientation = [ba.cameraPoses.Orientation; R]; % Relative Camera Pose
    ba.cameraPoses.Location = [ba.cameraPoses.Location; T]; % Relative Camera Pose
    ba.cameraPoseWC_i.Orientation = {R};
    ba.cameraPoseWC_i.Location = {T};

end

S = struct;

S.P = keyPoints;
S.X = landmarks3D;
S.C = [];
S.F = [];
S.T = [];

i = parameters.bootstrapFrame2 + 1;

while true
    img1 = img2;
    img2 = getFrame(ds, i);
%     imshow(img2);
    % I need to track both S1 and S2 temporarily to find
    % the new 3D landmark points only
    [S2, T, ba] = processFrame(img1, img2, S, intrinsics, parameters, ba, i);
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

    % TODO: FIX THIS CUS THIS DOES NOT MAKE SENSE
    topViewLandmarksX{topViewLastCell} = single(X2(:,1)');
    topViewLandmarksZ{topViewLastCell} = single(X2(:,3)');
    topViewCarX(i) = T(1,4); % x
    topViewCarZ(i) = T(3,4); % z
    topViewLastCell = mod(topViewLastCell + 1, numFrames+1);
    if topViewLastCell == 0
        topViewLastCell = 1;
    end

    updateFigure(fig, img2, i, P2, S2.C, row2,...
                [T(1,4),T(3,4)], topViewLandmarksX, topViewLandmarksZ, ...
                topViewCarX, topViewCarZ, ba, groundTruthPose);
    S = S2;
    
    pause(0.1)
end

function [S_2,T_WC_i, ba] = processFrame(I_1, I_2, S_1, intrinsics, parameters, ba, i)
%processFrame Process incoming frames in the continous VO pipeline
% Summary: 
%   Takes as input the incoming (current) ith frame, the previous i-1th
%   frame, and the state of the previous i-1th frame. Returns the updated
%   state for the current frame and the pose of the current frame. Relies
%   the Markov property as the state of the current frame is only dependent
%   on the state of the previous frame (and the new image).
%   
%   Uses KLT + RANSAC to track keypoints in previous frame to current frame
%   and associates tracked keypoints with exisiting landmarks.
% 
%   Uses P3P + RANSAC to estimate pose of current frame from new 2D-3D
%   point correspondences from previous step.
%
%   Maintains a set of candidate keypoints and triangulates new 3D
%   landmarks when possible. 

% Inputs: 
%   I_2         = Image of the ith (current) frame
%   I_1         = Image of the i-1th (previous) frame
%   S_1         = State of the i-1th (previous) frame
%   intrinsics  = cameraIntrinsics object

% Outputs: 
%   S_2       = State of the ith (current) frame
%   T_WC_i    = Pose of the ith (current) frame 

%% Setup
P = S_1.P; % Set of keypoints in the i-1th (previous) frame (Kx2)
X = S_1.X; % Set of landmarks in the i-1th (previous) frame (Kx3)
C = S_1.C; % Set of candidate keypoints in the i-1th (previous) frame (Mx2)
F = S_1.F; % Set of first observation of each candidate keypoints in the i-1th (previous) frame (Mx2)
T = S_1.T; % Set of camera poses for first observation of each candidate keypoints in the i-1th (previous) frame (Mx16)

% K_j = size(P_j,2); % Number of keypoints in the i-1th (previous) frame
% M_j = size(C_j,2); % Number of candidate keypoints in the i-1th (previous) frame

% Initialize state of current frame
S_2 = struct;

%% Associate Keypoints to Existing Landmarks
% use KLT and RANSAC to track keypoints
[P, validity] = trackPoints(I_1, I_2, P, parameters);

% Update landmark set by only keeping corresponding landmarks of reliably
% tracked keypoints (RANSAC)
X = X(validity, :);

%% Estimate the Current Pose
[worldOrientation,worldLocation,ransac_inlier_ids] = ...
        estimateWorldCameraPose(P, X, intrinsics, ...
        'MaxNumTrials', parameters.MaxNumTrials, ...
        'Confidence', parameters.Confidence, ...
        'MaxReprojectionError', parameters.MaxReprojectionError);

T_WC_i = [worldOrientation', worldLocation'];
T_WC_i(4, 4) = 1; % Make it a homogeneous transformation matrix

% Delete points that are not inliers
P = P(ransac_inlier_ids, :);
X = X(ransac_inlier_ids, :);

%% Track Candidate Keypoints
[C, validityCandidate] = trackPoints(I_1, I_2, C, parameters);
% remove lost candidate point data / retain matched candidate point data
F = F(validityCandidate, :);
T = T(validityCandidate, :);

%% Triangulating New Landmarks
[P_new, X_new, remove] = getNewLandmarks(F, C, T, T_WC_i, intrinsics, parameters);

F(remove, :) = [];
C(remove, :) = [];
T(remove, :) = [];

P = [P ; P_new];
X = [X ; X_new];

%% Bundle Adjustment
if ba.window
    ba = doBundleAdjustment(ba, i, I_2, intrinsics, T_WC_i, ...
        P, X, validity, ransac_inlier_ids);
end

%% Find the features in the new image and update the state
% N = getSIFTFeatures(I_2);
[~,N] = getHarrisFeatures(I_2, parameters);
N = N.Location;

% Check if we have redected features that we are already tracking.
tolerance = parameters.tolerance; % tolerance pixel values
inliers = ismembertol(N, [P; C], tolerance, "ByRows", true, 'DataScale', [1 1]);
N(inliers, :) = []; % Remove redetected features

% Add the new points to the set of candidate points
C = [C ; N];
% Add the new points to the set of first saw points
F = [F ; N];
% Update the set for first observation for each candidate keypoint
T = [T ; repmat(T_WC_i(:)', [size(N, 1), 1])];

%% Store Local Variables in State of Current Frame
S_2.P = P; % Set of keypoints in the ith (current) frame (K'x2)
S_2.X = X; % Set of landmarks in the ith (current) frame (K'x3)
S_2.C = C; % Set of candidate keypoints in the ith (previous) frame (M'x2)
S_2.F = F; % Set of first observation of each candidate keypoints in the ith (current) frame (M'x2)
S_2.T = T; % Set of camera poses for first observation of each candidate keypoints in the ith (current) frame (M'x16)

end


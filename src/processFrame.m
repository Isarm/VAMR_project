function [S_i,T_WC_i] = processFrame(I_i, I_j, S_j, intrinsics)
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
%   I_i         = Image of the ith (current) frame
%   I_j         = Image of the i-1th (previous) frame
%   S_j         = State of the i-1th (previous) frame
%   intrinsics  = cameraIntrinsics object

% Outputs: 
%   S_i       = State of the ith (current) frame
%   T_WC_i    = Pose of the ith (current) frame 

%% Setup
P = S_j.P; % Set of keypoints in the i-1th (previous) frame (Kx2)
X = S_j.X; % Set of landmarks in the i-1th (previous) frame (Kx3)
C = S_j.C; % Set of candidate keypoints in the i-1th (previous) frame (Mx2)
F = S_j.F; % Set of first observation of each candidate keypoints in the i-1th (previous) frame (Mx2)
T = S_j.T; % Set of camera poses for first observation of each candidate keypoints in the i-1th (previous) frame (Mx16)

% K_j = size(P_j,2); % Number of keypoints in the i-1th (previous) frame
% M_j = size(C_j,2); % Number of candidate keypoints in the i-1th (previous) frame

% Initialize state of current frame
S_i = struct;

%% Associate Keypoints to Existing Landmarks
% use KLT and RANSAC to track keypoints
[P, validity] = trackPoints(I_j, I_i, P);

% Update landmark set by only keeping corresponding landmarks of reliably
% tracked keypoints (RANSAC)
X = X(validity, :);
P = P(validity, :);  % Don't forget to remove these too!

%% Estimate the Current Pose
% NOTE: estimateWorldCameraPose takes as input Nx2, Nx3 matrices
% TODO: TEST THIS FUNCTION AND USE OF INTRINSICS
[worldOrientation,worldLocation,~] = estimateWorldCameraPose(P, X, intrinsics);

T_WC_i = [worldOrientation, worldLocation'];
T_WC_i(4, 4) = 1; % Make it a homogeneous transformation matrix

%% Track Candidate Keypoints
[C, validity] = trackPoints(I_j, I_i, C);
% remove lost candidate point data / retain matched candidate point data
F = F(validity, :);
T = T(validity, :);
C = C(validity, :); % Don't forget to remove these too!

%% Triangulating New Landmarks
alpha = 0.1; % Radians
[P_new, X_new] = getNewLandmarks(F, C, T, T_WC_i, intrinsics, alpha);

P = [P ; P_new];
X = [X ; X_new];

%% Store Local Variables in State of Current Frame
S_i.P = P; % Set of keypoints in the ith (current) frame (K'x2)
S_i.X = X; % Set of landmarks in the ith (current) frame (K'x3)
S_i.C = C; % Set of candidate keypoints in the ith (previous) frame (M'x2)
S_i.F = F; % Set of first observation of each candidate keypoints in the ith (current) frame (M'x2)
S_i.T = T; % Set of camera poses for first observation of each candidate keypoints in the ith (current) frame (M'x16)

end


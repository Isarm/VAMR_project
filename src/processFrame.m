function [S_i,T_WC_i] = processFrame(I_i, I_j, S_j, cameraParams)
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
%   I_i       = Image of the ith (current) frame
%   I_j       = Image of the i-1th (previous) frame
%   S_j       = State of the i-1th (previous) frame
%   TODO: Correctly handle cameraParams object

% Outputs: 
%   S_i       = State of the ith (current) frame
%   T_WC_i    = Pose of the ith (current) frame 

%% Setup
P_j = S_j.P; % Set of keypoints in the i-1th (previous) frame (2xK)
X_j = S_j.X; % Set of landmarks in the i-1th (previous) frame (3xK)
C_j = S_j.C; % Set of candidate keypoints in the i-1th (previous) frame (2xM)
F_j = S_j.F; % Set of first observation of each candidate keypoints in the i-1th (previous) frame (2xM)
T_j = S_j.T; % Set of camera poses for first observation of each candidate keypoints in the i-1th (previous) frame (16xM)

% K_j = size(P_j,2); % Number of keypoints in the i-1th (previous) frame
% M_j = size(C_j,2); % Number of candidate keypoints in the i-1th (previous) frame

% Initialize state of current frame
S_i = struct;

%% Associate Keypoints to Existing Landmarks
% Create KLT Point Tracker Object
% TODO: Specify name,value arguments for vision.PointTracker?
KLTPointTracker = vision.PointTracker;

% Initialize Tracking Process
initialize(KLTPointTracker, P_j, I_j)

% Track Points for Current Frame
[points,validity] = tracker(I_i);

% Update keypoint set by only keeping reliably tracked keypoints (RANSAC)
P_i = points(:,validity);

% Update landmark set by only keeping corresponding landmarks of reliably
% tracked keypoints (RANSAC)
X_i = X_j(:,validity);

%% Estimate the Current Pose
% NOTE: estimateWorldCameraPose takes as input Nx2, Nx3 matrices
% TODO: Correctly handle cameraParams object
[worldOrientation,worldLocation,~] = estimateWorldCameraPose(P_i', X_i', cameraParams);

T_WC_i = [worldOrientation, worldLocation];

%% Triangulating New Landmarks
% TODO: Fill code here

%% Store Local Variables in State of Current Frame
S_i.P = P_i; % Set of keypoints in the ith (current) frame (2xK')
S_i.X = X_i; % Set of landmarks in the ith (current) frame (3xK')
S_i.C = C_i; % Set of candidate keypoints in the ith (previous) frame (2xM')
S_i.F = F_i; % Set of first observation of each candidate keypoints in the ith (current) frame (2xM')
S_i.T = T_i; % Set of camera poses for first observation of each candidate keypoints in the ith (current) frame (16xM')

end


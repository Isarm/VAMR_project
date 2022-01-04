function [P_new, X_new, remove] = getNewLandmarks(F, C, T_F, T_C, intrinsics, alpha)
%GETNEWLANDMARKS Gets new 3D landmarks from previously matched points
%   Summary:
%     First it triangulates the 3D locations of matched points. After this
%     it will figure out the angle for each match and check if this angle
%     is large enough. If it is large enough, it will get added to the new
%     landmarks set.
% Inputs: 
%   F       = 2d image location of the first time a point was tracked (Nx2)
%   C       = 2d image location of the current position of a point (Nx2)
%   T_F     = Array of transformation matrices for the frame in which each point was first tracked (Nx16)
%   T_C     = Transformation matrix for the current frame
% Outputs: 
%   P_new   = 2d locations of the new landmarks in the current frame
%   X_new   = 3d locations of new landmarks in the world frame
%   remove = indices of the points in F and C that should be removed, as
%   they have been added to P and X

%% Early return if there is nothing to do
if isempty(C)
    P_new = [];
    X_new = [];
    remove = [];
    return
end

%% First triangulate the 3d landmarks
% We need to invert the homogeneous transformation matrix, as we need the
% matrix that describes the world points wrt the camera, while the current
% matrices describe the camera frame wrt the world frame.
T_C = inv(T_C);
R_C = T_C(1:3, 1:3);
P_C = T_C(1:3, 4);
camMatrix_current = cameraMatrix(intrinsics, R_C, P_C);

N = size(F, 1);
landmarks3D = zeros(N, 3);
valid = false(N, 1);
repr = zeros(N,1);

% This will be kind of slow, so maybe optimize later
K = intrinsics.IntrinsicMatrix';
% Get the camera matrix for the current camera pose
camMatrix_current = (K * T_C(1:3,:))';
% Get all unique rows in T_F
[T_F_uniq, IA, IC] = unique(T_F, 'rows');
% For each unique row 
for i = 1:size(T_F_uniq, 1)
    % Get the indices where it appears
    ids = (i == IC);
    % Get the camera transformation
    T_FW_i = inv(reshape(T_F_uniq(i,:), [4 4]));
    camMatrix_previous = (K * T_FW_i(1:3,:))';
    % Triangualate points
    [landmarks_i, repr_i, valid_i] = triangulate(F(ids,:), C(ids,:), ...
            camMatrix_previous, camMatrix_current);
    % Save values to corresponding positions
    landmarks3D_test(ids, :) = landmarks_i;
    repr(ids) = repr_i;
    valid(ids) = valid_i;
end

%% Figuring out angles
% Now we need to figure out the angle between 2 matched points. 
% First, we need the 2 vectors, described in the world frame, that point
% from each camera's origin to the 3D point. 

% Get the origins
O_F = T_F(:, 13:15)';
O_C = T_C(1:3,4);

% Vectors pointing from origin to 3D points
V_F = landmarks3D' - O_F;
V_C = landmarks3D' - O_C;

% Get the angle. From https://ch.mathworks.com/matlabcentral/answers/16243-angle-between-two-vectors-in-3d
angles = atan2(vecnorm(cross(V_F,V_C)), dot(V_F,V_C))';

%% Only keep entries if the angle is sufficiently large and the reprojection error is small enough
indices = abs(angles) > alpha & valid & (repr < 1);

P_new = C(indices, :);
X_new = landmarks3D(indices, :);

remove = indices;

end

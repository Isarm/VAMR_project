function [P_new, X_new, remove] = getNewLandmarks(F, C, T_F, T_C, intrinsics, parameters)
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
for i = 1:N
    T_FW_i = inv(reshape(T_F(i,:), [4 4]));

    % THIS CODE IMPLEMENTS TRIANGULATE BY HAND (DOES NOT YIELD GOOD RESULTS)
    % M_F = K * T_FW_i(1:3, :);
    % M_C = K * T_C(1:3, :);
    % % Compute landmark
    % Q = [skew([F(i,:) 1]) * M_F;
    %      skew([C(i,:) 1]) * M_C];
    % [U, S, V] = svd(Q);
    % landmarks3D(i,:) = V(1:3,end) ./ V(4,end);
    % % Compute reprojection error
    % p_repr = M_C * [landmarks3D(i,:) 1]';
    % lambda_p = p_repr(end);
    % p_repr = p_repr(1:2) / lambda_p;
    % repr(i) = norm(p_repr' - C(i,:));
    % % Compute validity
    % valid(i) = lambda_p > 0;

    % H = reshape(T_F(i, :), [4 4]);
    % H = inv(H); % Again invert
    % camMatrix_previous = cameraMatrix(intrinsics, H(1:3, 1:3)', H(1:3, 4));

    camMatrix_current = (K * T_C(1:3,:))';
    camMatrix_previous = (K * T_FW_i(1:3,:))';
    [landmarks3D(i,:), repr(i), valid(i)] = triangulate(F(i,:), C(i,:), camMatrix_previous, camMatrix_current);
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
indices = abs(angles) > parameters.alpha & valid & (repr < 1);

P_new = C(indices, :);
X_new = landmarks3D(indices, :);

remove = indices;

end

function s = skew(v)
        s = [0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0 ];
end

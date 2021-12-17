function [P_new, X_new] = getNewLandmarks(F, C, T_F, T_C, intrinsics, alpha)
%GETNEWLANDMARKS Gets new 3D landmarks from previously matched points
%   Summary:
%     First it triangulates the 3D locations of matched points. After this
%     it will figure out the angle for each match and check if this angle
%     is large enough. If it is large enough, it will get added to the new
%     landmarks set.
% Inputs: 
%   F       = 2d image location of the first time a point was tracked (Nx2)
%   C       = 2d image location of the current poition of a point (Nx2)
%   T_F     = Array of transformation matrices for the frame in which each point was first tracked (Nx16)
%   T_C     = Transformation matrix for the current frame
% Outputs: 
%   P_new   = 2d locations of the new landmarks in the current frame
%   X_new   = 3d locations of new landmarks in the world frame

%% Early return if there is nothing to do
if isempty(C)
    P_new = [];
    X_new = [];
    return
end

%% First triangulate the 3d landmarks
camMatrix_current = cameraMatrix(intrinsics, T_C(1:3,1:3), T_C(1:3, 4));
N = size(F, 1);
landmarks3D = zeros(N, 3);

% This will be kind of slow, so maybe optimize later
for i = 1:N
    H = reshape(T_F(i, :), [4 4]);
    camMatrix_previous = cameraMatrix(intrinsics, H(1:3, 1:3), H(1:3, 4));
    landmarks3D(i,:) = triangulate(F(i,:), C(i,:), camMatrix_previous, camMatrix_current); 
    %% TODO: Figure out if the 'valid' part of triangulate has to be used (in case points lie behind the camera)
end

%% Figuring out angles
% Now we need to figure out the angle between 2 matched points. 
% First, we need the 2 vectors, described in the world frame, that point
% from each camera's origin to the 3D point. 

% Get the origins
O_F = T_F(13:15)';
O_C = T_C(1:3,4);

% Vectors pointing from origin to 3D points
V_F = landmarks3D' - O_F;
V_C = landmarks3D' - O_C;

% Get the angle. From https://ch.mathworks.com/matlabcentral/answers/16243-angle-between-two-vectors-in-3d
angles = atan2(norm(cross(V_F,V_C)), dot(V_F,V_C))';

%% Only keep entries if the angle is sufficiently large
indices = abs(angles) > alpha;

P_new = C(indices, :);
X_new = landmarks3D(indices, :);

end


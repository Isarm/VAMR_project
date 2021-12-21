function hidden_state = runBA(hidden_state, observations, K)
%RUNBA runs the bundle adjustment algorithm, finding the landmarks and extrinsic
% parameters that minimize the reprojection error. It does so by finding a local
% optimum of the non linear least squares problem.
% INPUT:
% - hidden_state: unidimensional vector that contains all the camera positions
%                 as 6D vectors and the set of 3D landmarks, all unrolled. This
%                 are all the values that we seek to tune in order to reduce the
%                 reprojection error. 
%                 [pose_1, pose_2, ..., pose_n, l_1, l_2, ..., l_m]
% - observations: unidimensional vector that contains:
%                 [n m O_1, O_2, ..., O_n]. Where n is the number of camera
%                 poses, m is the number of landmarks and O_i is the observation
%                 obtained at pose i and contains:
%                 [k_i, p_1, p_2, ..., p_k, l_1, l_2, ..., l_k], where:
%                   - k_i: is the number of landmarks observed in pose i.
%                   - p_j: 2D position in pixel space of landmark l_j.
%                   - l_j: index pointing to the corresponding 3D landmark in
%                   the set of all 3D landmarks.
% - K: camera projection matrix (3x3)
% OUTPUT:
% - hidden_state: unidimensional vector that contains all the camera positions
%                 landmarks as described previously. It is the result of the 
%                 minimization of the nonlinear least squares problem.

debug = false;
sparse_gradients = true;

% Extract the data
% Define the error function of the BA
error_function = @(x) errorBA(x, observations, K);

if sparse_gradients
        % Get the sparse jacobian filter matrix
        M = getSparseJacobianFilter(hidden_state, observations);

        % Plot the sparsity pattern of M
        if debug
                figure(4);
                spy(M);
        end

        % Tell the optimizer to only compute the gradients that are non-zero
        options = optimoptions('lsqnonlin', ...
                               'JacobPattern', M, ...
                               'Display', 'iter', ...
                               'UseParallel', false, ...
                               'MaxIter', 20);
else
        options = optimoptions('lsqnonlin', ...
                               'Display', 'iter', ...
                               'UseParallel', false, ...
                               'MaxIter', 20);
end

[hidden_state, resnorm] = lsqnonlin(error_function, hidden_state, ...
        [], [], options);

end

function e = errorBA(hidden_state, observations, K)
%ERRORBA given the hidden state, the observations and the camera projection
%matrix, it computes the reprojection error used as the objective of the bundle
%adjustment.
% INPUT:
% - hidden_state: as described in the function RUNBA.
% - observations: as described in the function RUNBA.
% - K: camera projection matrix (3x3)

debug = false;

n = observations(1);
m = observations(2);
taus = hidden_state(1:(n*6));     % Camera twist (obtained from VO)
taus = reshape(taus, 6, n);       % 6xn
P    = hidden_state((n*6+1):end); % 3D Landmarks
P    = reshape(P, 3, m);          % 3xm

% The number of observations is the same as the number of frames
curr_id = 3;
Y = [];
f_x = [];
for f=1:n
        % Get the camera homogeneous matrix in the world frame
        hs_id = (f-1)*6+1;
        T_WC_i = twist2HomogMatrix(taus(:,f));
        T_CW_i = inv(T_WC_i);

        % Get the number of keypoints for this frame
        ki = observations(curr_id);
        % Generate the vector of reference 2D points
        % The observed points are in (row, cols) and they should be in x, y
        aux_y = flipud(reshape(observations(curr_id+1:curr_id+2*ki), 2, ki));
        Y  = [Y; aux_y(:)]; % 2 numbers per point

        % Generate the vector of reprojected 3D points from the camera twist
        l_i = floor(observations(curr_id+1+2*ki:curr_id+3*ki)); % landmark ids
        P_i = P(:,l_i);  % retrieve the seen landmarks by order
        p_i = K * T_CW_i(1:3,:) * [P_i; ones(1, size(P_i, 2))]; % Reprojection
        p_i = p_i(1:2,:) ./ p_i(3,:);
        f_x = [f_x; p_i(:)];

        % DEBUG: This is used to check that the projections make sense
        if debug
                figure(3)
                plot(p_i(1,:), p_i(2,:), 'x');
                hold on;
                plot(aux_y(1,:), aux_y(2,:), 'x');
                hold off;
                axis equal;
                pause(0.01);
        end

        % Update the current id for the next iteration
        curr_id = curr_id+1+3*ki; % 2 numbers + landmark id per point
end

e = f_x - Y;

end

function M = getSparseJacobianFilter(hidden_state, observations)
%GETSPARSEJACOBIANFILTER calculates the sparse matrix that represents all the
%positions where the jacobian used for the computation of the reprojection error
%are not zero.
% INPUT:
% - hidden_state: as described in RUNBA.
% - observations: as described in RUNBA.
% OUTPUT:
% - M: the sparse matrix.

% Populate the M matrix
n = observations(1);
m = observations(2);
taus = hidden_state(1:(n*6));     % Camera twist (obtained from VO)
taus = reshape(taus, 6, n);       % 6xn
P    = hidden_state((n*6+1):end); % 3D Landmarks
P    = reshape(P, 3, m);          % 3xm
numel_obs = (numel(observations)-2-n)/3;
% Prepare the sparse jacobian matrix
M = spalloc(numel_obs, length(hidden_state), 9*numel_obs);

curr_id = 3;
n_obs = 1;
for f=1:n
        ki = observations(curr_id);
        l_i = floor(observations(curr_id+1+2*ki:curr_id+3*ki));

        % Dependence on the camera pose
        M(n_obs:n_obs+ki*2-1, (f-1)*6+1:(f-1)*6+6) = 1;

        % Dependence on the landmark
        for l_id=1:numel(l_i)
                M(n_obs+2*(l_id-1):n_obs-1+l_id*2, ...
                  n*6+3*(l_i(l_id)-1)+1:n*6+3*l_i(l_id)) = 1;
        end
                  
        % Update the current id for the next iteration
        n_obs = n_obs + ki*2;
        curr_id = curr_id+1+3*ki; % 2 numbers + landmark id per point
end

end

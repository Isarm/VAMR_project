%% Test the function getNewLandmarks()

% For now this is just to test for syntax and if matrix dimensions match, it does
% not test if the final functionality is correct. 

[~, ~, intrinsics] = getInitialFrames(0, 0, 1);

alpha = 0.1;

N = 50;

F = floor(rand([N, 2]) .* 100);
C = floor(rand([N, 2]) .* 100);

T = zeros(N + 1, 16);

for i = 1:N + 1
    [Q, ~] = qr(randn(3)); % QR factorization results in uniformy sampled rotation matrix (according to stackexchange)
    Q(1:3, 4) = rand([3 1]);
    Q(4, 4) = 1;
    T(i, :) = Q(:);
end

T_WC_i = reshape(T(end, :), [4 4]);
T(end, :) = [];


[P_new, X_new] = getNewLandmarks(F, C, T, T_WC_i, intrinsics, alpha);


%% Testing some parts of the function
N = 45;

a = 0:N;
x = 1 - tan(a./180 * pi);
P = [1;1;0] * ones(1,N + 1);
O1 = [x ;zeros(1,N + 1); zeros(1, N + 1)];
O2= [1;0;0] * ones(1,N + 1);

V_F = P - O1;
V_C = P - O2;

% Angles should run from 0 to N
angles = atan2(vecnorm(cross(V_F,V_C)), dot(V_F,V_C))'./ pi .* 180


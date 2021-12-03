function M = estimatePoseDLT(p, P, K)
        % Convert pixel coordinates to calibrated coordinates
        p_cal = (K \ [p ones(length(p), 1)]')';
        % p_cal = inv(K) * [p; ones(1, size(p, 2))];

        % Compute the Q matrix
        Q = [];
        for i=1:size(P, 1)
                kron_lh = [1 0 -p_cal(i, 1); 0 1 -p_cal(i, 2)];
                Q = [Q; kron(kron_lh, [P(i,:) 1.0])];
        end

        % Solving the over-determined system of equations
        [U, S, V] = svd(Q);
        m = V(:,end);

        % Reordering in order to obtain the M matrix
        M = reshape(m, [4, 3])';

        % Ensure that the rotation matrix has det=1
        if M(3,4) < 0.0
                M = -1.0 * M;
        end

        % Find the closest orthogonal rotation matrix
        R = M(:, 1:3);
        [U, E, V] = svd(R);
        R_tilde = U * V';

        % Recover the scale of the projection matrix M
        alpha = norm(R_tilde, 'fro') / norm(R, 'fro');

        % Build the M_tilde with the corrected rotation and scale
        M = [R_tilde alpha * M(:, 4)];
end

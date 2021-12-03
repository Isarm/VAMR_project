function p = reprojectPoints(P, M, K)
        % P: [nx3] coordinates of the 3D points
        % M: [3x4] Projection matrix (R and t)
        % K: [3x3] Camera matrix
        % p: [nx2] coordinates of the reprojected 2D points
        p = (K * M * [P'; ones(1,size(P,1))])';
        p = p(:,1:2) ./ p(:,3);
        p = p(:,1:2);
end

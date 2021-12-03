function [R_C_W, t_C_W, best_inlier_mask, max_num_inliers_history, num_iteration_history] ...
    = ransacLocalization(matched_query_keypoints, corresponding_landmarks, K)
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% best_inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.

use_p3p = true;
adaptive = true;

if use_p3p
        num_iterations = 200;
        k = 3;
else
        num_iterations = 2000;
        k = 6;
end

max_inliers = 0;
min_inliers = 30;
threshold = 10;

matched_query_keypoints = flipud(matched_query_keypoints);

max_num_inliers_history = [];
num_iteration_history = [];

best_inlier_mask = zeros(size(matched_query_keypoints, 2), 1);

i = 1;
while i < num_iterations
        % Choose 3 points
        [keypoints, kp_ids] = datasample(matched_query_keypoints, k, 2, 'Replace', false);
        landmarks = corresponding_landmarks(:,kp_ids);

        if use_p3p
                % Construct the model using these points
                normalized_keypoints = K \ [keypoints; ones(1, size(keypoints, 2))];

                for ii=1:3
                        normalized_keypoints(:,ii) = normalized_keypoints(:,ii) / ...
                                norm(normalized_keypoints(:,ii), 2);
                end

                M = p3p(landmarks, normalized_keypoints);
                R_C_W_guess = zeros(3, 3, 4);
                t_C_W_guess = zeros(3, 1, 4);
                % For each solution of the p3p algorithm
                max_inliers_p3p = 0;
                for sol=1:4
                        % Retrieve the M matrix
                        R = real(M(:,((sol-1)*4+2):((sol-1)*4+4)));
                        t = real(M(:,(sol-1)*4+1));
                        R_C_W_guess(:,:,sol) = R';
                        t_C_W_guess(:,:,sol) = -R'*t;
                        M_current = [R_C_W_guess(:,:,sol) t_C_W_guess(:,:,sol)];
                        % Count the inliers
                        reprojected = reprojectPoints(corresponding_landmarks', ...
                                M_current, K)';
                        difference = reprojected - matched_query_keypoints;
                        reprojected_error = sum(difference.^2, 1);
                        inliers = reprojected_error < threshold^2;
                        num_inliers = nnz(inliers);
                        max_inliers_p3p = max(num_inliers, max_inliers_p3p);
                        % Save if the current guess is the best guess
                        if num_inliers > max_inliers && num_inliers >= min_inliers
                                max_inliers = num_inliers;
                                best_inlier_mask = inliers;
                        end
                end
        else % use DLT
                M_C_W_guess = estimatePoseDLT(keypoints', landmarks', K);
                % Count the inliers
                reprojected = reprojectPoints(corresponding_landmarks', ...
                        M_C_W_guess, K)';
                difference = matched_query_keypoints - reprojected;
                reprojected_error = sum(difference.^2, 1);
                inliers = reprojected_error < threshold^2;
                num_inliers = nnz(inliers);
                % Save if the current guess is the best guess
                if num_inliers > max_inliers && num_inliers >= min_inliers
                        max_inliers = num_inliers;
                        best_inlier_mask = inliers;
                end
        end

        % Adaptive RANSAC
        % estimate the outlier ratio
        if adaptive
                if use_p3p
                        num_inliers = max_inliers_p3p
                end
                outlier_ratio = 1 - num_inliers / numel(inliers);
                confidence = 0.95;
                upper_bound_on_outlier_ratio = 0.9;
                outlier_ratio = min(upper_bound_on_outlier_ratio, outlier_ratio);
                num_iterations = log(1-confidence)/log(1-(1-outlier_ratio)^k);
                num_iterations = min(2000, num_iterations);
        end

        max_num_inliers_history(i) = max_inliers;
        num_iteration_history(i) = i;

        i = i + 1;
end

% Rerun DLT to increase the precission
if nnz(best_inlier_mask) > 5
        M = estimatePoseDLT(matched_query_keypoints(:,best_inlier_mask)', ...
                corresponding_landmarks(:,best_inlier_mask)', K);
        % Calculate R and t
        R_C_W = M(1:3, 1:3);
        t_C_W = M(1:3, 4);
else
        disp("Couldn't find more than 5 inliers!");
        R_C_W = eye(3);
        t_C_W = zeros(3, 1);
end
        
end

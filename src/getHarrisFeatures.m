function [F,N] = getHarrisFeatures(img, parameters)
%GETHARRISFEATURES extract HARRIS features and returns the kx2 array of 2D
% keypoints on pixel space.
% INPUT:
% - image
% OUTPUT:
% - keypoints: kx2 array of x-y coordinates

%% Calculate the Harris Features of the image
N = detectHarrisFeatures(img, 'MinQuality', parameters.MinQuality, ...
    'FilterSize', parameters.FilterSize);
% N = detectMinEigenFeatures(imgaussfilt(img), 'MinQuality', 0.001, ...
%         'FilterSize', parameters.FilterSize);

%% Perform non-maxima supression
% Build synthetic image matrix, with only the HARRIS features
UV  = round(N.Location);   % u-v coordinates of all the keypoints
ids = sub2ind(size(img), UV(:,2), UV(:,1));
M   = N.Metric;     % harris scores for each u-v coordinate
harris_scores = zeros(size(img));
harris_scores(ids) = M;
% Perform non-maxima supression in this synthetic image
if parameters.HarrisMaxFeatures > length(M)
    num = length(M);
else
    num = parameters.HarrisMaxFeatures;
end
N = selectKeypoints(harris_scores, num, 10);

% Retrieve the 2D points of the Harris features detected
[F, N] = extractFeatures(img, N);

end

function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.
[h, w] = size(scores);
keypoints = zeros(num, 2);
for i=1:num
        % Find the largest score index
        [~, id] = max(scores(:));
        % Save the new keypoint
        [u, v] = ind2sub([h, w], id);
        keypoints(i, :) = [v, u];
        % Fill the neighbourhood with zeros
        scores([max(1, u-r):min(h, u+r)], ...
               [max(1, v-r):min(w, v+r)]) = 0.0;
end

end

function [F,N] = getHarrisFeatures(img, parameters)
%GETHARRISFEATURES extract HARRIS features and returns the kx2 array of 2D
% keypoints on pixel space.
% INPUT:
% - image
% OUTPUT:
% - keypoints: kx2 array of x-y coordinates

% Calculate the Harris Features of the image
N = detectHarrisFeatures(img, 'MinQuality', parameters.MinQuality, ...
    'FilterSize', parameters.FilterSize);

% Retrieve the 2D points of the Harris features detected
[F, N] = extractFeatures(img, N);

end

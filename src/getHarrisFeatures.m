function N = getHarrisFeatures(img)
%GETHARRISFEATURES extract HARRIS features and returns the kx2 array of 2D
% keypoints on pixel space.
% INPUT:
% - image
% OUTPUT:
% - keypoints: kx2 array of x-y coordinates

% Calculate the Harris Features of the image
% TODO: Tuning MinQuality?
N = detectHarrisFeatures(img, 'MinQuality', 0.01);
% Retrieve the 2D points of the Harris features detected
[~, N] = extractFeatures(img, N);
N = N.Location;

end

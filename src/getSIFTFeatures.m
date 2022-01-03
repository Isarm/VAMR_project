function N = getSIFTFeatures(img, maxpoints)
%GETSIFTFEATURES exctract SIFT features and return the kx2 array of 2D keypoints
% on pixel space.
% INPUT:
% - image
% - maxpoints: pick the strongest points up to maxpoints
% OUTPUT:
% - keypoints: kx2 array of x-y coordinates

if (nargin < 2)
        maxpoints = 500;
end

% Compute the SIFT Features
N = detectSIFTFeatures(img);
N = N.selectStrongest(maxpoints);
% Retrieve the 2D points of the detected SIFT Features
% [~, N] = extractFeatures(img, N);
N = N.Location;

end

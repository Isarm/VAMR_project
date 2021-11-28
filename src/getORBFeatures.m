function [features,validPoints] = getORBFeatures(img)
%GETORBFEATURES extract FAST features and ORB feature descriptors
% chosen for high efficiency, good localization accuracy for detector, and
% pretty great relocalization (see lecture 6)
% INPUT:
% - image
% OUTPUT:
% - features: feature descriptors
% - validPoints: feature locations in the image

    % detects corners using FAST
    imgPoints = detectFASTFeatures(img);
    % extract ORB features, as well as valid points (subset that isn't too
    % close to the edges of the image)
    [features, validPoints] = extractFeatures(img, imgPoints);
end


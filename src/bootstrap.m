function [orbFeatures, landmarks3D, R, T] = bootstrap(img0, img1, intrinsics)
%INTITIALIZE Initialize the VO pipeline
%   Takes 2 initial images, extracts and matches features to determine the
%   pose. The inlier keypoints are then triangulated, and the orb features 
%   with their corresponding 3D locations are then returned.

% Extract ORB features

[img0Features, img0ValidPoints] = getORBFeatures(img0);
[img1Features, img1ValidPoints] = getORBFeatures(img1);

% uses SSD (Sum of Square Differences) by default
matchIndices = matchFeatures(img0Features, img1Features);

% Save only matched points
img0ValidPoints = img0ValidPoints(matchIndices(:,1));
img1ValidPoints = img1ValidPoints(matchIndices(:,2));

% Save the descriptors of the 2nd image (as this is more recent)
orbFeatures = img1Features.Features(matchIndices(:,2), :);

% Estimate essential matrix
[E, inliersIndex, status] = estimateEssentialMatrix(img0ValidPoints, img1ValidPoints, intrinsics);

% Save inliers only
img0ValidPoints = img0ValidPoints(inliersIndex);
img1ValidPoints = img1ValidPoints(inliersIndex);
orbFeatures = orbFeatures(inliersIndex, :);

if status ~= 0
    disp("Error with estimating essential matrix during initialization")
end

% Get camera pose
[R, T] = relativeCameraPose(E, intrinsics, img0ValidPoints, img1ValidPoints);

% Get camera projection matrices
camMatrix0 = cameraMatrix(intrinsics, eye(3), zeros(1,3));
% It wants the pose of camera wrt world points, so invert R and T!
camMatrix1 = cameraMatrix(intrinsics, R', -T * R'); 

% Triangulate 3D landmarks
landmarks3D = triangulate(img0ValidPoints, img1ValidPoints, camMatrix0, camMatrix1);

end


function [points2, validity] = trackPoints(img1, img2, points1)
    % Tracks points between frames using KLT algorithm
    % mainly uses: https://www.mathworks.com/help/vision/ref/vision.pointtracker-system-object.html#d123e87705

    % Inputs:
    %   img1    = image 0
    %   img2    = image2
    %   points1 = (Mx2) pixel coordinates of points found in img1
    
    % Outputs:
    %   points2     = (Nx2) pixel coordinates of point matches found in img2
    %   validity    = (Nx2) logical array indicating valid points

    % track points
    if isempty(points1)
        points2 = [];
        validity = [];
        return
    end

    pointTracker = vision.PointTracker;
    initialize(pointTracker, points1, img1);
    [matchedPoints, validity] = pointTracker(img2);

    % refine using RANSAC (which is built into vision.PointTracker, since it returns the logical array validity)
    % only keeps valid rows
    points2 = matchedPoints(validity, :);
end
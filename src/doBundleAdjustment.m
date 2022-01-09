function [ba] = doBundleAdjustment(ba, i, img, intrinsics, T_WC_i, P, X, validity, ransac_inlier_ids)
    %UNTITLED4 Summary of this function goes here
    %   Detailed explanation goes here

    % Note: FREAK used because extractFeatures cornerPoint objects
    % default uses FREAK method
    [P_F,~] = extractFeatures(img, P, 'Method', 'FREAK');
    ba.vSet = addView(ba.vSet,uint32(i),'Features',P_F,'Points', P);

    % Matches between S_1.P and S_2.P
    % Column 1 is index of matched point in S_1.P
    % Column 2 is index of matched point in S_2.P
    % Initialize that all keypoints in I_1 are matched in the I_2
    matches = repmat(1:size(validity,1),[2,1])';
    matches = updateMatches(matches, validity);
    matches = updateMatches(matches, ransac_inlier_ids);
    ba.vSet = addConnection(ba.vSet,uint32(i-1),uint32(i),'Matches',matches);

    % Keep track only of XYZ that were tracked
    last = cell2mat(ba.xyzPoints(end));
    last = last(validity,:);
    last = last(ransac_inlier_ids,:);
    ba.xyzPoints(end) = {last};
    ba.xyzPointsRefined(end) = {last};
    ba.sizexyzPoints(end) = size(last,1);
    
    % Add new landmarks
    ba.xyzPoints = [ba.xyzPoints; X];
    ba.xyzPointsRefined = [ba.xyzPointsRefined; X];
    ba.sizexyzPoints = [ba.sizexyzPoints; size(X,1)];
        
    % Compute Relative Transformation Between Frames
    ba.cameraPoses.ViewId = [ba.cameraPoses.ViewId; uint32(i)];
    C_R = inv(ba.cameraPoseWC_i.Orientation{end})\T_WC_i(1:3,1:3);
    C_T = T_WC_i(1:3,4)' - ba.cameraPoseWC_i.Location{end};
    
    ba.cameraPoseWC_i.Orientation = [ba.cameraPoseWC_i.Orientation; T_WC_i(1:3,1:3)];
    ba.cameraPoseWC_i.Location = [ba.cameraPoseWC_i.Location; T_WC_i(1:3,4)'];
    
    ba.cameraPoses.Orientation = [ba.cameraPoses.Orientation; C_R];
    ba.cameraPoses.Location = [ba.cameraPoses.Location; C_T];

    if ba.vSet.NumViews > ba.window
        % Build Point Tracks Object 
        window = i-ba.window:i;
        pointTracks = findTracks(ba.vSet, window);
        
        % Number of XYZ points must match number of pointTracks
        xyzPoints_ = cell2mat(ba.xyzPoints(1:end-1));
        
        [xyzPoints, ia, ~] = unique(xyzPoints_, 'stable', 'Rows');
        xyzPointsRefined_ = cell2mat(ba.xyzPointsRefined(1:end-1));
        xyzPointsRefined = xyzPointsRefined_(ia,:);
        
        % Bundle Adjustment
        [xyzRefinedPoints,ba.refinedPoses, ba.reprojectionErrors] = ...
            bundleAdjustment(xyzPointsRefined,pointTracks,struct2table(ba.cameraPoses),intrinsics, ...
            'MaxIterations', 500);
                
        difference = ba.refinedPoses.Location{end}-(ba.cameraPoseWC_i.Location{end-2} + ba.cameraPoses.Location{end-1});
        norm(difference([1,3]))
        
        if norm(difference([1,3])) > 0
            % Store New Points
            xyzPointsAll_ = cell2mat(ba.xyzPoints);
            [xyzPointsAll, ia__, ~] = unique(xyzPointsAll_, 'stable', 'Rows');
            [~, index]=ismember(xyzPoints,xyzPointsAll,'rows');
            xyzPointsAll(index,:) = xyzRefinedPoints;
            xyzPointsAll_(ia__,:) = xyzPointsAll;
            ba.xyzPointsRefined = mat2cell(xyzPointsAll_, ba.sizexyzPoints, 3);

            % Store New Poses
            ba.cameraPoses.Orientation = ba.refinedPoses.Orientation;
            ba.cameraPoses.Location = ba.refinedPoses.Location;
        else
            ba.xyzPointsRefined = ba.xyzPoints;
            disp('help');
        end
        
        % Throw away points outside the window
        ba.xyzPoints = ba.xyzPoints(2:end);
        ba.xyzPointsRefined = ba.xyzPointsRefined(2:end);
        ba.sizexyzPoints = ba.sizexyzPoints(2:end);
        ba.cameraPoses.ViewId = ba.cameraPoses.ViewId(2:end);
        ba.cameraPoses.Orientation = ba.cameraPoses.Orientation(2:end);
        ba.cameraPoses.Location = ba.cameraPoses.Location(2:end); 
        ba.cameraPoseWC_i.Orientation = ba.cameraPoseWC_i.Orientation(2:end);
        ba.cameraPoseWC_i.Location = ba.cameraPoseWC_i.Location(2:end);
    end
end

function newMatches = updateMatches(matches,validity)
    if length(matches) ~= length(validity)
    print('error')
    return;
    end

    % Build Match Matrix based on Validity
    removeMatch = false(length(matches),1);
    offset = 0;
    for i = 1:length(matches)
        if ~validity(i)
            removeMatch(i) = true;
            offset = offset + 1;
        else
            matches(i,2) = matches(i,2) - offset;
        end
    end

    newMatches = matches(~removeMatch,:);
end

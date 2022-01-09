function [] = updateFigure(fig, img, frameNum, imgPoints, candidatePoints, numLandmarksPoint, fullTrajectoryPoints, topViewLandmarkPointsX, topViewLandmarkPointsY, topViewCarPointsX, topViewCarPointsY, groundTruth)
    % UPDATEFIGURE
    % updates plots with data from newest frame

    % Inputs
    %
    %   fig                     = figure to update
    %   img                     = raw frame
    %   frameNum                = frame number
    %   imgPoints               = set of keypoints to project on frame
    %   numLandmarksPoint       = number of landmarks in current frame
    %   fullTrajectoryPoints    = x and y coordinates of new vehicle position (can be batched)
    %   topViewLandmarkPointsX   = cell array to update landmarks to show in last numFrames frames
    %   topViewLandmarkPointsY   = cell array to update landmarks to show in last numFrames frames
    %   topViewCarPointsX        = cell array to update car positions to show in last numFrames frames
    %   topViewCarPointsY        = cell

    % Top left corner: plot current frame and keypoints or whatever else we'd like
    % TODO: Center this image?
    imshow(img, 'Parent', fig.currentFramePlot);
    plot(fig.currentFramePlot, candidatePoints(:,1), candidatePoints(:,2), 'ro')
    hold on
    plot(fig.currentFramePlot, imgPoints(:,1), imgPoints(:,2), 'go');
    title(sprintf('Current Frame: No. %d', frameNum), 'Parent', fig.currentFramePlot);

    % Top right corner: plot 3D landmarks and car position as seen from above in last numFrames frames
    % concatenate cells to get full X and Y sets    
    plot(fig.topViewPlot, topViewCarPointsX, topViewCarPointsY, '-ro');
    hold(fig.topViewPlot, 'on')
    plot(fig.topViewPlot, cell2mat(topViewLandmarkPointsX), cell2mat(topViewLandmarkPointsY), 'ko', 'MarkerSize', 2);
    hold(fig.topViewPlot, 'off')
    legend(fig.topViewPlot, 'Trajectory', 'Landmarks');

    medianY = median(cell2mat(topViewLandmarkPointsY));
    rangeY = std(rmoutliers(cell2mat(topViewLandmarkPointsY)));
    if isnan(medianY) || isnan(rangeY)
        medianY = 0;
        rangeY = 0;
    end

%     ylim(fig.topViewPlot, [medianY - 2 * rangeY - 1, medianY + 2 * rangeY + 1]);

    medianX = median(cell2mat(topViewLandmarkPointsX));
    rangeX = std(rmoutliers(cell2mat(topViewLandmarkPointsX)));
    if isnan(medianX) || isnan(rangeX)
        medianX = 0;
        rangeX = 0;
    end

%     xlim(fig.topViewPlot, [medianX - 2 * rangeX - 1, medianX + 2 * rangeX + 1]);

    % Bottom left corner: number of keypoints tracked over the last numFrames frames
    % uber janky way of getting this to plot from -numFrames to 0
    % TODO: Update this figure
    [x,y] = getpoints(fig.numLandmarksData);
    addpoints(fig.numLandmarksData, x-(max(x)+1), y);
    addpoints(fig.numLandmarksData, x(end)+1, numLandmarksPoint)


    % Bottom right corner: position of vehicle over time?
    addpoints(fig.fullTrajectoryData, fullTrajectoryPoints(:,1), fullTrajectoryPoints(:,2))
    if ~isempty(groundTruth)
        addpoints(fig.groundTruthData, groundTruth(frameNum, 1), groundTruth(frameNum, 3));
%         legend('Calculated Trajectory', 'Ground Truth Trajectory)');
    end
    
    drawnow
end

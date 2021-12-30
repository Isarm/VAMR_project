function [] = updateFigure(fig, img, imgPoints, numLandmarksPoint, fullTrajectoryPoints, topViewLandmarkPointsX, topViewLandmarkPointsY, topViewCarPointsX, topViewCarPointsY)
    % UPDATEFIGURE
    % updates plots with data from newest frame

    % Inputs
    %
    %   fig                     = figure to update
    %   img                     = raw frame
    %   imgPoints               = set of keypoints to project on frame
    %   numLandmarksPoint       = number of landmarks in current frame
    %   fullTrajectoryPoints    = x and y coordinates of new vehicle position (can be batched)
    %   topViewLandmarkPointsX   = cell array to update landmarks to show in last numFrames frames
    %   topViewLandmarkPointsY   = cell array to update landmarks to show in last numFrames frames
    %   topViewCarPointsX        = cell array to update car positions to show in last numFrames frames
    %   topViewCarPointsY        = cell

    % Top left corner: plot current frame and keypoints or whatever else we'd like
    imshow(img, 'Parent', fig.currentFramePlot);
    plot(fig.currentFramePlot, imgPoints(:,1), imgPoints(:,2), 'co');
    

    % Top right corner: plot 3D landmarks and car position as seen from above in last numFrames frames
    % concatenate cells to get full X and Y sets    
    plot(fig.topViewPlot, cell2mat(topViewLandmarkPointsX), cell2mat(topViewLandmarkPointsY), 'ko');
    hold(fig.topViewPlot, 'on')
    plot(fig.topViewPlot, cell2mat(topViewCarPointsX), cell2mat(topViewCarPointsY), 'ro');
    hold(fig.topViewPlot, 'off')

    % Bottom left corner: number of keypoints tracked over the last numFrames frames
    % uber janky way of getting this to plot from -numFrames to 0
    [x,y] = getpoints(fig.numLandmarksData);
    addpoints(fig.numLandmarksData, x-(max(x)+1), y);
    addpoints(fig.numLandmarksData, x(end)+1, numLandmarksPoint)


    % Bottom right corner: position of vehicle over time?
    addpoints(fig.fullTrajectoryData, fullTrajectoryPoints(:,1), fullTrajectoryPoints(:,2))

    drawnow
end
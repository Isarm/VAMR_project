function [fig, topViewLandmarksX, topViewLandmarksY, topViewCarX, topViewCarZ] = createFigure(numPointsNumLandmarks, numFrames)
    %CREATEFIGURE
    % Creates and returns a figure which can be updated with each frame processed

    % Inputs
    %
    %   numPointsNumLandmarks   = number of points to show in the Number of Landmarks plot before sliding the window
    %   numFrames               = number of frame data to show in the top-view plot
    
    % Outputs
    %
    %   fig                 = figure itself
    %   topViewLandmarksX   = cell array to update landmarks to show in last numFrames frames
    %   topViewLandmarksY   = cell array to update landmarks to show in last numFrames frames
    %   topViewCarX        = cell array to update car positions to show in last numFrames frames
    %   topViewCarY        = cell array to update car positions to show in last numFrames frames

    fig = struct;
    fig.f = figure;
    fig.numFrames = numFrames;
    set(gcf,'position',[220,377,1000,600])

    % Top left corner: plot current frame and keypoints or whatever else we'd like
    fig.currentFramePlot = subplot(2,4,[1,2]);
    title('Current Frame: No. 1')
    hold(fig.currentFramePlot, 'on')

    % Top right corner: plot 3D landmarks and car position as seen from above
    fig.topViewPlot = subplot(2,4,[3,4,7,8]);
    title(sprintf('Trajectory of last %d frames and landmarks', numFrames));
    axis equal;
    % each cell contains the values for a given frame
    % since matlab is pass by value only, we need to update these outside this function
    topViewLandmarksX = cell(1,numFrames);
    topViewLandmarksY = cell(1,numFrames);
    topViewCarX = zeros(1,numFrames);
    topViewCarZ = zeros(1,numFrames);
    % landmark data is given in singles
    % coordinates of vehicle are given in doubles (default)
    for i = 1:numFrames
        topViewLandmarksX{i} = single([]);
        topViewLandmarksY{i} = single([]);
    end
    
    % Bottom left corner: number of keypoints tracked over the last numFrames frames
    % TODO: Make this figure more clear
    fig.numLandmarksPlot = subplot(2,4,5);
    fig.numLandmarksData = animatedline('MaximumNumPoints',numPointsNumLandmarks, 'Color', 'g', 'Marker', 'o');
    xlim([-numFrames, 0])
    title(sprintf('Number of landmarks tracked over last %d frames', numFrames));
    addpoints(fig.numLandmarksData, 0, 0)
    axis equal;

    % Bottom right corner: position of vehicle over time?
    fig.fullTrajectoryPlot = subplot(2,4,6);
    fig.fullTrajectoryData = animatedline('Color', 'r', 'Marker', 'o');
    title('Full Trajectory')
end